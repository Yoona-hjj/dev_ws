import os
import re
import subprocess
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


os.environ.setdefault("DISPLAY", ":0")

XAUTHORITY_PATH = "/home/sunrise/.Xauthority"
if os.path.exists(XAUTHORITY_PATH):
    os.environ["XAUTHORITY"] = XAUTHORITY_PATH


def get_screen_resolution_xorg():
    try:
        output = subprocess.check_output(
            ["xrandr", "--display", os.environ["DISPLAY"], "--current"],
            stderr=subprocess.STDOUT,
            env=os.environ,
        ).decode("utf-8", errors="ignore")
    except Exception as exc:
        print("Failed to read screen resolution:", exc)
        return None, None

    match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
    if not match:
        return None, None
    return int(match.group(1)), int(match.group(2))


def find_display_font():
    font_paths = [
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/usr/share/fonts/truetype/arphic/ukai.ttc",
        "C:/Windows/Fonts/msyh.ttc",
        "C:/Windows/Fonts/simhei.ttf",
    ]
    for path in font_paths:
        if os.path.exists(path):
            return path
    return None


def text_size(draw, text, font):
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0], bbox[3] - bbox[1]


def wrap_text(draw, text, font, max_width):
    lines = []
    for paragraph in text.splitlines() or [""]:
        if not paragraph:
            lines.append("")
            continue
        current = ""
        for char in paragraph:
            candidate = current + char
            width, _ = text_size(draw, candidate, font)
            if width <= max_width:
                current = candidate
            else:
                if current:
                    lines.append(current)
                current = char
        if current:
            lines.append(current)
    return lines


def create_text_image(width, height, title_text, body_text):
    font_path = find_display_font()
    if font_path is None:
        print("Chinese font not found; Chinese text may not display correctly")

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    margin_x = int(width * 0.04)
    margin_y = int(height * 0.05)
    max_text_width = width - margin_x * 2
    max_text_height = height - margin_y * 2

    def load_font(size):
        if font_path:
            return ImageFont.truetype(font_path, size)
        return ImageFont.load_default()

    def measure(body_size):
        title_size = max(16, int(body_size * 1.65))
        title_font = load_font(title_size)
        body_font = load_font(body_size)
        body_lines = wrap_text(draw, body_text, body_font, max_text_width) if body_text else []
        title_w, title_h = text_size(draw, title_text, title_font)
        line_h = int(body_size * 1.35)
        gap = int(body_size * 1.1) if body_lines else 0
        total_h = title_h + gap + line_h * len(body_lines)
        widest = max([title_w] + [text_size(draw, line, body_font)[0] for line in body_lines if line])
        return title_font, body_font, body_lines, title_w, title_h, line_h, gap, total_h, widest

    low, high = 12, max(16, int(min(width, height) * 0.22))
    best = None
    while low <= high:
        mid = (low + high) // 2
        layout = measure(mid)
        fits = layout[7] <= max_text_height and layout[8] <= max_text_width
        if fits:
            best = layout
            low = mid + 1
        else:
            high = mid - 1

    if best is None:
        best = measure(12)

    title_font, body_font, body_lines, title_w, title_h, line_h, gap, total_h, _ = best
    y = max(margin_y, (height - total_h) // 2)
    draw.text(((width - title_w) // 2, y), title_text, fill="black", font=title_font)
    y += title_h + gap

    for line in body_lines:
        line_w, _ = text_size(draw, line, body_font) if line else (0, 0)
        draw.text(((width - line_w) // 2, y), line, fill="black", font=body_font)
        y += line_h

    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)


class RaceStatusDisplayNode(Node):
    def __init__(self, width, height):
        super().__init__("race_status_display")
        self.width = width
        self.height = height
        self.qrcode_direction = ""
        self.qrcode_number = ""
        self.model_result = ""
        self.last_display_state = None
        self.last_start_button_state = False
        self.result_ignore_until = 0.0
        self.screen_needs_refresh = True

        self.qrcode_subscription = self.create_subscription(
            String, "/qrcode_result", self.qrcode_callback, 10
        )
        self.vision_language_model_subscription = self.create_subscription(
            String, "/vision_language_model", self.vision_language_model_callback, 10
        )
        self.chassis_start_button_subscription = self.create_subscription(
            Bool, "/chassis_start_button", self.chassis_start_button_callback, 10
        )

        self.text_img = create_text_image(
            self.width, self.height, "Display ready, waiting for input...", ""
        )
        self.display_timer = self.create_timer(0.1, self.refresh_window)

        self.get_logger().info("race_status_display started")
        self.get_logger().info("Subscribed: /qrcode_result")
        self.get_logger().info("Subscribed: /vision_language_model")

    def reset_display_state_by_button(self):
        self.result_ignore_until = time.monotonic() + 1.0
        self.qrcode_direction = ""
        self.qrcode_number = ""
        self.model_result = ""
        self.last_display_state = None
        self.update_display(force=True)

    def chassis_start_button_callback(self, msg):
        button_pressed = bool(msg.data)
        rising_edge = button_pressed and not self.last_start_button_state
        self.last_start_button_state = button_pressed
        if rising_edge:
            self.reset_display_state_by_button()

    def qrcode_callback(self, msg):
        data = msg.data.strip()
        if not data:
            return
        if data == "ClockWise":
            self.qrcode_direction = "QR clockwise"
        elif data == "AntiClockWise":
            self.qrcode_direction = "QR anticlockwise"
        elif data.isdigit():
            self.qrcode_number = data
        else:
            self.qrcode_direction = data
        self.update_display()

    def vision_language_model_callback(self, msg):
        data = msg.data.strip()
        if not data:
            return
        if data == "start":
            self.model_result = ""
        elif time.monotonic() >= self.result_ignore_until:
            self.model_result = data
        self.update_display()

    def build_title(self):
        if self.qrcode_direction or self.qrcode_number:
            return f"{self.qrcode_direction} {self.qrcode_number}".strip()
        return "Waiting for QR code..."

    def update_display(self, force=False):
        state = (self.build_title(), self.model_result)
        if not force and state == self.last_display_state:
            return
        self.last_display_state = state
        self.text_img = create_text_image(self.width, self.height, state[0], state[1])
        self.screen_needs_refresh = True

    def refresh_window(self):
        if not self.screen_needs_refresh:
            cv2.waitKey(1)
            return
        cv2.namedWindow("race_status_display", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            "race_status_display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )
        cv2.imshow("race_status_display", self.text_img)
        cv2.waitKey(1)
        self.screen_needs_refresh = False


def main(args=None):
    rclpy.init(args=args)

    width, height = get_screen_resolution_xorg()
    if width is None or height is None:
        print("Unable to get screen resolution; fallback to 1280x720")
        width, height = 1280, 720

    node = RaceStatusDisplayNode(width, height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
