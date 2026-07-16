import base64
import os
import threading
import time

import rclpy
from openai import OpenAI
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


DEFAULT_BASE_URL = (
    "https://ws-0hnuudqllbtcwnc7.cn-beijing.maas.aliyuncs.com/"
    "api/v2/apps/protocols/compatible-mode/v1"
)
DEFAULT_MODEL = "qwen3.6-flash"


class SceneVlmClientNode(Node):
    def __init__(self):
        super().__init__("scene_vlm_client")

        self.declare_parameter("model_name", DEFAULT_MODEL)
        self.declare_parameter("base_url", DEFAULT_BASE_URL)
        self.declare_parameter("timeout", 15.0)
        self.declare_parameter("warmup_count", 3)

        self.model_name = self.get_parameter("model_name").value
        self.warmup_count = int(self.get_parameter("warmup_count").value)

        api_key = os.getenv("DASHSCOPE_API_KEY", "")
        self.client = OpenAI(
            api_key=api_key,
            base_url=self.get_parameter("base_url").value,
            timeout=float(self.get_parameter("timeout").value),
            max_retries=1,
        )

        self.subscription_image = self.create_subscription(
            CompressedImage,
            "/model_image",
            self.image_callback,
            10,
        )
        self.publisher_ = self.create_publisher(String, "/vision_language_model", 10)
        self.chassis_start_button_sub = self.create_subscription(
            Bool,
            "/chassis_start_button",
            self.chassis_start_button_callback,
            10,
        )

        self.last_start_button_state = False
        self.reset_generation = 0
        self.model_busy = False
        self.model_busy_lock = threading.Lock()
        self.warmup_timer = self.create_timer(1.0, self.warmup_once)

        self.get_logger().info("scene_vlm_client started")
        if not api_key:
            self.get_logger().warning(
                "DASHSCOPE_API_KEY is empty; model requests will fail until it is set"
            )

    def reset_model_state_by_button(self):
        self.reset_generation += 1
        reset_msg = String()
        reset_msg.data = "start"
        self.publisher_.publish(reset_msg)
        self.get_logger().info("scene VLM state reset by chassis start button")

    def chassis_start_button_callback(self, msg):
        button_pressed = bool(msg.data)
        rising_edge = button_pressed and not self.last_start_button_state
        self.last_start_button_state = button_pressed
        if rising_edge:
            self.reset_model_state_by_button()

    def warmup_once(self):
        self.warmup_timer.cancel()
        if self.warmup_count <= 0:
            return
        threading.Thread(target=self.run_warmup, daemon=True).start()

    def run_warmup(self):
        if not self.try_lock_model():
            return

        try:
            start = time.perf_counter()
            for index in range(self.warmup_count):
                try:
                    step_start = time.perf_counter()
                    response = self.client.responses.create(
                        model=self.model_name,
                        input=[
                            {
                                "role": "user",
                                "content": [
                                    {
                                        "type": "input_text",
                                        "text": f"Warmup request {index + 1}. Reply ok.",
                                    }
                                ],
                            }
                        ],
                        max_output_tokens=20,
                        extra_body={"enable_thinking": False},
                    )
                    result_text = self.extract_response_text(response) or "no response"
                    elapsed = time.perf_counter() - step_start
                    self.get_logger().info(
                        f"warmup {index + 1}/{self.warmup_count}: {result_text}, {elapsed:.3f}s"
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"warmup {index + 1}/{self.warmup_count} failed: {exc}"
                    )

                if index < self.warmup_count - 1:
                    time.sleep(0.5)

            total_elapsed = time.perf_counter() - start
            self.get_logger().info(
                f"warmup finished: {self.warmup_count} attempts, {total_elapsed:.3f}s"
            )
        finally:
            self.unlock_model()

    def image_callback(self, msg):
        if not msg.data:
            return
        if not self.try_lock_model():
            self.get_logger().warning("model is busy; dropped current image")
            return

        image_bytes = bytes(msg.data)
        generation = self.reset_generation
        threading.Thread(
            target=self.process_image_request,
            args=(image_bytes, generation),
            daemon=True,
        ).start()

    def process_image_request(self, image_bytes, generation):
        try:
            start_msg = String()
            start_msg.data = "start"
            self.publisher_.publish(start_msg)

            image_base64 = base64.b64encode(image_bytes).decode("utf-8")
            start = time.perf_counter()
            response = self.client.responses.create(
                model=self.model_name,
                input=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "input_text",
                                "text": "Describe the key traffic or race scene information in this image.",
                            },
                            {
                                "type": "input_image",
                                "image_url": f"data:image/jpeg;base64,{image_base64}",
                            },
                        ],
                    }
                ],
                max_output_tokens=100,
                extra_body={"enable_thinking": False},
            )

            result_text = self.extract_response_text(response) or "no response"
            elapsed = time.perf_counter() - start
            self.get_logger().info(f"model result: {result_text}")
            self.get_logger().info(f"model elapsed: {elapsed:.3f}s")

            if generation == self.reset_generation:
                response_msg = String()
                response_msg.data = result_text
                self.publisher_.publish(response_msg)
            else:
                self.get_logger().info("discarded stale VLM result after reset")
        except Exception as exc:
            self.get_logger().error(f"model request failed: {exc}")
            response_msg = String()
            response_msg.data = "model request failed"
            self.publisher_.publish(response_msg)
        finally:
            self.unlock_model()

    def extract_response_text(self, response):
        output_text = getattr(response, "output_text", None)
        if output_text:
            return output_text.strip()

        chunks = []
        for item in getattr(response, "output", []) or []:
            if getattr(item, "type", None) != "message":
                continue
            for content in getattr(item, "content", []) or []:
                text = getattr(content, "text", None)
                if text:
                    chunks.append(text)
        return "".join(chunks).strip()

    def try_lock_model(self):
        with self.model_busy_lock:
            if self.model_busy:
                return False
            self.model_busy = True
            return True

    def unlock_model(self):
        with self.model_busy_lock:
            self.model_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = SceneVlmClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
