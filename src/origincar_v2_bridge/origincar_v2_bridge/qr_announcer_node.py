"""
QR Code Voice Announcer (Chinese TTS over USB speaker)
======================================================
Subscribes to the QR result topic and announces it in Mandarin Chinese
through a USB speaker using espeak-ng + aplay (fully offline).

Rules:
  - Numeric QR (e.g. "5", "12"): announce the number, then
      odd  -> "顺时针" (clockwise)
      even -> "逆时针" (counter-clockwise)
  - "ClockWise"      -> announce "顺时针"
  - "AntiClockWise"  -> announce "逆时针"

Subscriptions:
  /zbar_number  (std_msgs/String) - QR payload from the qrcode node

Parameters:
  audio_device     (str)   ALSA device of the USB speaker, e.g. "plughw:2,0"
  voice            (str)   espeak-ng voice, default "cmn" (Mandarin)
  speed            (int)   espeak-ng words-per-minute, default 150
  repeat_cooldown  (float) seconds to suppress re-announcing the same value
"""

import os
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


# --- Integer -> Chinese numerals (supports 0..9999) ---
_CN_DIGITS = ['零', '一', '二', '三', '四', '五', '六', '七', '八', '九']
_CN_UNITS = ['', '十', '百', '千']


def int_to_chinese(n: int) -> str:
    """Convert an integer 0..9999 to natural Chinese numerals."""
    if n == 0:
        return _CN_DIGITS[0]
    if n < 0:
        return '负' + int_to_chinese(-n)
    if n > 9999:
        # Fall back to per-digit reading for very large values.
        return ''.join(_CN_DIGITS[int(d)] for d in str(n))

    digits = [int(d) for d in str(n)]
    length = len(digits)
    out = []
    zero_pending = False
    for i, d in enumerate(digits):
        unit = _CN_UNITS[length - 1 - i]
        if d == 0:
            zero_pending = True
            continue
        if zero_pending:
            out.append(_CN_DIGITS[0])
            zero_pending = False
        out.append(_CN_DIGITS[d] + unit)
    text = ''.join(out)
    # "一十二" -> "十二" (natural reading for 10..19)
    if text.startswith('一十'):
        text = text[1:]
    return text


class QrAnnouncerNode(Node):
    def __init__(self):
        super().__init__('qr_announcer_node')

        # 'auto' -> resolve the USB speaker card by name at runtime. Set an
        # explicit ALSA device (e.g. 'plughw:1,0') to override.
        self.declare_parameter('audio_device', 'auto')
        self.declare_parameter('device_name_match', 'USB Audio Device')
        self.declare_parameter('voice', 'cmn')
        self.declare_parameter('speed', 190)          # espeak-ng words-per-minute (higher = faster)
        self.declare_parameter('amplitude', 200)       # espeak-ng volume 0..200
        self.declare_parameter('word_gap', 8)          # pause between words (x10ms)
        self.declare_parameter('pitch', 50)            # 0..99
        self.declare_parameter('repeats', 1)           # times to repeat each phrase
        self.declare_parameter('set_mixer_max', True)  # raise ALSA volume at start
        self.declare_parameter('repeat_cooldown', 5.0)

        self.audio_device = self.get_parameter('audio_device').value
        self.device_name_match = self.get_parameter('device_name_match').value
        self.voice = self.get_parameter('voice').value
        self.speed = int(self.get_parameter('speed').value)
        self.amplitude = int(self.get_parameter('amplitude').value)
        self.word_gap = int(self.get_parameter('word_gap').value)
        self.pitch = int(self.get_parameter('pitch').value)
        self.repeats = max(1, int(self.get_parameter('repeats').value))
        self.repeat_cooldown = float(self.get_parameter('repeat_cooldown').value)

        self._last_text = None
        self._last_time = 0.0
        self._busy = threading.Lock()

        # Resolve the ALSA device (auto-detect USB speaker card if requested).
        if not self.audio_device or self.audio_device.lower() == 'auto':
            card = self._find_usb_card()
            if card is not None:
                self.audio_device = f'plughw:{card},0'
                self.get_logger().info(
                    f'Auto-detected USB speaker on card {card} '
                    f'("{self.device_name_match}") -> {self.audio_device}')
            else:
                self.audio_device = 'plughw:1,0'
                self.get_logger().warn(
                    f'USB speaker "{self.device_name_match}" not found; '
                    f'falling back to {self.audio_device}')

        if self.get_parameter('set_mixer_max').value:
            self._maximize_volume()

        self.sub = self.create_subscription(
            String, '/zbar_number', self.qr_callback, 10)
        self.vlm_sub = self.create_subscription(
            String, '/vlm_to_vlm', self.vlm_callback, 10)

        self.get_logger().info(
            f'QR announcer ready (device={self.audio_device}, voice={self.voice}, '
            f'speed={self.speed}, amplitude={self.amplitude})')

    def _find_usb_card(self):
        """Return the ALSA card index whose description matches device_name_match.

        Parses /proc/asound/cards, e.g. line:
          ' 1 [Device         ]: USB-Audio - USB Audio Device'
        Matching on the description avoids picking the USB camera's audio.
        """
        try:
            with open('/proc/asound/cards') as f:
                content = f.read()
        except OSError:
            return None
        match = (self.device_name_match or '').lower()
        for line in content.splitlines():
            stripped = line.strip()
            if not stripped or not stripped[0].isdigit():
                continue
            idx = stripped.split()[0]
            # Description is on this line after ']:'; also check the next-line tail.
            desc = line.split(']:', 1)[-1].lower()
            if match and match in desc:
                return idx
        return None

    def _card_index(self):
        """Extract the ALSA card index from audio_device like 'plughw:1,0'."""
        try:
            return self.audio_device.split(':', 1)[1].split(',', 1)[0]
        except (IndexError, AttributeError):
            return None

    def _maximize_volume(self):
        """Best-effort: set the USB card's playback volume to max & unmute."""
        card = self._card_index()
        if card is None:
            return
        for ctrl in ('PCM', 'Speaker', 'Master', 'Headphone'):
            try:
                subprocess.run(
                    ['amixer', '-c', card, 'sset', ctrl, '80%', 'unmute'],
                    check=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().info(f'Set ALSA card {card} "{ctrl}" to 100%')
            except (subprocess.CalledProcessError, FileNotFoundError):
                pass

    def qr_callback(self, msg: String):
        raw = (msg.data or '').strip()
        if not raw:
            return

        text = self._build_text(raw)
        if text is None:
            self.get_logger().warn(f'Unrecognized QR payload: "{raw}"')
            return

        now = time.time()
        if text == self._last_text and (now - self._last_time) < self.repeat_cooldown:
            return
        self._last_text = text
        self._last_time = now

        self.get_logger().info(f'QR "{raw}" -> announce: {text}')
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def vlm_callback(self, msg: String):
        text = (msg.data or '').strip()
        if not text:
            return
        if text.lower() == 'error':
            text = '图像识别失败'
        self.get_logger().info(f'VLM announce: {text}')
        threading.Thread(target=self._speak, args=(text,), daemon=True).start()

    def _build_text(self, raw: str):
        """Map a QR payload to the Chinese sentence to speak."""
        low = raw.lower()
        if low == 'clockwise':
            return '识别到，顺时针'
        if low == 'anticlockwise':
            return '识别到，逆时针'
        try:
            number = int(raw)
        except ValueError:
            return None
        cn = int_to_chinese(number)
        direction = '顺时针' if (number % 2 == 1) else '逆时针'
        return f'识别到数字{cn}，{direction}'

    def _aplay(self, wav_path: str):
        """Play a wav, retrying once if the device is transiently busy."""
        for attempt in range(2):
            try:
                subprocess.run(
                    ['aplay', '-D', self.audio_device, wav_path],
                    check=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return
            except subprocess.CalledProcessError:
                if attempt == 0:
                    time.sleep(0.3)  # device may be briefly busy
                    continue
                raise

    def _speak(self, text: str):
        """Synthesize with espeak-ng and play through the USB speaker."""
        # Serialize playback so overlapping QR reads don't talk over each other.
        with self._busy:
            wav_path = None
            try:
                fd, wav_path = tempfile.mkstemp(suffix='.wav', prefix='qr_tts_')
                os.close(fd)
                subprocess.run(
                    ['espeak-ng', '-v', self.voice,
                     '-s', str(self.speed),
                     '-a', str(self.amplitude),
                     '-g', str(self.word_gap),
                     '-p', str(self.pitch),
                     '-w', wav_path, text],
                    check=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                for _ in range(self.repeats):
                    self._aplay(wav_path)
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f'TTS playback failed: {e}')
            except FileNotFoundError as e:
                self.get_logger().error(f'Missing espeak-ng/aplay: {e}')
            finally:
                if wav_path and os.path.exists(wav_path):
                    try:
                        os.remove(wav_path)
                    except OSError:
                        pass


def main(args=None):
    rclpy.init(args=args)
    node = QrAnnouncerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
