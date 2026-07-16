import base64
import json
import os
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class ImageToLLMNode(Node):
    def __init__(self):
        super().__init__('vision_language_model')

        self.declare_parameter('base_url', 'https://ai-gateway.vei.volces.com/v1')
        self.declare_parameter('api_key', os.environ.get('VOLCENGINE_API_KEY', ''))
        self.declare_parameter('model', 'doubao-vision-lite-32k')
        self.declare_parameter('request_timeout_sec', 30.0)
        self.declare_parameter(
            'prompt',
            '请识别图片内容')

        api_key = self.get_parameter('api_key').value
        self.base_url = self.get_parameter('base_url').value.rstrip('/')
        self.api_key = api_key
        self.model = self.get_parameter('model').value
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.prompt = self.get_parameter('prompt').value
        if not api_key:
            self.get_logger().error('VOLCENGINE_API_KEY is not set; cloud image recognition is disabled')
        self.processing = False

        self.subscription_image = self.create_subscription(
            CompressedImage, '/model_image', self.image_callback, 10)

        self.publisher_ = self.create_publisher(String, '/vision_language_model', 10)

    def encode_image(self, image_data):
        try:
            return base64.b64encode(image_data).decode("utf-8")
        except Exception as e:
            self.get_logger().error(f"Failed to encode image to Base64: {str(e)}")
            return None

    def call_llm(self, base64_image):
        try:
            if not self.api_key:
                raise RuntimeError('VOLCENGINE_API_KEY is not set')
            payload = {
                'model': self.model,
                'messages': [
                    {
                        'role': 'user',
                        'content': [
                            {'type': 'text', 'text': self.prompt},
                            {
                                'type': 'image_url',
                                'image_url': {
                                    'url': f'data:image/jpeg;base64,{base64_image}',
                                },
                            },
                        ],
                    },
                ],
                'max_tokens': 120,
            }
            request = urllib.request.Request(
                f'{self.base_url}/chat/completions',
                data=json.dumps(payload).encode('utf-8'),
                headers={
                    'Authorization': f'Bearer {self.api_key}',
                    'Content-Type': 'application/json',
                },
                method='POST',
            )
            with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
                result = json.loads(response.read().decode('utf-8'))
            response_msg = String()
            response_msg.data = result['choices'][0]['message']['content'].strip()
            self.publisher_.publish(response_msg)
            self.get_logger().info("published to /vision_language_model topic")

        except Exception as e:
            self.get_logger().error(f'Cloud image recognition failed: {e}')
            response_msg = String()
            response_msg.data = "error"
            self.publisher_.publish(response_msg)
        finally:
            self.processing = False

    def image_callback(self, msg: CompressedImage):
        if not msg.data or self.processing:
            return
        self.processing = True

        response_msg = String()
        response_msg.data = "正在识别"
        self.publisher_.publish(response_msg)

        base64_image = self.encode_image(msg.data)

        if not base64_image:
            self.get_logger().error("Image encoding failed, skipping processing")
            self.processing = False
            return

        self.call_llm(base64_image)


def main(args=None):
    rclpy.init(args=args)
    image_to_llm_node = ImageToLLMNode()
    try:
        rclpy.spin(image_to_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        image_to_llm_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()