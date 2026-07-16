"""
Nonholonomic Constraint Node for Ackermann Robot
=================================================
Publishes a fake odometry message with vy=0 (small covariance)
to enforce the Ackermann non-holonomic constraint in EKF.

Topic: /akm_nonholonomic (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class NonholonomicNode(Node):
    def __init__(self):
        super().__init__('nonholonomic_node')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('sigma_vy', 0.01)

        rate = self.get_parameter('publish_rate').value
        self.sigma_vy = self.get_parameter('sigma_vy').value

        self.pub = self.create_publisher(Odometry, '/akm_nonholonomic', 10)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f'Nonholonomic constraint node started (rate={rate}Hz, σ_vy={self.sigma_vy})')

    def timer_callback(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Vy = 0 (Ackermann cannot move sideways)
        msg.twist.twist.linear.y = 0.0

        # Only vy covariance matters; rest set large to be ignored
        msg.twist.covariance[0] = 1e9    # vx (don't constrain)
        msg.twist.covariance[7] = self.sigma_vy ** 2   # vy = 0 ± sigma
        msg.twist.covariance[14] = 1e9   # vz
        msg.twist.covariance[21] = 1e9   # vroll
        msg.twist.covariance[28] = 1e9   # vpitch
        msg.twist.covariance[35] = 1e9   # vyaw

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NonholonomicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
