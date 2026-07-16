"""
ZUPT (Zero-velocity Update) Monitor Node
==========================================
When /robot_static is True, publishes strong zero-velocity observations
to EKF to prevent yaw and position drift while stationary.

Subscribes: /robot_static (std_msgs/Bool)
Publishes:  /zupt_obs     (nav_msgs/Odometry) - vx=0, vy=0, ωz=0
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class ZuptMonitor(Node):
    def __init__(self):
        super().__init__('zupt_monitor')

        self.declare_parameter('sigma_v', 1e-3)
        self.declare_parameter('sigma_omega', 1e-3)

        self.sigma_v = self.get_parameter('sigma_v').value
        self.sigma_omega = self.get_parameter('sigma_omega').value

        self.sub = self.create_subscription(
            Bool, '/robot_static', self.static_callback, 10)
        self.pub = self.create_publisher(Odometry, '/zupt_obs', 10)

        self.get_logger().info(
            f'ZUPT monitor started (σ_v={self.sigma_v}, σ_ω={self.sigma_omega})')

    def static_callback(self, msg: Bool):
        if not msg.data:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Zero velocity observation
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Very tight covariance (strong constraint)
        odom.twist.covariance[0] = self.sigma_v ** 2      # vx
        odom.twist.covariance[7] = self.sigma_v ** 2      # vy
        odom.twist.covariance[14] = 1e-9                  # vz
        odom.twist.covariance[21] = 1e9                   # vroll (unused)
        odom.twist.covariance[28] = 1e9                   # vpitch (unused)
        odom.twist.covariance[35] = self.sigma_omega ** 2 # vyaw

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ZuptMonitor()
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
