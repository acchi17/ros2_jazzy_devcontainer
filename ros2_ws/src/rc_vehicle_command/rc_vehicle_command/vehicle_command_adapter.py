import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from .feasibility import make_feasible


class VehicleCommandAdapter(Node):
    def __init__(self):
        super().__init__('vehicle_command_adapter')

        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/bicycle_steering_controller/reference')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value

        self.publisher = self.create_publisher(
            TwistStamped, self.get_parameter('output_topic').value, 10)
        self.subscription = self.create_subscription(
            Twist, self.get_parameter('input_topic').value, self.twist_callback, 10)

    def twist_callback(self, msg: Twist):
        v, omega = make_feasible(
            msg.linear.x, msg.angular.z,
            self.wheelbase, self.max_steering_angle, self.min_linear_speed)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist.linear.x = v
        out.twist.angular.z = omega
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
