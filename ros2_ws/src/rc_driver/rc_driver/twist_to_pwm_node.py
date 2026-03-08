# node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .twist_mapper import TwistMapper
from .pwm_driver import PwmDriver

class TwistToPwmNode(Node):
    def __init__(self):
        super().__init__('twist_to_pwm')

        self.declare_parameter('use_mock_gpio', False)
        use_mock_gpio = self.get_parameter('use_mock_gpio').get_parameter_value().bool_value

        self.mapper = TwistMapper(max_speed=1.0, max_turn=1.0)
        self.esc = PwmDriver(gpio_pin=18, use_mock_gpio=use_mock_gpio)   # 例: ESC
        self.servo = PwmDriver(gpio_pin=19, use_mock_gpio=use_mock_gpio) # 例: ステアリングサーボ

        topic_name = '/cmd_vel'
        qos_depth = 10

        self.subscription = self.create_subscription(
            Twist,
            topic_name,
            self.twist_callback,
            qos_depth
        )

    def twist_callback(self, msg: Twist):
        throttle = self.mapper.twist_to_throttle(msg.linear.x)
        steering = self.mapper.twist_to_steering(msg.angular.z)

        self.get_logger().info(
            f'Received Twist: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f} '
            f'-> throttle={throttle:.3f}, steering={steering:.3f}'
        )

        self.esc.set_value(throttle)
        self.servo.set_value(steering)

    def destroy_node(self):
        self.esc.stop()
        self.servo.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TwistToPwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()