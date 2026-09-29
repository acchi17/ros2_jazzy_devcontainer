# node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .twist_mapper import TwistMapper
from .pwm_driver import PwmDriver

# GPIO pin numbers
ESC_GPIO_PIN = 18
SERVO_GPIO_PIN = 17

# ESC pulse width spec (gpiozero default values: 1ms-2ms, 50Hz)
ESC_MIN_PULSE_WIDTH = 1 / 1000
ESC_MAX_PULSE_WIDTH = 2 / 1000
ESC_FRAME_WIDTH = 20 / 1000

# SG90-compatible servo pulse width spec (datasheet: 500us-2400us, 50Hz)
SG90_MIN_PULSE_WIDTH = 0.5 / 1000
SG90_MAX_PULSE_WIDTH = 2.4 / 1000
SG90_FRAME_WIDTH = 20 / 1000

class TwistToPwmNode(Node):
    def __init__(self):
        super().__init__('twist_to_pwm')

        self.declare_parameter('use_mock_gpio', False)
        use_mock_gpio = self.get_parameter('use_mock_gpio').get_parameter_value().bool_value

        self.mapper = TwistMapper(max_speed=1.0, max_turn=1.0)
        self.esc = PwmDriver(
            gpio_pin=ESC_GPIO_PIN,
            use_mock_gpio=use_mock_gpio,
            min_pulse_width=ESC_MIN_PULSE_WIDTH,
            max_pulse_width=ESC_MAX_PULSE_WIDTH,
            frame_width=ESC_FRAME_WIDTH,
        )
        self.servo = PwmDriver(
            gpio_pin=SERVO_GPIO_PIN,
            use_mock_gpio=use_mock_gpio,
            min_pulse_width=SG90_MIN_PULSE_WIDTH,
            max_pulse_width=SG90_MAX_PULSE_WIDTH,
            frame_width=SG90_FRAME_WIDTH,
        )

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
        #rclpy.shutdown()
        # Use a safe shutdown to avoid exceptions if SIGINT already triggered shutdown
        try:
            # Prefer try_shutdown when available (ROS 2 Jazzy+); fall back to guard
            if hasattr(rclpy, 'try_shutdown'):
                rclpy.try_shutdown()
            else:
                if rclpy.ok():
                    rclpy.shutdown()
        except Exception:
            # Ensure process exits cleanly even if shutdown was already called
            pass

if __name__ == '__main__':
    main()