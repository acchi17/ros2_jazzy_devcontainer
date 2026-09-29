from gpiozero import Servo


class PwmDriver:
    def __init__(
        self,
        gpio_pin,
        use_mock_gpio=False,
        min_pulse_width=1 / 1000,
        max_pulse_width=2 / 1000,
        frame_width=20 / 1000,
    ):
        pin_factory = None
        if use_mock_gpio:
            from gpiozero.pins.mock import MockFactory, MockPWMPin
            pin_factory = MockFactory(pin_class=MockPWMPin)
        self.servo = Servo(
            gpio_pin,
            min_pulse_width=min_pulse_width,
            max_pulse_width=max_pulse_width,
            frame_width=frame_width,
            pin_factory=pin_factory,
        )

    def set_value(self, value):
        # value: -1.0〜+1.0
        self.servo.value = value

    def stop(self):
        self.servo.value = None
        self.servo.close()
