class PwmDriver:
    def __init__(self, gpio_pin, use_mock_gpio=False):
        self.gpio_pin = gpio_pin
        if use_mock_gpio:
            from rc_driver.mock_pigpio import MockPigpio
            self.pi = MockPigpio()
            self.pi.set_mode(self.gpio_pin, MockPigpio.OUTPUT)
        else:
            import pigpio  # type: ignore
            self.pi = pigpio.pi()
            self.pi.set_mode(self.gpio_pin, pigpio.OUTPUT)

    def set_value(self, value):
        # value: -1.0〜+1.0
        pulse = 1500 + value * 500
        self.pi.set_servo_pulsewidth(self.gpio_pin, pulse)

    def stop(self):
        self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
        self.pi.stop()