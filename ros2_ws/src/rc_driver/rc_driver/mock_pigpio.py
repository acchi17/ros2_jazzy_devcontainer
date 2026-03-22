import rclpy.logging

class MockPigpio:
    OUTPUT = 'OUTPUT'

    def __init__(self):
        self.logger = rclpy.logging.get_logger('mock_pigpio')

    def set_mode(self, pin, mode):
        try:
            self.logger.info(f'set_mode(pin={pin}, mode={mode})')
        except Exception:
            pass

    def set_servo_pulsewidth(self, pin, pulse):
        try:
            self.logger.info(f'set_servo_pulsewidth(pin={pin}, pulse={pulse})')
        except Exception:
            pass

    def stop(self):
        try:
            self.logger.info('stop()')
        except Exception:
            pass
