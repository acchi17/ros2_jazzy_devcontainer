class MockPigpio:
    OUTPUT = 'OUTPUT'

    def __init__(self):
        pass

    def set_mode(self, pin, mode):
        print(f'[MockGPIO] set_mode(pin={pin}, mode={mode})')

    def set_servo_pulsewidth(self, pin, pulse):
        print(f'[MockGPIO] set_servo_pulsewidth(pin={pin}, pulse={pulse})')

    def stop(self):
        print('[MockGPIO] stop()')
