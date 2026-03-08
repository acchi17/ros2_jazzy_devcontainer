class TwistMapper:
    def __init__(self, max_speed=1.0, max_turn=1.0):
        self.max_speed = max_speed
        self.max_turn = max_turn

    def twist_to_throttle(self, linear_x):
        return max(min(linear_x / self.max_speed, 1.0), -1.0)

    def twist_to_steering(self, angular_z):
        return max(min(angular_z / self.max_turn, 1.0), -1.0)