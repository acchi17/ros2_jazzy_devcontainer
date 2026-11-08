#pragma once

#include <algorithm>
#include <cmath>

namespace rc_vehicle_driver
{

// Positive steering angle is a left turn (REP-103, counter-clockwise from above).
struct ServoCalibration
{
  double center_us;
  double left_us;   // pulse width at +max_angle
  double right_us;  // pulse width at -max_angle
  double max_angle; // [rad]

  double to_pulse_us(double angle) const
  {
    if (!std::isfinite(angle)) {
      return center_us;
    }
    const double ratio = std::clamp(angle / max_angle, -1.0, 1.0);
    return ratio >= 0.0 ? center_us + (left_us - center_us) * ratio
                        : center_us + (center_us - right_us) * ratio;
  }
};

// Positive velocity is forward.
struct EscCalibration
{
  double neutral_us;
  double forward_us;   // pulse width at +max_velocity
  double reverse_us;   // pulse width at -max_velocity
  double max_velocity; // [rad/s]

  double to_pulse_us(double velocity) const
  {
    if (!std::isfinite(velocity)) {
      return neutral_us;
    }
    const double ratio = std::clamp(velocity / max_velocity, -1.0, 1.0);
    return ratio >= 0.0 ? neutral_us + (forward_us - neutral_us) * ratio
                        : neutral_us + (neutral_us - reverse_us) * ratio;
  }
};

}  // namespace rc_vehicle_driver
