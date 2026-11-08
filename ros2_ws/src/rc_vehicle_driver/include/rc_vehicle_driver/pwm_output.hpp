#pragma once

namespace rc_vehicle_driver
{

// One RC-style PWM output channel (servo or ESC signal line).
class PwmOutput
{
public:
  virtual ~PwmOutput() = default;

  // Returns false on an I/O error.
  virtual bool set_pulse_width_us(double pulse_us) = 0;

  // Stops the signal and frees the hardware resource.
  virtual void release() = 0;
};

// Records the last pulse width only; used when no GPIO hardware is available.
class MockPwmOutput : public PwmOutput
{
public:
  bool set_pulse_width_us(double pulse_us) override
  {
    last_pulse_us = pulse_us;
    return true;
  }
  void release() override {}

  double last_pulse_us = 0.0;
};

}  // namespace rc_vehicle_driver
