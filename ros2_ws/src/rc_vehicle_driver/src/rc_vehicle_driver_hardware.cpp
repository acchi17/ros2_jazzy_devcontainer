#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rc_vehicle_driver/pwm_calibration.hpp"
#include "rc_vehicle_driver/pwm_output.hpp"

#ifdef RC_VEHICLE_DRIVER_HAS_LGPIO
#include <lgpio.h>
#endif

namespace rc_vehicle_driver
{

namespace
{

constexpr char kSteeringPosition[] = "steering_joint/position";
constexpr char kTractionVelocity[] = "traction_joint/velocity";

#ifdef RC_VEHICLE_DRIVER_HAS_LGPIO
// Software-timed servo pulses via liblgpio (same backend gpiozero uses on Raspberry Pi).
class LgpioPwmOutput : public PwmOutput
{
public:
  LgpioPwmOutput(int chip, int gpio, int frequency_hz)
  : gpio_(gpio), frequency_hz_(frequency_hz)
  {
    handle_ = lgGpiochipOpen(chip);
    if (handle_ < 0) {
      throw std::runtime_error("lgGpiochipOpen failed for gpiochip" + std::to_string(chip));
    }
    if (lgGpioClaimOutput(handle_, 0, gpio_, 0) < 0) {
      lgGpiochipClose(handle_);
      handle_ = -1;
      throw std::runtime_error("lgGpioClaimOutput failed for GPIO" + std::to_string(gpio_));
    }
  }

  ~LgpioPwmOutput() override {release();}

  bool set_pulse_width_us(double pulse_us) override
  {
    return handle_ >= 0 &&
           lgTxServo(handle_, gpio_, static_cast<int>(std::lround(pulse_us)), frequency_hz_, 0, 0) >= 0;
  }

  void release() override
  {
    if (handle_ < 0) {
      return;
    }
    lgTxServo(handle_, gpio_, 0, frequency_hz_, 0, 0);  // 0 us stops the pulse train
    lgGpioFree(handle_, gpio_);
    lgGpiochipClose(handle_);
    handle_ = -1;
  }

private:
  int handle_ = -1;
  int gpio_;
  int frequency_hz_;
};
#endif

}  // namespace

// Maps steering_joint/position [rad] to servo PWM and traction_joint/velocity [rad/s]
// to ESC PWM. No steering/wheel-speed sensors are assumed: state interfaces echo the
// last applied (clamped) command, i.e. they are open-loop, not measurements.
class RcVehicleDriverHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    double max_angle = 0.0;
    double max_velocity = 0.0;
    if (!read_limit("steering_joint", "position", max_angle) ||
      !read_limit("traction_joint", "velocity", max_velocity))
    {
      return CallbackReturn::ERROR;
    }

    servo_ = {
      param("servo_center_pulse_us", 1450.0), param("servo_left_pulse_us", 1950.0),
      param("servo_right_pulse_us", 950.0), max_angle};
    esc_ = {
      param("esc_neutral_pulse_us", 1500.0), param("esc_forward_pulse_us", 2000.0),
      param("esc_reverse_pulse_us", 1000.0), max_velocity};

    const auto & hw_params = get_hardware_info().hardware_parameters;
    const auto mock = hw_params.find("use_mock_gpio");
    // xacro may render booleans as "True"
    use_mock_gpio_ = mock != hw_params.end() &&
      (mock->second == "true" || mock->second == "True");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    try {
      open_outputs();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to open PWM outputs: %s", e.what());
      return CallbackReturn::ERROR;
    }
    set_command(kSteeringPosition, 0.0);
    set_command(kTractionVelocity, 0.0);
    set_state(kSteeringPosition, 0.0);
    set_state(kTractionVelocity, 0.0);
    return apply_safe_output() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    close_outputs();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    close_outputs();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    close_outputs();
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    set_state(kSteeringPosition, applied_steering_);
    set_state(kTractionVelocity, applied_velocity_);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!servo_out_ || !esc_out_) {
      return hardware_interface::return_type::ERROR;
    }
    const double steering = get_command<double>(kSteeringPosition);
    const double velocity = get_command<double>(kTractionVelocity);
    if (!servo_out_->set_pulse_width_us(servo_.to_pulse_us(steering)) ||
      !esc_out_->set_pulse_width_us(esc_.to_pulse_us(velocity)))
    {
      RCLCPP_ERROR(get_logger(), "PWM output failed; switching to safe output");
      close_outputs();
      return hardware_interface::return_type::ERROR;
    }
    applied_steering_ = std::isfinite(steering) ?
      std::clamp(steering, -servo_.max_angle, servo_.max_angle) : 0.0;
    applied_velocity_ = std::isfinite(velocity) ?
      std::clamp(velocity, -esc_.max_velocity, esc_.max_velocity) : 0.0;
    return hardware_interface::return_type::OK;
  }

private:
  double param(const std::string & name, double default_value) const
  {
    const auto & params = get_hardware_info().hardware_parameters;
    const auto it = params.find(name);
    return it == params.end() ? default_value : std::stod(it->second);
  }

  // Symmetric limit taken from the joint's command interface max in the URDF.
  bool read_limit(const std::string & joint, const std::string & interface, double & limit) const
  {
    for (const auto & j : get_hardware_info().joints) {
      if (j.name != joint) {
        continue;
      }
      for (const auto & ci : j.command_interfaces) {
        if (ci.name == interface && !ci.max.empty()) {
          limit = std::stod(ci.max);
          if (limit > 0.0) {
            return true;
          }
        }
      }
    }
    RCLCPP_ERROR(
      get_logger(), "%s/%s needs a positive <param name=\"max\"> command interface limit",
      joint.c_str(), interface.c_str());
    return false;
  }

  void open_outputs()
  {
    if (use_mock_gpio_) {
      servo_out_ = std::make_unique<MockPwmOutput>();
      esc_out_ = std::make_unique<MockPwmOutput>();
      return;
    }
#ifdef RC_VEHICLE_DRIVER_HAS_LGPIO
    const int chip = static_cast<int>(param("gpiochip", 0));
    const int frequency = static_cast<int>(param("pwm_frequency_hz", 50));
    esc_out_ = std::make_unique<LgpioPwmOutput>(
      chip, static_cast<int>(param("esc_gpio_pin", 18)), frequency);
    servo_out_ = std::make_unique<LgpioPwmOutput>(
      chip, static_cast<int>(param("servo_gpio_pin", 17)), frequency);
#else
    throw std::runtime_error("built without lgpio; set hardware param use_mock_gpio=true");
#endif
  }

  bool apply_safe_output()
  {
    applied_steering_ = 0.0;
    applied_velocity_ = 0.0;
    return servo_out_ && esc_out_ &&
           servo_out_->set_pulse_width_us(servo_.center_us) &&
           esc_out_->set_pulse_width_us(esc_.neutral_us);
  }

  // Servo to center and ESC to neutral, then release the GPIOs.
  void close_outputs()
  {
    if (servo_out_ && esc_out_) {
      apply_safe_output();
    }
    if (servo_out_) {servo_out_->release();}
    if (esc_out_) {esc_out_->release();}
    servo_out_.reset();
    esc_out_.reset();
  }

  ServoCalibration servo_{};
  EscCalibration esc_{};
  bool use_mock_gpio_ = false;
  std::unique_ptr<PwmOutput> servo_out_;
  std::unique_ptr<PwmOutput> esc_out_;
  double applied_steering_ = 0.0;
  double applied_velocity_ = 0.0;
};

}  // namespace rc_vehicle_driver

PLUGINLIB_EXPORT_CLASS(
  rc_vehicle_driver::RcVehicleDriverHardware, hardware_interface::SystemInterface)
