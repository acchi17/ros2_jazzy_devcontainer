#include <gtest/gtest.h>

#include <limits>

#include "rc_vehicle_driver/pwm_calibration.hpp"

using rc_vehicle_driver::EscCalibration;
using rc_vehicle_driver::ServoCalibration;

TEST(ServoCalibration, MapsCenterLeftRight)
{
  const ServoCalibration cal{1500.0, 2000.0, 1100.0, 0.5};
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(0.0), 1500.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(0.5), 2000.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(-0.5), 1100.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(0.25), 1750.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(-0.25), 1300.0);
}

TEST(ServoCalibration, ClampsAndHandlesNonFinite)
{
  const ServoCalibration cal{1500.0, 2000.0, 1100.0, 0.5};
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(2.0), 2000.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(-2.0), 1100.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(std::numeric_limits<double>::quiet_NaN()), 1500.0);
}

TEST(EscCalibration, MapsNeutralForwardReverse)
{
  const EscCalibration cal{1500.0, 2000.0, 1000.0, 60.0};
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(0.0), 1500.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(60.0), 2000.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(-60.0), 1000.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(30.0), 1750.0);
}

TEST(EscCalibration, ClampsAndHandlesNonFinite)
{
  const EscCalibration cal{1500.0, 2000.0, 1000.0, 60.0};
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(1e3), 2000.0);
  EXPECT_DOUBLE_EQ(cal.to_pulse_us(std::numeric_limits<double>::infinity()), 1500.0);
}
