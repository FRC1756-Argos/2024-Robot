/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include <ctre/phoenix6/TalonFX.hpp>

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"

ClimberSubsystem::ClimberSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::climber::primaryClimbing, address::practice_bot::climber::primaryClimbing, robotInstance))
    , m_secondaryMotor(GetCANAddr(address::comp_bot::climber::secondaryClimbing,
                                  address::practice_bot::climber::secondaryClimbing,
                                  robotInstance))
    , m_robotInstance(robotInstance)
    , m_manualOverride(false) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::primaryClimbing,
                                         motorConfig::practice_bot::climber::primaryClimbing>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::secondaryClimbing,
                                         motorConfig::practice_bot::climber::secondaryClimbing>(
      m_secondaryMotor, 100_ms, robotInstance);
  m_secondaryMotor.SetControl(ctre::phoenix6::controls::Follower(m_primaryMotor.GetDeviceID(), true));
  m_primaryMotor.SetPosition(sensor_conversions::climber::ToSensorUnit(measure_up::climber::lowerLimit));
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ClimberMove(double speed) {
  if (m_manualOverride) {
    m_primaryMotor.Set(speed);
  }
}

void ClimberSubsystem::SetHeight(units::inch_t height) {
  SetManualOverride(false);
  if (height > measure_up::climber::upperLimit) {
    height = measure_up::climber::upperLimit;
  } else if (height < measure_up::climber::lowerLimit) {
    height = measure_up::climber::lowerLimit;
  }
  m_primaryMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage(sensor_conversions::climber::ToSensorUnit(height)));
}

void ClimberSubsystem::SetManualOverride(bool overrideState) {
  m_manualOverride = overrideState;
}

void ClimberSubsystem::Disable() {
  m_primaryMotor.Set(0.0);
}
