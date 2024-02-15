/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

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
    , m_climberManualOverride(false) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::primaryClimbing,
                                         motorConfig::practice_bot::climber::primaryClimbing>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::secondaryClimbing,
                                         motorConfig::practice_bot::climber::secondaryClimbing>(
      m_secondaryMotor, 100_ms, robotInstance);
  m_secondaryMotor.SetControl(ctre::phoenix6::controls::Follower(m_primaryMotor.GetDeviceID(), true));
  m_primaryMotor.SetPosition(sensor_conversions::climber::ToSensorUnit(measure_up::climber::minExtension));
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ClimberMove(double speed) {
  if (m_climberManualOverride) {
    m_primaryMotor.Set(speed);
  }
}

void ClimberSubsystem::SetHeight(units::inch_t height) {
  SetClimberManualOverride(false);
  if (height > measure_up::climber::maxExtension) {
    height = measure_up::climber::maxExtension;
  } else if (height < measure_up::climber::minExtension) {
    height = measure_up::climber::minExtension;
  }
  m_primaryMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage(sensor_conversions::climber::ToSensorUnit(height)));
}

void ClimberSubsystem::SetClimberManualOverride(bool overrideState) {
  m_climberManualOverride = overrideState;
}

void ClimberSubsystem::Disable() {
  m_primaryMotor.Set(0.0);
}

bool ClimberSubsystem::IsClimberMoving() {
  return (m_primaryMotor.GetVelocity()).GetValue() > 10_tps;
}
void ClimberSubsystem::SetHomeFailed(bool failed) {
  m_climberHomeFailed = failed;
}
bool ClimberSubsystem::GetHomeFailed() {
  return m_climberHomeFailed;
}

bool ClimberSubsystem::IsClimberHomed() {
  return m_climberHomed || m_climberHomeFailed;
}

units::inch_t ClimberSubsystem::GetClimberExtension() {
  return sensor_conversions::climber::ToHeight(m_primaryMotor.GetRotorPosition())
}

void ClimberSubsystem::Stop() {
  m_primaryMotor.Set(0.0);
}

bool ClimberSubsystem::IsClimberManualOverride() {
  return m_climberManualOverride;
}

void ClimberSubsystem::UpdateClimberHome() {
  m_primaryMotor;
  m_climberHomed = true;
  m_climberHomeFailed = false;
  EnableClimberSoftLimits();
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::EnableClimberSoftLimits() {
  if (m_climberHomed) {
    m_primaryMotor.ForwardSoftLimitThreshold(
        sensor_conversions::climber::ToSensorUnit(measure_up::climber::maxExtension));
    m_primaryMotor.ReverseSoftLimitThreshold(
        sensor_conversions::climber::ToSensorUnit(measure_up::climber::minExtension));
    m_primaryMotor.ForwardSoftLimitEnable(true);
    m_primaryMotor.ReverseSoftLimitEnable(true);
  }
}
void m_primaryMotorSubsystem::DisableClimberSoftLimits() {
  m_primaryMotor.ForwardSoftLimitEnable(false);
  m_primaryMotor.ReverseSoftLimitEnable(false);
}
