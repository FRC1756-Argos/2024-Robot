/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include <units/math.h>

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
  m_primaryMotor.SetPosition(sensor_conversions::climber::ToSensorUnit(measure_up::climber::lowerLimit));
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ClimberMove(double speed, bool force) {
  if (m_climberManualOverride || force) {
    m_primaryMotor.Set(speed);
  }
}

void ClimberSubsystem::SetHeight(units::inch_t height) {
  SetClimberManualOverride(false);
  height = std::clamp<units::inch_t>(height, measure_up::climber::lowerLimit, measure_up::climber::upperLimit);
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
  return units::math::abs(m_primaryMotor.GetRotorVelocity().GetValue()) > 0.5_tps;
}

void ClimberSubsystem::SetHomeFailed(bool failed) {
  m_climberHomeFailed = failed;
  if (failed) {
    m_climberHomed = false;
  }
}

bool ClimberSubsystem::GetHomeFailed() const {
  return m_climberHomeFailed;
}

bool ClimberSubsystem::IsClimberHomed() const {
  return m_climberHomed;
}

units::inch_t ClimberSubsystem::GetClimberExtension() {
  return sensor_conversions::climber::ToHeight(m_primaryMotor.GetPosition().GetValue());
}

void ClimberSubsystem::Stop() {
  m_primaryMotor.Set(0.0);
}

bool ClimberSubsystem::IsClimberManualOverride() const {
  return m_climberManualOverride;
}

void ClimberSubsystem::UpdateClimberHome() {
  m_primaryMotor.SetPosition(sensor_conversions::climber::ToSensorUnit(measure_up::climber::lowerLimit));
  m_climberHomed = true;
  m_climberHomeFailed = false;
  EnableClimberSoftLimits();
}

void ClimberSubsystem::EnableClimberSoftLimits() {
  if (m_climberHomed) {
    ctre::phoenix6::configs::SoftwareLimitSwitchConfigs climberSoftLimits;
    climberSoftLimits.ForwardSoftLimitThreshold =
        sensor_conversions::climber::ToSensorUnit(measure_up::climber::upperLimit).to<double>();
    climberSoftLimits.ReverseSoftLimitThreshold =
        sensor_conversions::climber::ToSensorUnit(measure_up::climber::lowerLimit + 0.25_in).to<double>();
    climberSoftLimits.ForwardSoftLimitEnable = true;
    climberSoftLimits.ReverseSoftLimitEnable = true;
    m_primaryMotor.GetConfigurator().Apply(climberSoftLimits);
  }
}

void ClimberSubsystem::DisableClimberSoftLimits() {
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs climberSoftLimits;
  climberSoftLimits.ForwardSoftLimitEnable = false;
  climberSoftLimits.ReverseSoftLimitEnable = false;
  m_primaryMotor.GetConfigurator().Apply(climberSoftLimits);
}

bool ClimberSubsystem::IsClimberAtSetPoint() {
  // frc::SmartDashboard::PutString("ElevatorLiftMode", m_primaryMotor.GetControlMode().GetValue().ToString());
  // frc::SmartDashboard::PutNumber("ElevatorLiftError", m_primaryMotor.GetClosedLoopError().GetValue());
  // frc::SmartDashboard::PutNumber(
  //     "ElevatorHeightError",
  //    sensor_conversions::elevator::lift::ToHeight(units::turn_t{m_primaryMotor.GetClosedLoopError().GetValue()})
  //        .to<double>());
  if (m_primaryMotor.GetControlMode().GetValue() != ctre::phoenix6::signals::ControlModeValue::PositionVoltage &&
      m_primaryMotor.GetControlMode().GetValue() != ctre::phoenix6::signals::ControlModeValue::PositionVoltageFOC) {
    return false;
  }
  return units::math::abs(sensor_conversions::climber::ToHeight(
             units::turn_t{m_primaryMotor.GetClosedLoopError().GetValue()})) < 0.25_in;
}
