/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/elevator_subsystem.h"

#include <ctre/phoenix6/controls/PositionVoltage.hpp>

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"

ElevatorSubsystem::ElevatorSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(address::comp_bot::elevator::primaryElevator,
                                address::practice_bot::elevator::primaryElevator,
                                robotInstance))
    , m_carriageMotor(GetCANAddr(address::comp_bot::elevator::carriageRotation,
                                 address::practice_bot::elevator::carriageRotation,
                                 robotInstance))
    , m_robotInstance(robotInstance)
    , m_elevatorManualOverride{false}
    , m_carriageMotorManualOverride{false} {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::primaryElevator,
                                         motorConfig::practice_bot::elevator::primaryElevator>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::practice_bot::elevator::carriageRotation,
                                         motorConfig::comp_bot::elevator::carriageRotation>(
      m_carriageMotor, 100_ms, robotInstance);

  /// @todo Actually home elevator height instead of assuming elevator starts at bottom
  m_primaryMotor.SetPosition(sensor_conversions::elevator::lift::ToSensorUnit(measure_up::elevator::lift::minHeight));
  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs elevatorLiftSoftLimits;
  elevatorLiftSoftLimits.ForwardSoftLimitThreshold =
      sensor_conversions::elevator::lift::ToSensorUnit(measure_up::elevator::lift::maxHeight).to<double>();
  elevatorLiftSoftLimits.ReverseSoftLimitThreshold =
      sensor_conversions::elevator::lift::ToSensorUnit(measure_up::elevator::lift::minHeight).to<double>();
  elevatorLiftSoftLimits.ForwardSoftLimitEnable = true;
  elevatorLiftSoftLimits.ReverseSoftLimitEnable = true;
  // m_primaryMotor.GetConfigurator().Apply(elevatorLiftSoftLimits);

  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs carriageRotationSoftLimits;
  carriageRotationSoftLimits.ForwardSoftLimitEnable = true;
  carriageRotationSoftLimits.ReverseSoftLimitEnable = true;
  carriageRotationSoftLimits.ForwardSoftLimitThreshold =
      units::turn_t{measure_up::elevator::carriage::maxAngle}.to<double>();
  carriageRotationSoftLimits.ReverseSoftLimitThreshold =
      units::turn_t{measure_up::elevator::carriage::minAngle}.to<double>();
  //   m_carriageMotor.GetConfigurator().Apply(carriageRotationSoftLimits);
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}

void ElevatorSubsystem::ElevatorMove(double speed) {
  if (GetElevatorLiftManualOverride()) {
    m_primaryMotor.Set(speed);
  }
}

void ElevatorSubsystem::Pivot(double speed) {
  if (m_carriageMotorManualOverride) {
    m_carriageMotor.Set(speed);
  }
}

void ElevatorSubsystem::Disable() {
  m_primaryMotor.Set(0.0);
  m_carriageMotor.Set(0.0);
}

void ElevatorSubsystem::ElevatorMoveToHeight(units::inch_t height) {
  height =
      std::clamp<units::inch_t>(height, measure_up::elevator::lift::minHeight, measure_up::elevator::lift::maxHeight);
  SetElevatorLiftManualOverride(false);
  m_primaryMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage(sensor_conversions::elevator::lift::ToSensorUnit(height)));
}

void ElevatorSubsystem::SetElevatorLiftManualOverride(bool desiredOverrideState) {
  m_elevatorManualOverride = desiredOverrideState;
}

bool ElevatorSubsystem::GetElevatorLiftManualOverride() const {
  return m_elevatorManualOverride;
}

void ElevatorSubsystem::SetCarriageAngle(units::degree_t carriageAngle) {
  SetCarriageMotorManualOverride(false);
  carriageAngle = std::clamp<units::degree_t>(
      carriageAngle, measure_up::elevator::carriage::minAngle, measure_up::elevator::carriage::maxAngle);
  m_carriageMotor.SetControl(ctre::phoenix6::controls::PositionVoltage(carriageAngle));
}

void ElevatorSubsystem::SetCarriageMotorManualOverride(bool overrideState) {
  m_carriageMotorManualOverride = overrideState;
}

bool ElevatorSubsystem::IsCarriageMotorManualOverride() const {
  return m_carriageMotorManualOverride;
}

units::inch_t ElevatorSubsystem::GetElevatorHeight() {
  return sensor_conversions::elevator::lift::ToHeight(m_primaryMotor.GetPosition().GetValue());
}

bool ElevatorSubsystem::IsLiftAtSetPoint() {
  if (m_primaryMotor.GetControlMode().GetValue() != ctre::phoenix6::signals::ControlModeValue::PositionVoltage) {
    return false;
  }
  return sensor_conversions::elevator::lift::ToHeight(units::turn_t{m_primaryMotor.GetClosedLoopError().GetValue()}) <
         0.25_in;
}

bool ElevatorSubsystem::IsCarriageAtSetPoint() {
  if (m_carriageMotor.GetControlMode().GetValue() != ctre::phoenix6::signals::ControlModeValue::PositionVoltage) {
    return false;
  }
  return sensor_conversions::elevator::carriage::ToAngle(
             units::turn_t{m_carriageMotor.GetClosedLoopError().GetValue()}) < 0.5_deg;
}

bool ElevatorSubsystem::IsElevatorAtSetPoint() {
  return IsLiftAtSetPoint() && IsCarriageAtSetPoint();
}
