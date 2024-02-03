/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/elevator_subsystem.h"

#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

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
    , m_elevatorManualOverride{false}{
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::primaryElevator,
                                         motorConfig::practice_bot::elevator::primaryElevator>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::practice_bot::elevator::carriageRotation,
                                         motorConfig::comp_bot::elevator::carriageRotation>(
      m_carriageMotor, 100_ms, robotInstance);
      m_primaryMotor.SetPosition(sensor_conversions::elevator::raise::ToSensorUnit(measure_up::elevator::minHeight));
      ctre::phoenix6::configs::SoftwareLimitSwitchConfigs elevatorLiftSoftLimits;
      elevatorLiftSoftLimits.ForwardSoftLimitThreshold = sensor_conversions::elevator::raise::ToSensorUnit(measure_up::elevator::maxHeight).to<double>();
      elevatorLiftSoftLimits.ReverseSoftLimitThreshold = sensor_conversions::elevator::raise::ToSensorUnit(measure_up::elevator::minHeight).to<double>();
      elevatorLiftSoftLimits.ForwardSoftLimitEnable = true;
      elevatorLiftSoftLimits.ReverseSoftLimitEnable = true;
      m_primaryMotor.GetConfigurator().Apply(elevatorLiftSoftLimits);
    }

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}

void ElevatorSubsystem::ElevatorMove(double speed) {
  if (GetElevatorLiftManualOverride()) {
    m_primaryMotor.Set(speed);
  }
}

void ElevatorSubsystem::Pivot(double speed) {
  m_carriageMotor.Set(speed);
}

void ElevatorSubsystem::Disable() {
  m_primaryMotor.Set(0.0);
  m_carriageMotor.Set(0.0);
}

void ElevatorSubsystem::ElevatorMoveToHeight(units::inch_t height) {
  height = std::clamp<units::inch_t>(height, measure_up::elevator::minHeight, measure_up::elevator::maxHeight);
  SetElevatorLiftManualOverride(false);
  m_primaryMotor.SetControl(
      ctre::phoenix6::controls::PositionVoltage(sensor_conversions::elevator::raise::ToSensorUnit(height)));
}

void ElevatorSubsystem::SetElevatorLiftManualOverride(bool desiredOverrideState) {
  m_elevatorManualOverride = desiredOverrideState;
}

bool ElevatorSubsystem::GetElevatorLiftManualOverride() const {
  return m_elevatorManualOverride;
}
