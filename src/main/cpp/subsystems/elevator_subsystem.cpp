/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/elevator_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

ElevatorSubsystem::ElevatorSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(address::comp_bot::elevator::primaryElevator,
                                address::practice_bot::elevator::primaryElevator,
                                robotInstance))
    , m_carriageMotor(GetCANAddr(address::comp_bot::elevator::carriageRotation,
                                 address::practice_bot::elevator::carriageRotation,
                                 robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::primaryElevator,
                                         motorConfig::practice_bot::elevator::primaryElevator>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::practice_bot::elevator::carriageRotation,
                                         motorConfig::comp_bot::elevator::carriageRotation>(
      m_carriageMotor, 100_ms, robotInstance);

  ctre::phoenix6::configs::SoftwareLimitSwitchConfigs carriageRotationSoftLimits;
  carriageRotationSoftLimits.ForwardSoftLimitEnable = true;
  carriageRotationSoftLimits.ReverseSoftLimitEnable = true;
  carriageRotationSoftLimits.ForwardSoftLimitThreshold = units::turn_t{measure_up::elevator::maxAngle}.to<double>();
  carriageRotationSoftLimits.ReverseSoftLimitThreshold = units::turn_t{measure_up::elevator::minAngle}.to<double>();
  //   m_carriageMotor.GetConfigurator().Apply(carriageRotationSoftLimits);
}
// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}

void ElevatorSubsystem::ElevatorMove(double speed) {
  m_primaryMotor.Set(speed);
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

void ElevatorSubsystem::SetCarriageAngle(units::degree_t carriageAngle) {
  SetCarriageMotorManualOverride(false);
  carriageAngle =
      std::clamp<units::degree_t>(carriageAngle, measure_up::elevator::minAngle, measure_up::elevator::maxAngle);
  m_carriageMotor.SetControl(ctre::phoenix6::controls::PositionVoltage(carriageAngle));
}

void ElevatorSubsystem::SetCarriageMotorManualOverride(bool overrideState) {
  m_carriageMotorManualOverride = overrideState;
}

bool ElevatorSubsystem::IsCarriageMotorManualOverride() {
  return m_carriageMotorManualOverride;
}
