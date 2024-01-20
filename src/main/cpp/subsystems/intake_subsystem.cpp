/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"

#include "argos_lib/config/talonsrx_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::intake::primaryIntake, address::practice_bot::intake::primaryIntake, robotInstance))
    , m_secondaryMotor(GetCANAddr(
          address::comp_bot::intake::secondaryIntake, address::practice_bot::intake::secondaryIntake, robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::primaryIntake,
                                             motorConfig::practice_bot::intake::primaryIntake>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::secondaryIntake,
                                             motorConfig::practice_bot::intake::secondaryIntake>(
      m_secondaryMotor, 100_ms, robotInstance);
  m_secondaryMotor.Follow(m_primaryMotor);
}
// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::Intake(double speed) {
  m_primaryMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::Disable() {
  m_primaryMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}
