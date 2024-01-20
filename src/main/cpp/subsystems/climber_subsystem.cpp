// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

ClimbingSubsystem::ClimbingSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::Climbing::primaryClimbing, address::practice_bot::Climbing::primaryClimbing, robotInstance))
    , m_secondaryMotor(GetCANAddr(
          address::comp_bot::Climbing::secondaryClimbing, address::practice_bot::Climbing::secondaryClimbing, robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::talonfx_config::TalonFXConfig<motorConfig::comp_bot::Climbing::primaryClimbing,
                                             motorConfig::practice_bot::Climbing::primaryClimbing>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::talonsfx_config::TalonsxConfig<motorConfig::comp_bot::Climbing::secondaryClimbing,
                                             motorConfig::practice_bot::Climbing::secondaryClimbing>(
      m_secondaryMotor, 100_ms, robotInstance);
  m_secondaryMotor.Follow(m_primaryMotor);
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}
