// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/intake_subsystem.h"
#include "constants/motors.h"
#include "constants/addresses.h"
#include "argos_lib/config/talonsrx_config.h"
IntakeSubsystem::IntakeSubsystem(argos_lib::RobotInstance robotInstance):
m_primaryMotor(GetCANAddr(address::comp_bot::intake::primaryIntake, address::practice_bot::intake::primaryIntake, robotInstance)),
m_secondaryMotor(GetCANAddr(address::comp_bot::intake::secondaryIntake, address::practice_bot::intake::secondaryIntake, robotInstance)),
m_robotInstance(robotInstance){
    argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::primaryIntake, motorConfig::practice_bot::intake::primaryIntake>(m_primaryMotor,100_ms,robotInstance);
    argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::intake::secondaryIntake, motorConfig::practice_bot::intake::secondaryIntake>(m_secondaryMotor,100_ms,robotInstance);
    m_secondaryMotor.Follow(m_primaryMotor);
}
// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}
