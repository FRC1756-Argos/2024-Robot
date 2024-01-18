/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "argos_lib/config/talonsrx_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

ShooterSubsystem::ShooterSubsystem(const argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(
          GetCANAddr(address::comp_bot::shooter::primaryMotor, address::practice_bot::primaryMotor, robotInstance))
    , m_secondaryMotor(
          GetCANAddr(address::comp_bot::shooter::primaryMotor, address::practice_bot::primaryMotor, robotInstance))
    , m_feedMotor(GetCANAddr(address::comp_bot::shooter::feedMotor, address::practice_bot::feedMotor, robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::falcon_config<motorConfig::comp_bot::shooter::primaryMotor,
                                          motorConfig::practice_bot::shooter::primaryMotor>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::falcon_config<motorConfig::comp_bot::shooter::secondaryMotor,
                                          motorConfig::practice_bot::shooter::secondaryMotor>(
      m_secondaryMotor, 100_ms, robotInstance);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::shooter::feedMotor,
                                             motorConfig::practice_bot::shooter::feedMotor>(
      m_feedMotor, 100_ms, robotInstance);
  m_secondaryMotor.Follow(m_primaryMotor);
}
// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}
