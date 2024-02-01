/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/climber_subsystem.h"

#include <ctre/phoenix6/TalonFX.hpp>

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

ClimberSubsystem::ClimberSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::climber::primaryClimbing, address::practice_bot::climber::primaryClimbing, robotInstance))
    , m_secondaryMotor(GetCANAddr(address::comp_bot::climber::secondaryClimbing,
                                  address::practice_bot::climber::secondaryClimbing,
                                  robotInstance))
    , m_robotInstance(robotInstance) {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::primaryClimbing,
                                         motorConfig::practice_bot::climber::primaryClimbing>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::climber::secondaryClimbing,
                                         motorConfig::practice_bot::climber::secondaryClimbing>(
      m_secondaryMotor, 100_ms, robotInstance);
  m_secondaryMotor.SetControl(ctre::phoenix6::controls::Follower(m_primaryMotor.GetDeviceID(), true));
}
// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ClimberMove(double speed) {
  m_primaryMotor.Set(speed);
}

void ClimberSubsystem::Disable() {
  m_primaryMotor.Set(0.0);
}
