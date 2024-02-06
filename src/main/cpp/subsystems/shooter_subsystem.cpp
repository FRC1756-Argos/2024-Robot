/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/shooter_subsystem.h"

#include <argos_lib/config/falcon_config.h>
#include <argos_lib/config/talonsrx_config.h>

#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"

ShooterSubsystem::ShooterSubsystem(const argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::shooter::primaryMotor, address::practice_bot::shooter::primaryMotor, robotInstance))
    , m_secondaryMotor(GetCANAddr(
          address::comp_bot::shooter::secondaryMotor, address::practice_bot::shooter::secondaryMotor, robotInstance))
    , m_feedMotor(
          GetCANAddr(address::comp_bot::shooter::feedMotor, address::practice_bot::shooter::feedMotor, robotInstance))
    , m_robotInstance(robotInstance)
    , m_velocityControl{0_tps} {
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::shooter::primaryMotor,
                                         motorConfig::practice_bot::shooter::primaryMotor>(
      m_primaryMotor, 100_ms, robotInstance);
  argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::shooter::secondaryMotor,
                                         motorConfig::practice_bot::shooter::secondaryMotor>(
      m_secondaryMotor, 100_ms, robotInstance);
  argos_lib::talonsrx_config::TalonSRXConfig<motorConfig::comp_bot::shooter::feedMotor,
                                             motorConfig::practice_bot::shooter::feedMotor>(
      m_feedMotor, 100_ms, robotInstance);
  m_secondaryMotor.SetControl(ctre::phoenix6::controls::Follower(m_primaryMotor.GetDeviceID(), true));
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}

void ShooterSubsystem::Shoot(double speed) {
  m_primaryMotor.Set(speed);
}

void ShooterSubsystem::ShooterGoToSpeed(units::turns_per_second_t speed) {
  speed = std::clamp<units::turns_per_second_t>(speed, measure_up::shooter::minSpeed, measure_up::shooter::maxSpeed);
  m_primaryMotor.SetControl(m_velocityControl.WithVelocity(speed));
}

void ShooterSubsystem::ShooterGoToSpeed(units::turns_per_second_t speed) {
  speed = std::clamp<units::turns_per_second_t>(speed, measure_up::shooter::minSpeed, measure_up::shooter::maxSpeed);
  m_primaryMotor.SetControl(m_velocityControl.WithVelocity(speed));
}

void ShooterSubsystem::Feed(double speed) {
  m_feedMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void ShooterSubsystem::Disable() {
  m_primaryMotor.Set(0.0);
  m_feedMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}
