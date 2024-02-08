/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  explicit ShooterSubsystem(const argos_lib::RobotInstance robotInstance);

  void Periodic() override;

  void Shoot(double speed);

  void Feed(double speed);

  void Disable();

  void ShooterGoToSpeed(units::turns_per_second_t speed);

  [[nodiscard]] bool IsNotePresent();

  void NoteDetectionOverride(bool override);

 private:
  ctre::phoenix6::hardware::TalonFX m_primaryMotor;
  ctre::phoenix6::hardware::TalonFX m_secondaryMotor;
  ctre::phoenix::motorcontrol::can::TalonSRX m_feedMotor;
  argos_lib::RobotInstance m_robotInstance;
  ctre::phoenix6::controls::VelocityVoltage m_velocityControl;
};
// Components (e.g. motor controllers and sensors) should generally be
// declared private and exposed only through public methods.
