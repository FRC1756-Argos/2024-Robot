/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc2/command/SubsystemBase.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  explicit IntakeSubsystem(argos_lib::RobotInstance robotInstance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Intake(double speed);

  void Disable();

  [[nodiscard]] bool IsNotePresent();

  void NoteDetectionOverride(bool override);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::motorcontrol::can::TalonSRX m_primaryMotor;
  ctre::phoenix::motorcontrol::can::TalonSRX m_secondaryMotor;
  argos_lib::RobotInstance m_robotInstance;
};
