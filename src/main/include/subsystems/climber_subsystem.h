// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <argos_lib/config/config_types.h>
#include <ctre/phoenix/motorcontrol/can/TalonFx.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  explicit ClimberSubsystem(argos_lib::RobotInstance robotInstance);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 ctre::phoenix::motorcontrol::can::TalonFX m_primaryMotor;
 ctre::phoenix::motorcontrol::can::TalonFX m_secondaryMotor;
};
