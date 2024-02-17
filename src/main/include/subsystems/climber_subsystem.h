/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <argos_lib/config/config_types.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  explicit ClimberSubsystem(argos_lib::RobotInstance robotInstance);
  void SetExtensionSpeed(double speed);

  void SetExtensionLength(units::inch_t length);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void UpdateClimberHome();

  [[nodiscard]] bool IsClimberHomed() const;

  [[nodiscard]] bool IsClimberManualOverride() const;

  units::inch_t GetClimberExtension();

  [[nodiscard]] bool IsClimberMoving();

  void SetHomeFailed(bool failed);

  [[nodiscard]] bool GetHomeFailed() const;

  void ClimberMove(double speed, bool force = false);

  void SetHeight(units::inch_t height);

  void SetClimberManualOverride(bool state);

  void Disable();

  void Stop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_primaryMotor;
  ctre::phoenix6::hardware::TalonFX m_secondaryMotor;
  argos_lib::RobotInstance m_robotInstance;
  bool m_climberHomed;
  bool m_climberHomeFailed;
  bool m_climberManualOverride;

  void EnableClimberSoftLimits();
  void DisableClimberSoftLimits();
};
