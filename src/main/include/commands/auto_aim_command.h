/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/debouncer.h>
#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"

class AutoAimCommand : public frc2::CommandHelper<frc2::Command, AutoAimCommand> {
 public:
  AutoAimCommand(SwerveDriveSubsystem* swerveDrive,
                 ShooterSubsystem* shooter,
                 ElevatorSubsystem* elevator,
                 VisionSubsystem* vision,
                 argos_lib::SwappableControllersSubsystem* controllers,
                 SimpleLedSubsystem* leds,
                 bool endWhenAimed = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pSwerveDrive;
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  VisionSubsystem* m_pVision;
  argos_lib::SwappableControllersSubsystem* m_pControllers;
  SimpleLedSubsystem* m_pLeds;
  bool m_endWhenAimed;
  argos_lib::Debouncer m_aimedDebouncer;
  bool m_aimed;

  struct AimParams {
    units::degree_t carriageAngle;
    units::angular_velocity::revolutions_per_minute_t shooterSpeed;
    units::degree_t targetAngleOffset;
    units::degree_t cameraAngleOffset;
  };

  /// @brief Read latest vision targets from vision subsystem and generate parameters useful for aiming
  /// @return std::nullopt if no target found, otherwise parameters for aiming to vision target
  [[nodiscard]] std::optional<AimParams> GetAimParams();

  /// @brief Calculate angle offset to aim toward vision target
  /// @param params Aiming parameters generated from vision subsystem
  /// @return std::nullopt if no target found, otherwise offset angle
  [[nodiscard]] std::optional<units::degree_t> GetAdjustmentOffset(const std::optional<AimParams>& params);

  /// @brief Check if robot is currently aimed at vision target
  /// @param params Aiming parameters generated from vision subsystem
  /// @return true indicates robot is aimed, false if not aimed or no target found
  [[nodiscard]] bool Aimed(const std::optional<AimParams>& params);

  /// @brief Aim at vision target
  /// @param params Aiming parameters generated from vision subsystem
  /// @return true indicates robot is aimed, false if not aimed or no target found
  [[nodiscard]] bool Aim(const std::optional<AimParams>& params);

  /// @brief Set LEDs and vibration based on aiming status
  /// @param params Aiming parameters generated from vision subsystem
  void SetOperatorFeedback(const std::optional<AimParams>& params);
};
