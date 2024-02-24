/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

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
                 SimpleLedSubsystem* leds);

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
};
