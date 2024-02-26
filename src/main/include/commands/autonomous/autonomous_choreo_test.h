/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"

class AutonomousChoreoTest
    : public frc2::CommandHelper<frc2::Command, AutonomousChoreoTest>
    , public AutonomousCommand {
 public:
  AutonomousChoreoTest(ElevatorSubsystem* elevator,
                       IntakeSubsystem* intake,
                       ShooterSubsystem* shooter,
                       SwerveDriveSubsystem* swerve,
                       VisionSubsystem* vision);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  ElevatorSubsystem* m_pElevator;
  IntakeSubsystem* m_pIntake;
  ShooterSubsystem* m_pShooter;
  SwerveDriveSubsystem* m_pSwerve;
  VisionSubsystem* m_pVision;

  frc2::SequentialCommandGroup m_allCommands;
};
