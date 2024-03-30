// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>


#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "commands/shooter_command.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class CrossfieldShotCommand
    : public frc2::CommandHelper<frc2::Command, CrossfieldShotCommand> {
 public:
  CrossfieldShotCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    ShooterSubsystem* m_pShooter;
    ElevatorSubsystem* m_pElevator;
    ShooterCommand m_ShootCommand;

};
