// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/climber_subsystem.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ReverseClimbCommand
    : public frc2::CommandHelper<frc2::Command, ReverseClimbCommand> {
 public:
  ReverseClimbCommand(ClimberSubsystem* climber,
                 ShooterSubsystem* shooter,
                 ElevatorSubsystem* elevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  ClimberSubsystem* m_pClimber;
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;

  frc2::SequentialCommandGroup m_SeqCommand;

  bool is_0_deg;
};
