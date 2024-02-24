/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>

#include "commands/go_to_trap_position_command.h"
#include "commands/lower_climber_command.h"
#include "commands/raise_climber_command.h"
#include "commands/ready_for_climb_command.h"
#include "subsystems/climber_subsystem.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

class ClimberCommand : public frc2::CommandHelper<frc2::Command, ClimberCommand> {
 public:
  ClimberCommand(ClimberSubsystem* climber, ShooterSubsystem* shooter, ElevatorSubsystem* elevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSubsystem* m_pClimber;
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  // GoToTrapPositionCommand* m_pTrapCommand;
  // ReadyForClimbCommand* m_pReadyForClimbCommand;
  // RaiseClimberCommand* m_pRaiseClimberCommand;
  // LowerClimberCommand* m_pLowerClimberCommand;
  frc2::CommandPtr m_allCommands;
};