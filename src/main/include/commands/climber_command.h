/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/go_to_trap_position_command.h"
#include "commands/lower_climber_command.h"
#include "commands/raise_climber_command.h"
#include "commands/ready_for_climb_command.h"
#include "commands/reverse_climb_command.h"
#include "commands/shooter_command.h"
#include "subsystems/climber_subsystem.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

class ClimberCommand : public frc2::CommandHelper<frc2::Command, ClimberCommand> {
 public:
  ClimberCommand(ClimberSubsystem* climber,
                 ShooterSubsystem* shooter,
                 ElevatorSubsystem* elevator,
                 argos_lib::SwappableControllersSubsystem* controllers);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSubsystem* m_pClimber;
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  GoToTrapPositionCommand m_TrapCommand;
  ReadyForClimbCommand m_ReadyForClimbCommand;
  RaiseClimberCommand m_RaiseClimberCommand;
  LowerClimberCommand m_LowerClimberCommand;
  frc2::SequentialCommandGroup m_SeqCommand;
  ShooterCommand m_ShootCommand;
  ReverseClimbCommand m_ReverseClimbCommand;

  argos_lib::SwappableControllersSubsystem* m_pControllers;
  int button_count;
  bool end_command;
};
