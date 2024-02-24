/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climber_command.h"

#include <frc2/command/SequentialCommandGroup.h>

ClimberCommand::ClimberCommand(ClimberSubsystem* climber, ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pClimber{climber}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    // , m_pTrapCommand{trapCommand}  //, m_pReadyForClimbCommand{&ReadyForClimbCommand(shooter, elevator)}
    // , m_pRaiseClimberCommand{&RaiseClimberCommand(climber)}
    // , m_pLowerClimberCommand{&LowerClimberCommand(climber)}
    , m_allCommands{frc2::InstantCommand{[]() {}, {}}} {
  AddRequirements({m_pClimber, m_pShooter, m_pClimber});
}

// Called when the command is initially scheduled.
void ClimberCommand::Initialize() {
  // m_allCommands = ReadyForClimbCommand{m_pShooter, m_pElevator}.ToPtr().AndThen()

  // (frc2::SequentialCommandGroup(&m_pReadyForClimbCommand, &m_pRaiseClimberCommand, &m_pLowerClimberCommand));
}

// Called repeatedly when this Command is scheduled to run
void ClimberCommand::Execute() {
  if (!m_allCommands.IsScheduled()) {
    Cancel();
    return;
  }
}

// Called once the command ends or is interrupted.
void ClimberCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberCommand::IsFinished() {
  return false;
}
