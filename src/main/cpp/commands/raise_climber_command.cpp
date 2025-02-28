/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/raise_climber_command.h"

RaiseClimberCommand::RaiseClimberCommand(ClimberSubsystem* climber) : m_pClimber{climber} {
  AddRequirements({m_pClimber});
}

// Called when the command is initially scheduled.
void RaiseClimberCommand::Initialize() {
  is_raise_climb_finished = false;
  m_pClimber->SetHeight(measure_up::climber::climbRaisedHeight);
}

// Called repeatedly when this Command is scheduled to run
void RaiseClimberCommand::Execute() {}

// Called once the command ends or is interrupted.
void RaiseClimberCommand::End(bool interrupted) {
  is_raise_climb_finished = true;
}

// Returns true when the command should end.
bool RaiseClimberCommand::IsFinished() {
  return m_pClimber->IsClimberAtSetPoint();
}

bool RaiseClimberCommand::GetIsRaiseCLimbFinished() {
  return is_raise_climb_finished;
}
