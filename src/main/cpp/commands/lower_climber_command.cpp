/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/lower_climber_command.h"

LowerClimberCommand::LowerClimberCommand(ClimberSubsystem* climber) : m_pClimber{climber} {
  AddRequirements({m_pClimber});
}

// Called when the command is initially scheduled.
void LowerClimberCommand::Initialize() {
  is_lower_climb_finished = false;
  m_pClimber->SetHeight(measure_up::climber::climbLoweredHeight);
}

// Called repeatedly when this Command is scheduled to run
void LowerClimberCommand::Execute() {}

// Called once the command ends or is interrupted.
void LowerClimberCommand::End(bool interrupted) {
  is_lower_climb_finished = true;
}

// Returns true when the command should end.
bool LowerClimberCommand::IsFinished() {
  return m_pClimber->IsClimberMoving();
}

bool LowerClimberCommand::GetIsLowerCLimbFinished(){
  return is_lower_climb_finished;
}
