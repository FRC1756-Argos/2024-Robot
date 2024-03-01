/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/ready_for_climb_command.h"

ReadyForClimbCommand::ReadyForClimbCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}, m_pElevator{elevator} {
  AddRequirements({m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void ReadyForClimbCommand::Initialize() {
  is_ready_climb_finished = false;
  m_pShooter->Disable();
  m_pElevator->SetCarriageAngle(90_deg);
}

// Called repeatedly when this Command is scheduled to run
void ReadyForClimbCommand::Execute() {}

// Called once the command ends or is interrupted.
void ReadyForClimbCommand::End(bool interrupted) {
  is_ready_climb_finished = true;
}

// Returns true when the command should end.
bool ReadyForClimbCommand::IsFinished() {
  return m_pElevator->IsCarriageAtSetPoint();
}

bool ReadyForClimbCommand::GetIsReadyCLimbFinished() {
  return is_ready_climb_finished;
}
