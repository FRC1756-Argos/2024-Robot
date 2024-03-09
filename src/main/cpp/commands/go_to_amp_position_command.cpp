/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_amp_position_command.h"

#include "constants/measure_up.h"

GoToAmpPositionCommand::GoToAmpPositionCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}, m_pElevator{elevator} {
  AddRequirements({m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void GoToAmpPositionCommand::Initialize() {
  m_pShooter->SetAmpMode(true);
  m_pShooter->SetTrapMode(false);
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::ampHeight);
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::ampAngle);
}

// Called repeatedly when this Command is scheduled to run
void GoToAmpPositionCommand::Execute() {
  if (m_pElevator->GetElevatorLiftManualOverride() || m_pElevator->IsCarriageMotorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void GoToAmpPositionCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToAmpPositionCommand::IsFinished() {
  return m_pElevator->IsElevatorAtSetPoint();
}
