/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_podium_position_command.h"

#include "constants/measure_up.h"

GoToPodiumPositionCommand::GoToPodiumPositionCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}, m_pElevator{elevator} {
  AddRequirements({m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void GoToPodiumPositionCommand::Initialize() {
  m_pShooter->SetAmpAndTrapMode(false);
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::ampHeight);
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::ampAngle);
}

// Called repeatedly when this Command is scheduled to run
void GoToPodiumPositionCommand::Execute() {
  if (m_pElevator->GetElevatorLiftManualOverride() || m_pElevator->IsCarriageMotorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void GoToPodiumPositionCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool GoToPodiumPositionCommand::IsFinished() {
  return m_pElevator->IsElevatorAtSetPoint();
}
