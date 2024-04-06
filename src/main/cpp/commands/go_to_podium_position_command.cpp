/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_podium_position_command.h"

#include "constants/measure_up.h"

GoToPodiumPositionCommand::GoToPodiumPositionCommand(ShooterSubsystem* shooter,
                                                     ElevatorSubsystem* elevator,
                                                     bool highPodiumShot)
    : m_pShooter{shooter}, m_pElevator{elevator}, m_highPodiumShot{highPodiumShot} {
  AddRequirements({m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void GoToPodiumPositionCommand::Initialize() {
  m_pShooter->SetTrapMode(false);
  m_pShooter->SetAmpMode(false);
  m_pShooter->ShooterGoToSpeed(5300_rpm);
  if (m_highPodiumShot) {
    m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::podiumHighHeight);
    m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::podiumHighAngle);
  } else {
    m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::podiumLowHeight);
    m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::podiumLowAngle);
  }
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
