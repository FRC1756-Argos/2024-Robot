/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/go_to_trap_position_command.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "constants/measure_up.h"

GoToTrapPositionCommand::GoToTrapPositionCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}, m_pElevator{elevator} {
  AddRequirements({m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void GoToTrapPositionCommand::Initialize() {
  is_trap_done = false;
  m_pShooter->SetAmpAndTrapMode(true);
  m_pShooter->Disable();
  m_pElevator->SetCarriageAngle(90_deg);
  is_0_deg = false;
}

// Called repeatedly when this Command is scheduled to run
void GoToTrapPositionCommand::Execute() {
  if (!is_0_deg && m_pElevator->IsCarriageAtSetPoint()) {
    m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::trapHeight);
    is_0_deg = true;
  }
  if (is_0_deg && m_pElevator->IsLiftAtSetPoint()) {
    m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::trapAngle);
  }

  if (m_pElevator->GetElevatorLiftManualOverride() || m_pElevator->IsCarriageMotorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void GoToTrapPositionCommand::End(bool interrupted) {
  is_trap_done = true;
}

// Returns true when the command should end.
bool GoToTrapPositionCommand::IsFinished() {
  return m_pElevator->IsElevatorAtSetPoint();
}

bool GoToTrapPositionCommand::GetIsTrapDone() {
  return is_trap_done;
}

void GoToTrapPositionCommand::ResetIsTrapDone() {
  is_trap_done = false;
}
