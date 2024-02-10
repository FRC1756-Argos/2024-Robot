/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/shooter_command.h"

using namespace std::chrono_literals;

ShooterCommand::ShooterCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}, m_pElevator{elevator} {
  AddRequirements({m_pShooter, m_pElevator});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {
  if (!(m_pShooter->ReadyToShoot())) {
    Cancel();
    return;
  }
  m_pShooter->NoteDetectionOverride(false);
  m_pShooter->ShooterGoToSpeed(5000_rpm);
  // put code for angle here
  m_startTime = std::chrono::steady_clock::now();
}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {
  if ((std::chrono::steady_clock::now() - m_startTime) > 3.0s) {
    m_pShooter->Feed(0.3);
    if (!(m_pShooter->IsNotePresent())) {
      Cancel();
    }
  }
}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {
  m_pShooter->Feed(0);
  m_pShooter->Disable();
}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  return false;
}
