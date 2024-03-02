/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/shooter_command.h"

using namespace std::chrono_literals;

ShooterCommand::ShooterCommand(ShooterSubsystem* shooter, bool endAfterShot)
    : m_pShooter{shooter}, m_endAfterShot{endAfterShot}, m_notePresent{false}, m_noteShot{false} {
  AddRequirements({m_pShooter});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {
  m_pShooter->NoteDetectionOverride(true);
  m_notePresent = m_pShooter->IsNotePresent();
  m_noteShot = false;
}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {
  m_pShooter->Shoot();
  if (!m_notePresent) {
    m_notePresent = m_pShooter->IsNotePresent();
  }
  if (m_notePresent && !m_noteShot) {
    m_noteShot = !m_pShooter->IsNotePresent();
  }
}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {
  m_pShooter->NoteDetectionOverride(false);
  m_pShooter->StopShoot();
}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  return m_endAfterShot && m_noteShot;
}
