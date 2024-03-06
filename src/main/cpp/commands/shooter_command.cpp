/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/shooter_command.h"

#include <units/time.h>

using namespace std::chrono_literals;

ShooterCommand::ShooterCommand(ShooterSubsystem* shooter, bool endAfterShot, units::millisecond_t timeout)
    : m_pShooter{shooter}, m_endAfterShot{endAfterShot}, m_notePresent{false}, m_noteShot{false}, m_timeout{timeout} {
  AddRequirements({m_pShooter});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {
  m_pShooter->NoteDetectionOverride(true);
  m_notePresent = m_pShooter->IsNotePresent();
  m_noteShot = false;
  m_startTime = std::chrono::steady_clock::now();
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
  return m_endAfterShot && (m_noteShot || units::second_t{std::chrono::steady_clock::now() - m_startTime} > m_timeout);
}
