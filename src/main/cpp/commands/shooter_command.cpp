/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/shooter_command.h"

using namespace std::chrono_literals;

ShooterCommand::ShooterCommand(ShooterSubsystem* shooter)
    : m_pShooter{shooter} {
  AddRequirements({m_pShooter});
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ShooterCommand::Initialize() {
  m_pShooter->NoteDetectionOverride(false);
  m_pShooter->Feed(0.0);
}

// Called repeatedly when this Command is scheduled to run
void ShooterCommand::Execute() {
  if (m_pShooter->ReadyToShoot()) {
    m_pShooter->Feed(1);
  }
}

// Called once the command ends or is interrupted.
void ShooterCommand::End(bool interrupted) {
  m_pShooter->Feed(0);
  //m_pShooter->Disable();
}

// Returns true when the command should end.
bool ShooterCommand::IsFinished() {
  return false;
}
