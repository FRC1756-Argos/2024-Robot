/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/intake_command.h"

#include "constants/measure_up.h"

IntakeCommand::IntakeCommand(IntakeSubsystem* intake, ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pIntake{intake}, m_pShooter{shooter}, m_pElevator{elevator} {
  AddRequirements({m_pIntake, m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {
<<<<<<< HEAD
  if (m_pShooter->IsNotePresent()) {
    Cancel();
    return;
  }
=======
>>>>>>> origin/main
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::intakeAngle);
  m_pIntake->NoteDetectionOverride(false);
  m_pShooter->NoteDetectionOverride(false);
  m_pIntake->Intake(1);
  m_pShooter->Feed(0.3);
}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {
  if (m_pShooter->IsNotePresent()) {
<<<<<<< HEAD
    Cancel();
  } else {
    m_pIntake->Intake(1);
  }
=======
    m_pIntake->Intake(0);
  } else {
    m_pIntake->Intake(1);
  }
  if (m_pElevator->IsElevatorAtSetPoint()) {
    m_pIntake->NoteDetectionOverride(true);
  } else {
    m_pIntake->NoteDetectionOverride(false);
  }
>>>>>>> origin/main
}

// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {
  m_pIntake->Intake(0);
  m_pShooter->Feed(0);
<<<<<<< HEAD
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::intakeAngle);
=======
>>>>>>> origin/main
}

// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  return false;
}
