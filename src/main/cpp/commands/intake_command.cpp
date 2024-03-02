/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/intake_command.h"

#include "constants/measure_up.h"

IntakeCommand::IntakeCommand(IntakeSubsystem* intake,
                             ShooterSubsystem* shooter,
                             ElevatorSubsystem* elevator,
                             argos_lib::SwappableControllersSubsystem* controllers,
                             SimpleLedSubsystem* leds,
                             bool endOnNoteAcquisition)
    : m_pIntake{intake}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_pControllers{controllers}
    , m_pLeds{leds}
    , m_endOnNoteAcquisition{endOnNoteAcquisition} {
  AddRequirements({m_pIntake, m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::intakeAngle);
  m_pIntake->NoteDetectionOverride(false);
  m_pShooter->NoteDetectionOverride(false);
  m_pIntake->Intake(m_pIntake->IsNotePresent() ? 0.4 : 1.0);
  m_pShooter->Feed(m_pIntake->IsNotePresent() ? 0.5 : 0.3);
  m_pShooter->SetAmpAndTrapMode(false);
}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {
  m_pShooter->Feed(m_pIntake->IsNotePresent() ? 0.6 : 0.3);
  if (m_pShooter->IsNotePresent()) {
    m_pIntake->Intake(0);
    if (m_pLeds) {
      m_pLeds->TemporaryAnimate(
          [this]() { m_pLeds->SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kReallyGreen, false); }, 200_ms);
    }
    if (m_pControllers) {
      m_pControllers->DriverController().SetVibration(
          argos_lib::TemporaryVibrationPattern(argos_lib::VibrationConstant(1.0), 500_ms));
    }
  } else {
    m_pIntake->Intake(m_pIntake->IsNotePresent() ? 0.4 : 1.0);
  }
  if (m_pElevator->IsElevatorAtSetPoint()) {
    m_pIntake->NoteDetectionOverride(true);
  } else {
    m_pIntake->NoteDetectionOverride(false);
  }
}

// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {
  m_pIntake->Intake(0);
  m_pShooter->Feed(0);
  if (m_pLeds) {
    if (m_pIntake->IsNotePresent()) {
      m_pLeds->TemporaryAnimate(
          [this]() { m_pLeds->SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kCatYellow, false); }, 500_ms);
    } else if (m_pShooter->IsNotePresent()) {
      m_pLeds->TemporaryAnimate(
          [this]() { m_pLeds->SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kReallyGreen, false); }, 500_ms);
    } else {
      m_pLeds->TemporaryAnimate(
          [this]() { m_pLeds->SetAllGroupsFlash(argos_lib::gamma_corrected_colors::kWhite, false); }, 500_ms);
    }
  }
}

// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  return m_endOnNoteAcquisition && m_pShooter->IsNotePresent();
}
