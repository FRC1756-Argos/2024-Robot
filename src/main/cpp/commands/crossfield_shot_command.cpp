/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/crossfield_shot_command.h"

#include <frc2/command/WaitCommand.h>

#include "constants/measure_up.h"

CrossfieldShotCommand::CrossfieldShotCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}, m_pElevator{elevator} {}

// Called when the command is initially scheduled.
void CrossfieldShotCommand::Initialize() {
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::crossFieldAngle);
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
  m_pShooter->ShooterGoToSpeed(measure_up::shooter::crossFieldSpeed);
  m_pShooter->SetFeedingShotActive(true);
}

// Called repeatedly when this Command is scheduled to run
void CrossfieldShotCommand::Execute() {}

// Called once the command ends or is interrupted.
void CrossfieldShotCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool CrossfieldShotCommand::IsFinished() {
  return true;
}
