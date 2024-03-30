// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/crossfield_shot_command.h"
#include "constants/measure_up.h"
#include <frc2/command/WaitCommand.h>

CrossfieldShotCommand::CrossfieldShotCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator)
    : m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_ShootCommand{ShooterCommand{shooter}}
 {}

// Called when the command is initially scheduled.
void CrossfieldShotCommand::Initialize() {
  m_pElevator->SetCarriageAngle(measure_up::elevator::carriage::skipShotAngle);
  m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
  m_pShooter->ShooterGoToSpeed(2000_rpm);
}

// Called repeatedly when this Command is scheduled to run
void CrossfieldShotCommand::Execute() {
  if(m_pShooter->ShooterAtSpeed()){
    m_ShootCommand.Schedule();
    frc2::WaitCommand(50_ms);
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void CrossfieldShotCommand::End(bool interrupted) {
  m_pShooter->ShooterGoToSpeed(5000_rpm);
}

// Returns true when the command should end.
bool CrossfieldShotCommand::IsFinished() {
  return false;
}
