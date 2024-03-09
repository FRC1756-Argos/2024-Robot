// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/reverse_climb_command.h"

#include "constants/measure_up.h"

#include <frc2/command/WaitCommand.h>

ReverseClimbCommand::ReverseClimbCommand(ClimberSubsystem* climber,
                 ShooterSubsystem* shooter,
                 ElevatorSubsystem* elevator)
    : m_pClimber{climber}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_SeqCommand{ frc2::WaitCommand(200_ms), frc2::InstantCommand([this](){m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight), m_pClimber->SetHeight(measure_up::climber::climbRaisedHeight);})}
     {
  AddRequirements({m_pShooter, m_pElevator, m_pClimber});
}

// Called when the command is initially scheduled.
void ReverseClimbCommand::Initialize() {

  m_pShooter->SetTrapMode(false);
  m_pElevator->SetCarriageAngle(90_deg);
  is_0_deg = false;
}

// Called repeatedly when this Command is scheduled to run
void ReverseClimbCommand::Execute() {
  if (!is_0_deg && m_pElevator->IsCarriageAtSetPoint()) {
    //frc2::WaitCommand(200_ms);
    // m_pElevator->ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
    // m_pClimber->SetHeight(measure_up::climber::climbRaisedHeight);
    m_SeqCommand.Schedule();
    is_0_deg = true;
  }
  if (m_pClimber->IsClimberManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void ReverseClimbCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ReverseClimbCommand::IsFinished() {
  return is_0_deg;
}
