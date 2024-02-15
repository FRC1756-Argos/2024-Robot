/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_aim_command.h"

#include "constants/measure_up.h"

AutoAimCommand::AutoAimCommand(SwerveDriveSubsystem* swerveDrive,
                               ShooterSubsystem* shooter,
                               ElevatorSubsystem* elevator,
                               VisionSubsystem* vision)
    : m_pSwerveDrive{swerveDrive}, m_pShooter{shooter}, m_pElevator{elevator}, m_pVision{vision} {
  AddRequirements({m_pSwerveDrive, m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void AutoAimCommand::Initialize() {
  m_pShooter->ShooterGoToSpeed(5000_rpm);
}

// Called repeatedly when this Command is scheduled to run
void AutoAimCommand::Execute() {
  auto angle = m_pVision->getShooterAngle();
  if (angle != std::nullopt) {
    m_pElevator->SetCarriageAngle(angle.value());
  }

  auto horzOffset = m_pVision->GetCalculatedDistanceToSpeaker();
  if (horzOffset != std::nullopt) {
    double offset = horzOffset.value().to<double>();
    m_pSwerveDrive->SwerveDrive(0.0, 0.0, offset * 0.008);
  }
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  return false;
}
