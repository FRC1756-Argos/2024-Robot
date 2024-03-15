/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_nothing.h"

AutonomousNothing::AutonomousNothing(SwerveDriveSubsystem& swerve)
    : m_swerve{swerve}, m_initializeOdometry{&m_swerve, frc::Pose2d{{0_m, 0_m}, 0_deg}} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutonomousNothing::Initialize() {
  m_initializeOdometry.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousNothing::Execute() {
  m_initializeOdometry.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousNothing::End(bool interrupted) {
  m_initializeOdometry.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousNothing::IsFinished() {
  return m_initializeOdometry.IsFinished();
}

std::string AutonomousNothing::GetName() const {
  return "00. Do Nothing";
}

frc2::Command* AutonomousNothing::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
