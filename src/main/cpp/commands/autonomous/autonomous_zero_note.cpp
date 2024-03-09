/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_zero_note.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "commands/drive_choreo.h"

AutonomousZeroNote::AutonomousZeroNote(SwerveDriveSubsystem& swerve)
    : m_Swerve{swerve}, m_commands{frc2::SequentialCommandGroup{DriveChoreo{m_Swerve, "Any_0Note", true}}} {}

// Called when the command is initially scheduled.
void AutonomousZeroNote::Initialize() {
  m_commands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousZeroNote::Execute() {
  m_commands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousZeroNote::End(bool interrupted) {
  m_commands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousZeroNote::IsFinished() {
  return m_commands.IsFinished();
}

std::string AutonomousZeroNote::GetName() const {
  return "11. Zero Note - Just Move";
}

frc2::Command* AutonomousZeroNote::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
