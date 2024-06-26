/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_summer_activity.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "commands/drive_choreo.h"

AutonomousSummerActivty::AutonomousSummerActivty(
                                                IntakeSubsystem& intake,
                                                ShooterSubsystem& shooter,
                                                ElevatorSubsystem& elevator,
                                                ClimberSubsystem& climb,
                                                SwerveDriveSubsystem& swerve)
    : m_Intake{intake},
     m_Shooter{shooter},
     m_Elevator{elevator},
     m_Climb{climb},
     m_Swerve{swerve},
     m_commands{frc2::SequentialCommandGroup{DriveChoreo{m_Swerve, "Any_0Note", true}}} {}

// Called when the command is initially scheduled.
void AutonomousSummerActivty::Initialize() {
  m_commands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousSummerActivty::Execute() {
  m_commands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousSummerActivty::End(bool interrupted) {
  m_commands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousSummerActivty::IsFinished() {
  return m_commands.IsFinished();
}

std::string AutonomousSummerActivty::GetName() const {
  return "12. Summer Activity";
}

frc2::Command* AutonomousSummerActivty::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
