/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center2_wing.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/length.h>

#include "commands/auto_aim_command.h"
#include "commands/drive_choreo.h"
#include "commands/intake_command.h"
#include "commands/prime_shooter_command.h"
#include "subsystems/shooter_subsystem.h"

AutonomousCenter2Wing::AutonomousCenter2Wing(ElevatorSubsystem& elevator,
                                             IntakeSubsystem& intake,
                                             ShooterSubsystem& shooter,
                                             SwerveDriveSubsystem& swerve,
                                             VisionSubsystem& vision)
    : m_Elevator{elevator}
    , m_Intake{intake}
    , m_Shooter{shooter}
    , m_Swerve{swerve}
    , m_Vision{vision}  //, m_ShooterCommand{ShooterCommand{*shooter}}
    , m_allCommands{frc2::SequentialCommandGroup{
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 12_ft},
          ShooterCommand{&m_Shooter},
          // frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "center2wing", true},IntakeCommand{&m_Intake}},
          ShooterCommand{&m_Shooter}}} {}

// Called when the command is initially scheduled.
void AutonomousCenter2Wing::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenter2Wing::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenter2Wing::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenter2Wing::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousCenter2Wing::GetName() const {
  return "99. Choreo Test";
}

frc2::Command* AutonomousCenter2Wing::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
