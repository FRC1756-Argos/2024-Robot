/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_source_side_subwoofer_steal2.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/length.h>

#include "commands/auto_aim_command.h"
#include "commands/autonomous/autonomous_source1.h"
#include "commands/drive_choreo.h"
#include "commands/intake_command.h"
#include "commands/prime_shooter_command.h"
#include "commands/shooter_command.h"
#include "subsystems/shooter_subsystem.h"

AutonomousSourceSideSteal2::AutonomousSourceSideSteal2(IntakeSubsystem& intake,
                                                       ShooterSubsystem& shooter,
                                                       ElevatorSubsystem& elevator,
                                                       SwerveDriveSubsystem& swerve,
                                                       VisionSubsystem& vision,
                                                       argos_lib::SwappableControllersSubsystem& controllers,
                                                       SimpleLedSubsystem& leds)
    : m_Intake{intake}
    , m_Shooter{shooter}
    , m_Elevator{elevator}
    , m_Swerve{swerve}
    , m_Vision{vision}
    , m_SeqCommands{frc2::SequentialCommandGroup{
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 43_in, 2000_rpm},
          ShooterCommand{&m_Shooter, true},
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 10_ft, 5000_rpm, false},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Source_Side_Subwoofer_Steal2.1", true},  ///< @todo: Update to match choreo
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 1.5_s}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source_Side_Subwoofer_Steal2.2", false},
                                     IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true}},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source_Side_Subwoofer_Steal2.3", false},
                                     PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source_Side_Subwoofer_Steal2.4", false},
                                     IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true}},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source_Side_Subwoofer_Steal2.5", false},
                                     PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true}}} {}

// Called when the command is initially scheduled.
void AutonomousSourceSideSteal2::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousSourceSideSteal2::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousSourceSideSteal2::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousSourceSideSteal2::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousSourceSideSteal2::GetName() const {
  return "09. Source-side subwoofer steal 2";
}

frc2::Command* AutonomousSourceSideSteal2::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
