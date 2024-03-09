/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_source_side_subwoofer3.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/length.h>

#include "commands/auto_aim_command.h"
#include "commands/drive_choreo.h"
#include "commands/intake_command.h"
#include "commands/prime_shooter_command.h"
#include "commands/shooter_command.h"
#include "subsystems/shooter_subsystem.h"

AutonomousSourceSideSubwoofer3::AutonomousSourceSideSubwoofer3(IntakeSubsystem& intake,
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
    , m_Vision{vision}  //, m_ShooterCommand{ShooterCommand{*shooter}}
    , m_SeqCommands{frc2::SequentialCommandGroup{
          frc2::ParallelCommandGroup{PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft},
                                     DriveChoreo{m_Swerve, "Source_Side_Subwoofer_3", true}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.2", false},
                                     IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.3", false},
                                     PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.4", false},
                                     IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.5", false},
                                     PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true}}} {}

// Called when the command is initially scheduled.
void AutonomousSourceSideSubwoofer3::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousSourceSideSubwoofer3::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousSourceSideSubwoofer3::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousSourceSideSubwoofer3::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousSourceSideSubwoofer3::GetName() const {
  return "08. 1 Preloaded Source-side subwoofer ";
}

frc2::Command* AutonomousSourceSideSubwoofer3::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
