/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_source2.h"

#include <frc2/command/ConditionalCommand.h>
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

AutonomousSource2::AutonomousSource2(IntakeSubsystem& intake,
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
          AutonomousSource1{m_Intake, m_Shooter, m_Elevator, m_Swerve, m_Vision, controllers, leds},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.2", false},
                                     IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}},
          frc2::ConditionalCommand{
              frc2::SequentialCommandGroup{
                  // Got note!
                  frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.3", false},
                                             PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
                  AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                  ShooterCommand{&m_Shooter, true},
                  frc2::ParallelCommandGroup{
                      DriveChoreo{m_Swerve, "Source1preload.4", false},
                      IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}}},
              frc2::ParallelCommandGroup{
                  // Missed note
                  DriveChoreo{m_Swerve, "Source1preloadShortcut.1", false},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
              [&shooter]() { return shooter.IsNotePresent(); }},
          // frc2::ConditionalCommand{
          //     frc2::SequentialCommandGroup{
          //         // Got note!
          //         frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Source1preload.5", false},
          //                                    PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          //         AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          //         ShooterCommand{&m_Shooter, true}},
          //     frc2::ParallelCommandGroup{
          //         // Missed note
          //         DriveChoreo{m_Swerve, "Source1preloadShortcut.2", false},
          //         IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          //     [&shooter]() { return shooter.IsNotePresent(); }},
          frc2::ParallelCommandGroup{
              DriveChoreo{
                  m_Swerve, "Source1preload.5", false},  // remove this command when we reincorporate the conditional.
              PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true}}} {}

// Called when the command is initially scheduled.
void AutonomousSource2::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousSource2::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousSource2::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousSource2::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousSource2::GetName() const {
  return "10. 2 Preloaded Source ";
}

frc2::Command* AutonomousSource2::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
