/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center_subwoofer_6_piece.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "commands/auto_aim_command.h"
#include "commands/autonomous/autonomous_center_subwoofer_4_piece.h"
#include "commands/drive_choreo.h"
#include "commands/intake_command.h"
#include "commands/prime_shooter_command.h"
#include "commands/shooter_command.h"
#include "subsystems/shooter_subsystem.h"

AutonomousCenterSubwoofer6Piece::AutonomousCenterSubwoofer6Piece(IntakeSubsystem& intake,
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
          AutonomousCenterSubwoofer4Piece{m_Intake, m_Shooter, m_Elevator, m_Swerve, m_Vision, controllers, leds},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Center_Subwoofer_4.4", false},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 3.0_s}},

          frc2::ConditionalCommand{
              frc2::SequentialCommandGroup{
                  frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Center_Subwoofer_4.5", false},
                                             PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
                  AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                  ShooterCommand{&m_Shooter, true},
                  frc2::ParallelCommandGroup{
                      DriveChoreo{m_Swerve, "Center_Subwoofer_4.6", false},
                      IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.7_s}}},
              frc2::ParallelCommandGroup{
                  DriveChoreo{m_Swerve, "Center_Subwoofer_4_Shortcut.1", false},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
              [&shooter]() { return shooter.IsNotePresent(); }},
          frc2::ParallelCommandGroup{DriveChoreo{m_Swerve, "Center_Subwoofer_4.7", false},
                                     PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true}}} {}

// Called when the command is initially scheduled.
void AutonomousCenterSubwoofer6Piece::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenterSubwoofer6Piece::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenterSubwoofer6Piece::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenterSubwoofer6Piece::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousCenterSubwoofer6Piece::GetName() const {
  return "04. Center Subwoofer 6 Piece";
}

frc2::Command* AutonomousCenterSubwoofer6Piece::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
