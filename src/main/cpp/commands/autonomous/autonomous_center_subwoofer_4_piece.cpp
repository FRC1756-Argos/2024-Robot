/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_center_subwoofer_4_piece.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "commands/auto_aim_command.h"
#include "commands/drive_choreo.h"
#include "commands/intake_command.h"
#include "commands/prime_shooter_command.h"
#include "commands/shooter_command.h"
#include "subsystems/shooter_subsystem.h"

AutonomousCenterSubwoofer4Piece::AutonomousCenterSubwoofer4Piece(IntakeSubsystem& intake,
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
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 15_ft, std::nullopt, false},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Center_Subwoofer_4.1", true},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          frc2::ConditionalCommand{// Got note
                                   frc2::SequentialCommandGroup{
                                       AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                                       ShooterCommand{&m_Shooter, true}},
                                   frc2::InstantCommand([]() {}, {}),  // No note
                                   [&shooter]() { return shooter.IsNotePresent(); }},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Center_Subwoofer_4.2", false},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          frc2::ConditionalCommand{// Got note
                                   frc2::SequentialCommandGroup{
                                       AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                                       ShooterCommand{&m_Shooter, true}},
                                   frc2::InstantCommand([]() {}, {}),  // No note
                                   [&shooter]() { return shooter.IsNotePresent(); }},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Center_Subwoofer_4.3", false},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          frc2::ConditionalCommand{// Got note
                                   frc2::SequentialCommandGroup{
                                       AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                                       ShooterCommand{&m_Shooter, true}},
                                   frc2::InstantCommand([]() {}, {}),  // No note
                                   [&shooter]() { return shooter.IsNotePresent(); }}}} {}

// Called when the command is initially scheduled.
void AutonomousCenterSubwoofer4Piece::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousCenterSubwoofer4Piece::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousCenterSubwoofer4Piece::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousCenterSubwoofer4Piece::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousCenterSubwoofer4Piece::GetName() const {
  return "03. Center Subwoofer 4 Piece";
}

frc2::Command* AutonomousCenterSubwoofer4Piece::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
