/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_amp_side_center_rush.h"

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

AutonomousAmpSideSubwooferCenterRush::AutonomousAmpSideSubwooferCenterRush(
    IntakeSubsystem& intake,
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
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 43_in, 2500_rpm},
          ShooterCommand{&m_Shooter, true},
          frc2::InstantCommand{[this]() {
                                 m_Shooter.ShooterGoToSpeed(m_Vision.getShooterSpeed(
                                     15_ft, VisionSubsystem::InterpolationMode::LinearInterpolation));
                               },
                               {&m_Shooter}},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.1", true},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          frc2::ConditionalCommand{
              frc2::SequentialCommandGroup{
                  DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.3"},
                  AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                  ShooterCommand{&m_Shooter, true},
                  frc2::ParallelCommandGroup{
                      DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.4", true},
                      IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}}},
              frc2::ParallelCommandGroup{
                  DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP_Shortcut.1", true},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
              [&shooter]() { return shooter.IsNotePresent(); }},
          frc2::ConditionalCommand{
              frc2::SequentialCommandGroup{
                  DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.5"},
                  AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
                  ShooterCommand{&m_Shooter, true},
                  frc2::ParallelCommandGroup{
                      DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.6", true},
                      IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}}},
              frc2::ParallelCommandGroup{
                  DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP_Shortcut.2", true},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
              [&shooter]() { return shooter.IsNotePresent(); }},
          DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.7", true},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP_Shortcut.2", true},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.8", true},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 2.5_s}},
          DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_Center_Rush.9", true},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
      }} {}

// Called when the command is initially scheduled.
void AutonomousAmpSideSubwooferCenterRush::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousAmpSideSubwooferCenterRush::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousAmpSideSubwooferCenterRush::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousAmpSideSubwooferCenterRush::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousAmpSideSubwooferCenterRush::GetName() const {
  return "0. Amp Side Subwoofer Center Rush";
}

frc2::Command* AutonomousAmpSideSubwooferCenterRush::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
