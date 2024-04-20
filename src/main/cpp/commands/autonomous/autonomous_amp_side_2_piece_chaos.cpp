/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_amp_side_2_piece_chaos.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "commands/auto_aim_command.h"
#include "commands/drive_choreo.h"
#include "commands/intake_command.h"
#include "commands/prime_shooter_command.h"
#include "commands/shooter_command.h"
#include "subsystems/shooter_subsystem.h"

AutonomousAmpSideSubwoofer2PieceChaos::AutonomousAmpSideSubwoofer2PieceChaos(
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
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP.1", true},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 1.5_s}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 43_in, 1600_rpm, false},
          frc2::ParallelRaceGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP.2", true},
              frc2::SequentialCommandGroup{
                  ShooterCommand{&m_Shooter, true, 500_ms},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}}},
          frc2::ParallelRaceGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP_Shortcut.2", true},
              frc2::SequentialCommandGroup{
                  ShooterCommand{&m_Shooter, true, 500_ms},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}}},
          frc2::ParallelRaceGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP_Shortcut.3", true},
              frc2::SequentialCommandGroup{
                  ShooterCommand{&m_Shooter, true, 500_ms},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}}},
          frc2::ParallelRaceGroup{
              DriveChoreo{m_Swerve, "Amp_Side_Subwoofer_OP_Shortcut.4", true},
              frc2::SequentialCommandGroup{
                  ShooterCommand{&m_Shooter, true, 500_ms},
                  IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 4_s}}},
          ShooterCommand{&m_Shooter, true, 500_ms}}} {}

// Called when the command is initially scheduled.
void AutonomousAmpSideSubwoofer2PieceChaos::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousAmpSideSubwoofer2PieceChaos::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousAmpSideSubwoofer2PieceChaos::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousAmpSideSubwoofer2PieceChaos::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousAmpSideSubwoofer2PieceChaos::GetName() const {
  return "01.2 ** Amp Side Subwoofer OP (CHAOS) **";
}

frc2::Command* AutonomousAmpSideSubwoofer2PieceChaos::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
