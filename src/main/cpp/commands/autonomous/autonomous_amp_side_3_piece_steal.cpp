/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_amp_side_3_piece_steal.h"

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

AutonomousAmpSideSubwoofer3PieceSteal::AutonomousAmpSideSubwoofer3PieceSteal(
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
          PrimeShooterCommand{m_Shooter, m_Elevator, m_Vision, 43_in, 2000_rpm},
          ShooterCommand{&m_Shooter, true},
          frc2::InstantCommand{[this]() {
                                 m_Shooter.ShooterGoToSpeed(m_Vision.getShooterSpeed(
                                     15_ft, VisionSubsystem::InterpolationMode::LinearInterpolation));
                               },
                               {&m_Shooter}},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Amp_Side_3_Mid.1", true},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 1.5_s}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Amp_Side_3_Mid.2", false},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 1.5_s}},
          AutoAimCommand{&swerve, &shooter, &elevator, &vision, &controllers, &leds, true},
          ShooterCommand{&m_Shooter, true},
          frc2::ParallelCommandGroup{
              DriveChoreo{m_Swerve, "Amp_Side_3_Mid.3", false},
              IntakeCommand{&m_Intake, &m_Shooter, &m_Elevator, &controllers, &leds, true, 1.5_s}}}} {}

// Called when the command is initially scheduled.
void AutonomousAmpSideSubwoofer3PieceSteal::Initialize() {
  m_SeqCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousAmpSideSubwoofer3PieceSteal::Execute() {
  m_SeqCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousAmpSideSubwoofer3PieceSteal::End(bool interrupted) {
  m_SeqCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousAmpSideSubwoofer3PieceSteal::IsFinished() {
  return m_SeqCommands.IsFinished();
}

std::string AutonomousAmpSideSubwoofer3PieceSteal::GetName() const {
  return "12. Amp Side Subwoofer 3 Piece Steal";
}

frc2::Command* AutonomousAmpSideSubwoofer3PieceSteal::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
