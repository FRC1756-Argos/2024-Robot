/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/prime_shooter_command.h"

#include "constants/measure_up.h"

PrimeShooterCommand::PrimeShooterCommand(ShooterSubsystem& shooter,
                                         ElevatorSubsystem& elevator,
                                         VisionSubsystem& vision,
                                         const units::inch_t distance,
                                         const std::optional<units::revolutions_per_minute_t> customSpeed)
    : m_Shooter{shooter}, m_Elevator{elevator}, m_Vision{vision}, m_distance{distance}, m_customSpeed{customSpeed} {
  AddRequirements({&m_Shooter, &m_Elevator});
}

// Called when the command is initially scheduled.
void PrimeShooterCommand::Initialize() {
  if (m_customSpeed.has_value()) {
    m_Shooter.ShooterGoToSpeed(m_customSpeed.value());
  } else {
    m_Shooter.ShooterGoToSpeed(m_Vision.getShooterSpeed(m_distance, VisionSubsystem::InterpolationMode::Polynomial));
  }
  m_Elevator.ElevatorMoveToHeight(measure_up::elevator::lift::intakeHeight);
  m_Elevator.SetCarriageAngle(m_Vision.getShooterAngle(m_distance, VisionSubsystem::InterpolationMode::Polynomial));
}

// Called repeatedly when this Command is scheduled to run
void PrimeShooterCommand::Execute() {
  if (m_Elevator.GetElevatorLiftManualOverride() || m_Elevator.IsCarriageMotorManualOverride()) {
    Cancel();
  }
}

// Called once the command ends or is interrupted.
void PrimeShooterCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool PrimeShooterCommand::IsFinished() {
  return m_Shooter.ShooterAtSpeed() && m_Elevator.IsElevatorAtSetPoint();
}
