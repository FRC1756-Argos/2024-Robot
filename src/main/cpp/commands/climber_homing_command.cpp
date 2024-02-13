// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/climber_homing_command.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>

using namespace std::chrono_literals;

ClimberHomingCommand::ClimberHomingCommand(ClimberSubsystem& subsystem)
    :m_climberSubsystem(subsystem), m_climberMovingDebounce{{0_ms, 250_ms}, true} {}
  // Use addRequirements() here to declare subsystem dependencies.
// Called when the command is initially scheduled.
void ClimberHomingCommand::Initialize() {
  m_climberSubsystem.SetExtensionSpeed(-0.1);
  m_climberSubsystem.SetManualOverride(false);
  m_startTime = std::chrono::steady_clock::now();
  m_climberMovingDebounce.Reset(true);
}

// Called repeatedly when this Command is scheduled to run
void ClimberHomingCommand::Execute() {
  auto timePassed = units::second_t{std::chrono::steady_clock::now() - m_startTime};

  if (m_climberSubsystem.IsClimberManualOverride() || (timePassed) > 1.5_s) {
    Cancel();

  } else {
    m_climberSubsystem.SetExtensionSpeed(-0.1);
  }
}

// Called once the command ends or is interrupted.
void ClimberHomingCommand::End(bool interrupted) {
  if (!interrupted) {
    m_climberSubsystem.UpdateClimberHome();
    m_climberSubsystem.SetHomeFailed(false);
  } else {
    m_climberSubsystem.SetExtensionSpeed(0.0);
    m_climberSubsystem.SetHomeFailed(true);
  }
}

// Returns true when the command should end.
bool ClimberHomingCommand::IsFinished() {
  return !m_climberMovingDebounce(m_climberSubsystem.IsClimberMoving());
}
