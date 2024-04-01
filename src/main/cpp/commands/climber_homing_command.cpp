/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climber_homing_command.h"

#include <chrono>

#include "constants/measure_up.h"

using namespace std::chrono_literals;

ClimberHomingCommand::ClimberHomingCommand(ClimberSubsystem& subsystem, const argos_lib::RobotInstance instance)
    : m_climberSubsystem(subsystem), m_climberMovingDebounce{{0_ms, 250_ms}, true}, m_instance{instance} {
  AddRequirements({&m_climberSubsystem});
}

void ClimberHomingCommand::Initialize() {
  m_climberSubsystem.ClimberMove(-0.07, true);
  m_climberSubsystem.SetClimberManualOverride(false);
  m_startTime = std::chrono::steady_clock::now();
  m_climberMovingDebounce.Reset(true);
}

// Called repeatedly when this Command is scheduled to run
void ClimberHomingCommand::Execute() {
  auto timePassed = units::second_t{std::chrono::steady_clock::now() - m_startTime};

  if (m_climberSubsystem.IsClimberManualOverride() || (timePassed) > 3_s) {
    Cancel();

  } else {
    m_climberSubsystem.ClimberMove(-0.07, true);
  }
}

// Called once the command ends or is interrupted.
void ClimberHomingCommand::End(bool interrupted) {
  m_climberSubsystem.ClimberMove(0.0, !m_climberSubsystem.IsClimberManualOverride());
  if (!interrupted) {
    m_climberSubsystem.UpdateClimberHome();
    m_climberSubsystem.SetHomeFailed(false);
  } else {
    m_climberSubsystem.SetHomeFailed(true);
  }
}

// Returns true when the command should end.
bool ClimberHomingCommand::IsFinished() {
  return !m_climberMovingDebounce(m_climberSubsystem.IsClimberMoving());
}
