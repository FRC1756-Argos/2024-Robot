/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <argos_lib/general/debouncer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <subsystems/climber_subsystem.h>

class ClimberHomingCommand : public frc2::CommandHelper<frc2::Command, ClimberHomingCommand> {
 public:
  explicit ClimberHomingCommand(ClimberSubsystem& subsystem, const argos_lib::RobotInstance instance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ClimberSubsystem& m_climberSubsystem;
  argos_lib::Debouncer m_climberMovingDebounce;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
  const argos_lib::RobotInstance m_instance;
};
