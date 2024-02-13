// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <argos_lib/general/debouncer.h>
#include <subsystems/climber_subsystem.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimberHomingCommand
    : public frc2::CommandHelper<frc2::Command, ClimberHomingCommand> {
 public:
  explicit ClimberHomingCommand(ClimberSubsystem& subsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
 ClimberSubsystem& m_climberSubsystem;
 argos_lib::Debouncer m_climberMovingDebounce;
 std::chrono::time_point<std::chrono::steady_clock> m_startTime;


};
