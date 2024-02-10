/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <chrono>

#include "constants/measure_up.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShooterCommand : public frc2::CommandHelper<frc2::Command, ShooterCommand> {
 public:
  ShooterCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
};
