/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/time.h>

#include <chrono>

#include "constants/measure_up.h"
#include "subsystems/shooter_subsystem.h"

class ShooterCommand : public frc2::CommandHelper<frc2::Command, ShooterCommand> {
 public:
  explicit ShooterCommand(ShooterSubsystem* shooter, bool endAfterShot = false, units::millisecond_t timeout = 500_ms);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
  bool m_endAfterShot;
  bool m_notePresent;
  bool m_noteShot;
  units::millisecond_t m_timeout;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
};
