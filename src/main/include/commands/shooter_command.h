/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "constants/measure_up.h"
#include "subsystems/shooter_subsystem.h"

class ShooterCommand : public frc2::CommandHelper<frc2::Command, ShooterCommand> {
 public:
  explicit ShooterCommand(ShooterSubsystem* shooter);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
};
