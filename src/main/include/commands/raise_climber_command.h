/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "constants/measure_up.h"
#include "subsystems/climber_subsystem.h"

class RaiseClimberCommand : public frc2::CommandHelper<frc2::Command, RaiseClimberCommand> {
 public:
  explicit RaiseClimberCommand(ClimberSubsystem* climber);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  bool GetIsRaiseCLimbFinished();

 private:
  ClimberSubsystem* m_pClimber;
  bool is_raise_climb_finished;
};
