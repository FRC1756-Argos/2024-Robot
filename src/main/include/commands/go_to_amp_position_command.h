/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

class GoToAmpPositionCommand : public frc2::CommandHelper<frc2::Command, GoToAmpPositionCommand> {
 public:
  GoToAmpPositionCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
};
