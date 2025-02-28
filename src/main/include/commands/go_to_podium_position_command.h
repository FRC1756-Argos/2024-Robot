/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

class GoToPodiumPositionCommand : public frc2::CommandHelper<frc2::Command, GoToPodiumPositionCommand> {
 public:
  GoToPodiumPositionCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator, bool highPodiumShot);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  bool m_highPodiumShot;
};
