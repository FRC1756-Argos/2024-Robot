/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"

class ReadyForClimbCommand : public frc2::CommandHelper<frc2::Command, ReadyForClimbCommand> {
 public:
  explicit ReadyForClimbCommand(ShooterSubsystem* shooter, ElevatorSubsystem* elevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  bool GetIsReadyCLimbFinished();

 private:
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  bool is_ready_climb_finished;
};