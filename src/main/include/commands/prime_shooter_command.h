/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/length.h>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/vision_subsystem.h"

class PrimeShooterCommand : public frc2::CommandHelper<frc2::Command, PrimeShooterCommand> {
 public:
  PrimeShooterCommand(ShooterSubsystem& shooter,
                      ElevatorSubsystem& elevator,
                      VisionSubsystem& vision,
                      const units::inch_t distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem& m_Shooter;
  ElevatorSubsystem& m_Elevator;
  VisionSubsystem& m_Vision;
  const units::inch_t m_distance;
};
