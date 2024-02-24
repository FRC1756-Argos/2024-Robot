/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/simple_led_subsystem.h"

class IntakeCommand : public frc2::CommandHelper<frc2::Command, IntakeCommand> {
 public:
  IntakeCommand(IntakeSubsystem* intake,
                ShooterSubsystem* shooter,
                ElevatorSubsystem* elevator,
                argos_lib::SwappableControllersSubsystem* controllers,
                SimpleLedSubsystem* leds);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  IntakeSubsystem* m_pIntake;
  ShooterSubsystem* m_pShooter;
  ElevatorSubsystem* m_pElevator;
  argos_lib::SwappableControllersSubsystem* m_pControllers;
  SimpleLedSubsystem* m_pLeds;
};
