/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <string>

#include "commands/autonomous/autonomous_command.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"

class AutonomousSourceSideSubwoofer3
    : public frc2::CommandHelper<frc2::Command, AutonomousSourceSideSubwoofer3>
    , public AutonomousCommand {
 public:
  AutonomousSourceSideSubwoofer3(IntakeSubsystem& intake,
                                 ShooterSubsystem& shooter,
                                 ElevatorSubsystem& elevator,
                                 SwerveDriveSubsystem& swerve,
                                 VisionSubsystem& vision,
                                 argos_lib::SwappableControllersSubsystem& controllers,
                                 SimpleLedSubsystem& leds);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  /**
   * @copydoc AutonomousCommand::GetName()
   */
  std::string GetName() const final;
  /**
   * @copydoc AutonomousCommand::GetCommand()
   */
  frc2::Command* GetCommand() final;

 private:
  IntakeSubsystem& m_Intake;
  ShooterSubsystem& m_Shooter;
  ElevatorSubsystem& m_Elevator;
  SwerveDriveSubsystem& m_Swerve;
  VisionSubsystem& m_Vision;
  frc2::SequentialCommandGroup m_SeqCommands;
};
