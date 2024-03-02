/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/config/config_types.h>
#include <argos_lib/general/generic_debouncer.h>
#include <argos_lib/subsystems/swappable_controllers_subsystem.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/auto_aim_command.h"
#include "commands/autonomous/autonomous_choreo_test.h"
#include "commands/autonomous/autonomous_nothing.h"
#include "commands/autonomous/autonomous_source1.h"
#include "commands/climber_command.h"
#include "commands/climber_homing_command.h"
#include "commands/go_to_amp_position_command.h"
#include "commands/go_to_podium_position_command.h"
#include "commands/go_to_subwoofer_position_command.h"
#include "commands/go_to_trap_position_command.h"
#include "commands/intake_command.h"
#include "commands/lower_climber_command.h"
#include "commands/raise_climber_command.h"
#include "commands/ready_for_climb_command.h"
#include "commands/shooter_command.h"
#include "subsystems/climber_subsystem.h"
#include "subsystems/elevator_subsystem.h"
#include "subsystems/intake_subsystem.h"
#include "subsystems/shooter_subsystem.h"
#include "subsystems/simple_led_subsystem.h"
#include "subsystems/swerve_drive_subsystem.h"
#include "subsystems/vision_subsystem.h"
#include "utils/auto_selector.h"

/**
 * @brief  Command-based is a "declarative" paradigm, very little robot logic should
 *         actually be handled in the {@link Robot} periodic methods (other than the
 *         scheduler calls).  Instead, the structure of the robot (including subsystems,
 *         commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  /// @brief Called once when robot is disabled
  void Disable();

  /// @brief Called once when robot is enabled
  void Enable();

  /// @brief Called when the alliance is changed
  void AllianceChanged();

  void SetLedsConnectedBrightness(bool connected);

 private:
  // Interpolation of controller inputs. Used for making the inputs non-linear, allowing finer control of how the robot responds to the joystick.
  argos_lib::InterpolationMap<decltype(controllerMap::driveSpeed.front().inVal), controllerMap::driveSpeed.size()>
      m_driveSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::driveRotSpeed.front().inVal), controllerMap::driveRotSpeed.size()>
      m_driveRotSpeed;
  argos_lib::InterpolationMap<decltype(controllerMap::elevatorSpeed.front().inVal), controllerMap::elevatorSpeed.size()>
      m_elevatorSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::elevatorRotateSpeed.front().inVal),
                              controllerMap::elevatorRotateSpeed.size()>
      m_elevatorRotateSpeedMap;
  argos_lib::InterpolationMap<decltype(controllerMap::climberSpeed.front().inVal), controllerMap::climberSpeed.size()>
      m_climberSpeedMap;

  const argos_lib::RobotInstance m_instance;

  // The robot's subsystems are defined here...
  argos_lib::SwappableControllersSubsystem m_controllers;
  SwerveDriveSubsystem m_swerveDrive;
  SimpleLedSubsystem m_ledSubSystem;
  VisionSubsystem m_visionSubSystem;
  ShooterSubsystem m_ShooterSubSystem;
  IntakeSubsystem m_intakeSubsystem;
  ClimberSubsystem m_climberSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;

  IntakeCommand m_IntakeCommand;
  ShooterCommand m_ShooterCommand;
  AutoAimCommand m_autoAimCommand;
  ClimberHomingCommand m_ClimberHomeCommand;
  GoToAmpPositionCommand m_GoToAmpPositionCommand;
  GoToPodiumPositionCommand m_GoToHighPodiumPositionCommand;
  GoToPodiumPositionCommand m_GoToLowPodiumPositionCommand;
  GoToSubwooferPositionCommand m_GoToSubwooferPositionCommand;
  GoToTrapPositionCommand m_GoToTrapPositionCommand;
  ClimberCommand m_ClimberCommand;

  // Autonomous
  AutonomousNothing m_autoNothing;
  AutonomousChoreoTest m_autoChoreoTest;
  AutonomousCenter2Wing m_autoCenter2wing;
  AutonomousSource1 m_autoSource1;

  AutoSelector m_autoSelector;

  void ConfigureBindings();

  /* —————————————————— PID TESTING SETPOINT NT OBJECTS —————————————————— */

  nt::GenericEntry* p_wristSetpoint;
  frc::SlewRateLimiter<units::scalar> m_lateralNudgeRate;
  frc::SlewRateLimiter<units::scalar> m_rotationalNudgeRate;
  frc::SlewRateLimiter<units::scalar> m_distanceNudgeRate;
  argos_lib::GenericDebouncer<AlignLedStatus> m_alignLedDebouncer;
};
