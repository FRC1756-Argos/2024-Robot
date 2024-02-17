/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "RobotContainer.h"

#include <argos_lib/commands/swap_controllers_command.h>
#include <argos_lib/controller/trigger_composition.h>
#include <argos_lib/general/angle_utils.h>
#include <argos_lib/general/color.h>
#include <argos_lib/general/swerve_utils.h>
#include <frc/DriverStation.h>
#include <frc/RobotState.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/angular_velocity.h>
#include <units/length.h>

// Include GamePiece enum
#include <constants/field_points.h>
#include <Constants.h>

#include <cmath>
#include <memory>
#include <optional>

#include "Constants.h"
#include "argos_lib/subsystems/led_subsystem.h"
#include "commands/drive_to_position.h"
#include "commands/intake_command.h"
#include "utils/custom_units.h"

RobotContainer::RobotContainer()
    : m_driveSpeedMap(controllerMap::driveSpeed)
    , m_driveRotSpeed(controllerMap::driveRotSpeed)
    , m_elevatorSpeedMap(controllerMap::elevatorSpeed)
    , m_elevatorRotateSpeedMap(controllerMap::elevatorRotateSpeed)
    , m_climberSpeedMap(controllerMap::climberSpeed)
    , m_instance(argos_lib::GetRobotInstance())
    , m_controllers(address::comp_bot::controllers::driver, address::comp_bot::controllers::secondary)
    , m_swerveDrive(m_instance)
    , m_ledSubSystem(m_instance)
    , m_visionSubSystem(m_instance, &m_swerveDrive)
    , m_ShooterSubSystem(m_instance)
    , m_intakeSubsystem(m_instance)
    , m_climberSubsystem(m_instance)
    , m_elevatorSubsystem(m_instance)
    , m_autoNothing{}
    , m_autoSelector{{&m_autoNothing}, &m_autoNothing}
    , m_lateralNudgeRate{12 / 1_s}
    , m_rotationalNudgeRate{4 / 1_s}
    , m_distanceNudgeRate{12 / 1_s}
    , m_alignLedDebouncer{50_ms}
    , m_IntakeCommand{&m_intakeSubsystem, &m_ShooterSubSystem, &m_elevatorSubsystem}
    , m_ShooterCommand{&m_ShooterSubSystem}
    , m_autoAimCommand{&m_swerveDrive, &m_ShooterSubSystem, &m_elevatorSubsystem, &m_visionSubSystem} {
  // Initialize all of your commands and subsystems here

  AllianceChanged();

  // ================== DEFAULT COMMANDS ===============================
  m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        auto deadbandTranslationSpeeds = argos_lib::swerve::CircularInterpolate(
            argos_lib::swerve::TranslationSpeeds{
                -m_controllers.DriverController().GetY(
                    argos_lib::XboxController::JoystickHand::kLeftHand),  // Y axis is negative forward
                -m_controllers.DriverController().GetX(
                    argos_lib::XboxController::JoystickHand::
                        kLeftHand)},  // X axis is positive right, but swerve coordinates are positive left
            m_driveSpeedMap);
        auto deadbandRotSpeed = m_driveRotSpeed(
            -m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));

        if (frc::DriverStation::IsTeleop() &&
            (m_swerveDrive.GetManualOverride() || deadbandTranslationSpeeds.forwardSpeedPct != 0 ||
             deadbandTranslationSpeeds.leftSpeedPct != 0 || deadbandRotSpeed != 0)) {
          m_swerveDrive.SwerveDrive(
              deadbandTranslationSpeeds.forwardSpeedPct,
              deadbandTranslationSpeeds.leftSpeedPct,
              deadbandRotSpeed);  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)
        }
        // DEBUG STUFF
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left Y",
            m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
        frc::SmartDashboard::PutNumber(
            "(DRIVER) Joystick Left X",
            m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
      },
      {&m_swerveDrive}));

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  frc::SmartDashboard::PutNumber("elevator/Height (in)", 12.0);
  frc::SmartDashboard::PutNumber("elevator/Angle (deg)", 0.0);
  frc::SmartDashboard::PutNumber("shooter/Speed (rpm)", 3000);

  /* ———————————————————————— CONFIGURE DEBOUNCING ——————————————————————— */

  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kY, {1500_ms, 0_ms});

  /* —————————————————————————————— TRIGGERS ————————————————————————————— */

  auto robotEnableTrigger = (frc2::Trigger{[this]() { return frc::DriverStation::IsEnabled(); }});

  // DRIVE TRIGGERS
  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kY);

  // INTAKE TRIGGERS
  auto intake = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger);
  auto outtakeManual = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);

  // CLIMBER TRIGGERS
  auto climberUp = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kUp);
  auto climberDown = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kDown);

  // SHOOT TRIGGERS
  auto shootManual = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);
  auto feedForward = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kUp);
  auto feedBackward = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kDown);
  auto shoot = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto aim = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);

  auto closedLoopSet = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kA);

  // ELEVATOR TRIGGERS
  auto elevatorLiftManualInput = (frc2::Trigger{[this]() {
    return std::abs(m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand)) > 0.2;
  }});

  auto overrideCarriageTrigger = (frc2::Trigger([this]() {
    return std::abs(m_controllers.OperatorController().GetY(argos_lib::XboxController::JoystickHand::kRightHand)) > 0.2;
  }));

  // Swap controllers config
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.DriverController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kBack, {1500_ms, 0_ms});
  m_controllers.OperatorController().SetButtonDebounce(argos_lib::XboxController::Button::kStart, {1500_ms, 0_ms});

  // SWAP CONTROLLER TRIGGERS
  frc2::Trigger driverTriggerSwapCombo = m_controllers.DriverController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  frc2::Trigger operatorTriggerSwapCombo = m_controllers.OperatorController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});

  /* ————————————————————————— TRIGGER ACTIVATION ———————————————————————— */

  // DRIVE TRIGGER ACTIVATION
  fieldHome.OnTrue(frc2::InstantCommand([this]() { m_swerveDrive.FieldHome(); }, {&m_swerveDrive}).ToPtr());

  // INTAKE TRIGGER ACTIVATION
  outtakeManual.OnTrue(
      frc2::InstantCommand([this]() { m_intakeSubsystem.Intake(-0.8); }, {&m_intakeSubsystem}).ToPtr());
  (outtakeManual)
      .OnFalse(frc2::InstantCommand([this]() { m_intakeSubsystem.Intake(0.0); }, {&m_intakeSubsystem}).ToPtr());

  intake.WhileTrue(&m_IntakeCommand);
  shoot.WhileTrue(&m_ShooterCommand);
  aim.WhileTrue(&m_autoAimCommand);

  // CLIMBER TRIGGER ACTIVATION
  climberUp.OnTrue(frc2::InstantCommand(
                       [this]() {
                         m_climberSubsystem.SetManualOverride(true);
                         m_climberSubsystem.ClimberMove(0.2);
                       },
                       {&m_climberSubsystem})
                       .ToPtr());
  climberDown.OnTrue(frc2::InstantCommand(
                         [this]() {
                           m_climberSubsystem.SetManualOverride(true);
                           m_climberSubsystem.ClimberMove(-0.2);
                         },
                         {&m_climberSubsystem})
                         .ToPtr());
  (climberUp || climberDown)
      .OnFalse(frc2::InstantCommand([this]() { m_climberSubsystem.ClimberMove(0.0); }, {&m_climberSubsystem}).ToPtr());

  // ELEVATOR TRIGGER ACTIVATION
  elevatorLiftManualInput.OnTrue(
      frc2::InstantCommand([this]() { m_elevatorSubsystem.SetElevatorLiftManualOverride(true); }, {}).ToPtr());
  m_elevatorSubsystem.SetDefaultCommand(frc2::RunCommand(
                                            [this] {
                                              double elevatorSpeed = m_controllers.OperatorController().GetY(
                                                  argos_lib::XboxController::JoystickHand::kLeftHand);
                                              double carriageSpeed = m_controllers.OperatorController().GetY(
                                                  argos_lib::XboxController::JoystickHand::kRightHand);
                                              m_elevatorSubsystem.ElevatorMove(m_elevatorSpeedMap(elevatorSpeed));
                                              m_elevatorSubsystem.Pivot(m_elevatorRotateSpeedMap(carriageSpeed));
                                            },
                                            {&m_elevatorSubsystem})
                                            .ToPtr());

  overrideCarriageTrigger.OnTrue(
      frc2::InstantCommand([this]() { m_elevatorSubsystem.SetCarriageMotorManualOverride(true); }, {}).ToPtr());

  // SHOOTER TRIGGER ACTIVATION
  shootManual.OnTrue(
      frc2::InstantCommand([this]() { m_ShooterSubSystem.ShooterGoToSpeed(5000_rpm); }, {&m_ShooterSubSystem}).ToPtr());
  feedForward.OnTrue(
      frc2::InstantCommand([this]() { m_ShooterSubSystem.Feed(0.5, true); }, {&m_ShooterSubSystem}).ToPtr());
  feedBackward.OnTrue(frc2::InstantCommand([this]() { m_ShooterSubSystem.Feed(-0.5); }, {&m_ShooterSubSystem}).ToPtr());
  shootManual.OnFalse(frc2::InstantCommand([this]() { m_ShooterSubSystem.Shoot(0.0); }, {&m_ShooterSubSystem}).ToPtr());
  (feedForward || feedBackward)
      .OnFalse(frc2::InstantCommand([this]() { m_ShooterSubSystem.Feed(0.0); }, {&m_ShooterSubSystem}).ToPtr());

  //
  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileTrue(argos_lib::SwapControllersCommand(&m_controllers).ToPtr());

  closedLoopSet.OnTrue(frc2::InstantCommand(
                           [this]() {
                             m_ShooterSubSystem.ShooterGoToSpeed(units::revolutions_per_minute_t(
                                 frc::SmartDashboard::GetNumber("shooter/Speed (rpm)", 3000)));
                             m_elevatorSubsystem.ElevatorMoveToHeight(
                                 units::inch_t(frc::SmartDashboard::GetNumber("elevator/Height (in)", 5.0)));
                             m_elevatorSubsystem.SetCarriageAngle(
                                 units::degree_t(frc::SmartDashboard::GetNumber("elevator/Angle (deg)", 0.0)));
                           },
                           {&m_ShooterSubSystem, &m_elevatorSubsystem})
                           .ToPtr());
}

void RobotContainer::Disable() {
  m_ledSubSystem.Disable();
  m_swerveDrive.Disable();
  m_intakeSubsystem.Disable();
  m_elevatorSubsystem.Disable();
  m_climberSubsystem.Disable();
  m_ShooterSubSystem.Disable();
}

void RobotContainer::Enable() {
  m_ledSubSystem.Enable();
}

void RobotContainer::AllianceChanged() {
  // If disabled, set alliance colors
  m_ledSubSystem.SetAllGroupsAllianceColor(true, true);
  m_ledSubSystem.SetDisableAnimation([this]() { m_ledSubSystem.SetAllGroupsAllianceColor(false, false); });
}

void RobotContainer::SetLedsConnectedBrightness(bool connected) {
  m_ledSubSystem.SetLedsConnectedBrightness(connected);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoSelector.GetSelectedCommand();
}
