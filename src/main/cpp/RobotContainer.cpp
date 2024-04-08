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
#include <constants/feature_flags.h>
#include <Constants.h>

#include <cmath>
#include <memory>
#include <optional>

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
    , m_IntakeCommand{&m_intakeSubsystem, &m_ShooterSubSystem, &m_elevatorSubsystem, &m_controllers, &m_ledSubSystem}
    , m_ShooterCommand{&m_ShooterSubSystem}
    , m_autoAimCommand{&m_swerveDrive,
                       &m_ShooterSubSystem,
                       &m_elevatorSubsystem,
                       &m_visionSubSystem,
                       &m_controllers,
                       &m_ledSubSystem}
    , m_ClimberHomeCommand(m_climberSubsystem, m_instance)
    , m_GoToAmpPositionCommand{&m_ShooterSubSystem, &m_elevatorSubsystem}
    , m_GoToHighPodiumPositionCommand{&m_ShooterSubSystem, &m_elevatorSubsystem, true}
    , m_GoToLowPodiumPositionCommand{&m_ShooterSubSystem, &m_elevatorSubsystem, false}
    , m_GoToSubwooferPositionCommand{&m_ShooterSubSystem, &m_elevatorSubsystem}
    , m_GoToTrapPositionCommand{&m_ShooterSubSystem, &m_elevatorSubsystem}
    , m_ClimberCommand{&m_climberSubsystem, &m_ShooterSubSystem, &m_elevatorSubsystem, &m_controllers}
    , m_CrossfieldShotCommand(&m_ShooterSubSystem, &m_elevatorSubsystem)
    , m_autoNothing{m_swerveDrive}
    , m_autoChoreoTest{m_elevatorSubsystem, m_intakeSubsystem, m_ShooterSubSystem, m_swerveDrive, m_visionSubSystem}
    , m_autoCenter2wing{m_intakeSubsystem,
                        m_ShooterSubSystem,
                        m_elevatorSubsystem,
                        m_swerveDrive,
                        m_visionSubSystem,
                        m_controllers,
                        m_ledSubSystem}
    , m_autoSource1{m_intakeSubsystem,
                    m_ShooterSubSystem,
                    m_elevatorSubsystem,
                    m_swerveDrive,
                    m_visionSubSystem,
                    m_controllers,
                    m_ledSubSystem}
    , m_autoSourceSideSubwoofer2Piece{m_intakeSubsystem,
                                      m_ShooterSubSystem,
                                      m_elevatorSubsystem,
                                      m_swerveDrive,
                                      m_visionSubSystem,
                                      m_controllers,
                                      m_ledSubSystem}
    , m_autoSource2{m_intakeSubsystem,
                    m_ShooterSubSystem,
                    m_elevatorSubsystem,
                    m_swerveDrive,
                    m_visionSubSystem,
                    m_controllers,
                    m_ledSubSystem}
    , m_autoSourceSideSteal2{m_intakeSubsystem,
                             m_ShooterSubSystem,
                             m_elevatorSubsystem,
                             m_swerveDrive,
                             m_visionSubSystem,
                             m_controllers,
                             m_ledSubSystem}
    , m_autoAmpSideSubwoofer2Piece{m_intakeSubsystem,
                                   m_ShooterSubSystem,
                                   m_elevatorSubsystem,
                                   m_swerveDrive,
                                   m_visionSubSystem,
                                   m_controllers,
                                   m_ledSubSystem}
    , m_autoSourceSideSubwoofer3PieceSteal{m_intakeSubsystem,
                                           m_ShooterSubSystem,
                                           m_elevatorSubsystem,
                                           m_swerveDrive,
                                           m_visionSubSystem,
                                           m_controllers,
                                           m_ledSubSystem}
    , m_autoZeroNote{m_swerveDrive}
    , m_autoSourceSideSubwoofer4Piece{m_intakeSubsystem,
                                      m_ShooterSubSystem,
                                      m_elevatorSubsystem,
                                      m_swerveDrive,
                                      m_visionSubSystem,
                                      m_controllers,
                                      m_ledSubSystem}
    , m_autoCenterSubwoofer4Piece{m_intakeSubsystem,
                                  m_ShooterSubSystem,
                                  m_elevatorSubsystem,
                                  m_swerveDrive,
                                  m_visionSubSystem,
                                  m_controllers,
                                  m_ledSubSystem}
    , m_autoCenterSubwoofer6Piece{m_intakeSubsystem,
                                  m_ShooterSubSystem,
                                  m_elevatorSubsystem,
                                  m_swerveDrive,
                                  m_visionSubSystem,
                                  m_controllers,
                                  m_ledSubSystem}
    , m_autoSourceSideSubwoofer5Piece{m_intakeSubsystem,
                                      m_ShooterSubSystem,
                                      m_elevatorSubsystem,
                                      m_swerveDrive,
                                      m_visionSubSystem,
                                      m_controllers,
                                      m_ledSubSystem}
    , m_autoSelector{{&m_autoNothing,
                      &m_autoAmpSideSubwoofer2Piece,
                      &m_autoCenter2wing,
                      &m_autoCenterSubwoofer4Piece,
                      &m_autoCenterSubwoofer6Piece,
                      &m_autoSourceSideSubwoofer2Piece,
                      &m_autoSourceSideSubwoofer4Piece,
                      &m_autoSourceSideSubwoofer5Piece,
                      &m_autoSourceSideSteal2,
                      &m_autoSource1,
                      &m_autoSource2,
                      &m_autoZeroNote,
                      // &m_autoSourceSideSubwoofer3PieceSteal,
                      &m_autoChoreoTest},
                     &m_autoNothing}
    , m_transitionedFromAuto{false} {
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

        auto rotateSpeed = deadbandRotSpeed;

        if constexpr (feature_flags::nt_debugging) {
          frc::SmartDashboard::PutBoolean("(DRIVER) IsAimingActive", m_visionSubSystem.IsAimWhileMoveActive());
        }

        if (m_visionSubSystem.IsAimWhileMoveActive() || m_visionSubSystem.IsOdometryAimingActive()) {
          auto speed = m_visionSubSystem.getShooterSpeed();
          auto rotationWithInertia =
              m_visionSubSystem.getRotationSpeedWithInertia(deadbandTranslationSpeeds.leftSpeedPct);
          auto shooterAngleWithInertia =
              m_visionSubSystem.getShooterAngleWithInertia(deadbandTranslationSpeeds.forwardSpeedPct);

          if (speed.has_value()) {
            m_ShooterSubSystem.ShooterGoToSpeed(speed.value());
          } else {
            m_ShooterSubSystem.ShooterGoToSpeed(
                m_visionSubSystem.getShooterSpeed(15_ft, VisionSubsystem::InterpolationMode::LinearInterpolation));
          }

          if (rotationWithInertia && shooterAngleWithInertia) {
            m_elevatorSubsystem.SetCarriageAngle(shooterAngleWithInertia.value());
            rotateSpeed = rotationWithInertia.value();

            // simmer down the translation speeds
            deadbandTranslationSpeeds.forwardSpeedPct *= speeds::drive::aimSpeedReductionPct;
            deadbandTranslationSpeeds.leftSpeedPct *= speeds::drive::aimSpeedReductionPct;
          } else {
            rotateSpeed = deadbandRotSpeed;
          }

          if (!speed || !rotationWithInertia || !shooterAngleWithInertia) {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyRed, false); },
                200_ms);
          } else if (m_elevatorSubsystem.IsCarriageAtSetPoint() && std::abs(rotationWithInertia.value()) <= 0.1) {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyGreen, false); },
                200_ms);
          } else {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsColor(argos_lib::gamma_corrected_colors::kCatYellow, false); },
                200_ms);
          }
        } else if (m_ShooterSubSystem.IsFeedingShotActive()) {
          auto rotationWithInertia = m_visionSubSystem.getFeedOffsetWithInertia(deadbandTranslationSpeeds.leftSpeedPct);
          auto feederAngleWithInertia =
              m_visionSubSystem.getFeederAngleWithInertia(deadbandTranslationSpeeds.forwardSpeedPct);

          if (rotationWithInertia && feederAngleWithInertia) {
            m_elevatorSubsystem.SetCarriageAngle(feederAngleWithInertia.value());
            rotateSpeed = rotationWithInertia.value();
          } else {
            rotateSpeed = deadbandRotSpeed;
          }

          if (!rotationWithInertia || !feederAngleWithInertia) {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyRed, false); },
                200_ms);
          } else if (m_elevatorSubsystem.IsCarriageAtSetPoint() && std::abs(rotationWithInertia.value()) <= 0.1) {
            m_ledSubSystem.TemporaryAnimate(
                [this]() { m_ledSubSystem.SetAllGroupsColor(argos_lib::gamma_corrected_colors::kNoteOrange, false); },
                200_ms);
          }
        }

        if (frc::DriverStation::IsTeleop() &&
            (m_swerveDrive.GetManualOverride() || deadbandTranslationSpeeds.forwardSpeedPct != 0 ||
             deadbandTranslationSpeeds.leftSpeedPct != 0 || rotateSpeed != 0)) {
          m_visionSubSystem.SetEnableStaticRotation(false);
          m_swerveDrive.SwerveDrive(
              deadbandTranslationSpeeds.forwardSpeedPct,
              deadbandTranslationSpeeds.leftSpeedPct,
              rotateSpeed);  // X axis is positive right (CW), but swerve coordinates are positive left (CCW)
        }
        // DEBUG STUFF
        if constexpr (feature_flags::nt_debugging) {
          frc::SmartDashboard::PutBoolean("(DRIVER) Static Enable", m_visionSubSystem.IsStaticRotationEnabled());
          frc::SmartDashboard::PutNumber(
              "(DRIVER) Joystick Left Y",
              m_controllers.DriverController().GetY(argos_lib::XboxController::JoystickHand::kLeftHand));
          frc::SmartDashboard::PutNumber(
              "(DRIVER) Joystick Left X",
              m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kLeftHand));
          frc::SmartDashboard::PutNumber(
              "(DRIVER) Joystick Right X",
              m_controllers.DriverController().GetX(argos_lib::XboxController::JoystickHand::kRightHand));
        }
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

  // Climber homing trigger

  auto ClimberHomeRequiredTrigger = (frc2::Trigger{[this]() { return !m_climberSubsystem.IsClimberHomed(); }});

  auto startupClimberHomeTrigger = robotEnableTrigger && ClimberHomeRequiredTrigger;

  // DRIVE TRIGGERS
  auto fieldHome = m_controllers.DriverController().TriggerDebounced(argos_lib::XboxController::Button::kY);

  // INTAKE TRIGGERS
  auto intake = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  auto outtakeManual = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kBumperLeft);
  auto crossfieldShot = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger);

  // CLIMBER TRIGGERS
  auto climberUp = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kUp);
  auto climberDown = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kDown);
  auto climberSequenceTrigger =
      m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBumperRight);
  // auto reverseClimbTrigger = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);

  // SHOOT TRIGGERS
  auto shoot = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kRightTrigger);
  auto feedForward = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kUp);
  auto feedBackward = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kDown);
  auto aim = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kLeftTrigger);
  auto aimWhileMove = m_controllers.DriverController().TriggerRaw(argos_lib::XboxController::Button::kA);
  auto odometryAim = m_controllers.DriverController().TriggerRaw(
      argos_lib::XboxController::Button::kB);  // for debugging only, will be removed

  auto ampPositionTrigger = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kA);
  auto highPodiumPositionTrigger = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kX);
  auto lowPodiumPositionTrigger = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kB);
  auto subwooferPositionTrigger = m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kY);

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

  auto fireTrigger = m_controllers.OperatorController().TriggerDebounced(argos_lib::XboxController::Button::kStart) &&
                     !m_controllers.OperatorController().TriggerRaw(argos_lib::XboxController::Button::kBack);

  // SWAP CONTROLLER TRIGGERS
  frc2::Trigger driverTriggerSwapCombo = m_controllers.DriverController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});
  frc2::Trigger operatorTriggerSwapCombo = m_controllers.OperatorController().TriggerDebounced(
      {argos_lib::XboxController::Button::kBack, argos_lib::XboxController::Button::kStart});

  /* ————————————————————————— TRIGGER ACTIVATION ———————————————————————— */

  // Restart shooter on transition from auto to teleop
  robotEnableTrigger.OnTrue(frc2::InstantCommand([this]() {
                              if (frc::DriverStation::IsAutonomous()) {
                                m_transitionedFromAuto = true;
                              } else {
                                if (frc::DriverStation::IsTeleop()) {
                                  if (m_transitionedFromAuto) {
                                    m_ShooterSubSystem.ShooterGoToSpeed(m_visionSubSystem.getShooterSpeed(
                                        15_ft, VisionSubsystem::InterpolationMode::LinearInterpolation));
                                  }
                                }
                                m_transitionedFromAuto = false;
                              }
                            }).ToPtr());

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
  aimWhileMove
      .OnTrue(frc2::InstantCommand([this]() { m_visionSubSystem.SetAimWhileMove(true); }, {&m_visionSubSystem}).ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { m_visionSubSystem.SetAimWhileMove(false); }, {&m_visionSubSystem}).ToPtr());

  odometryAim
      .OnTrue(
          frc2::InstantCommand([this]() { m_visionSubSystem.SetOdometryAiming(true); }, {&m_visionSubSystem}).ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { m_visionSubSystem.SetOdometryAiming(false); }, {&m_visionSubSystem}).ToPtr());
  // CLIMBER TRIGGER ACTIVATION
  startupClimberHomeTrigger.OnTrue(&m_ClimberHomeCommand);
  (!ClimberHomeRequiredTrigger && robotEnableTrigger)
      .OnTrue(frc2::InstantCommand(
                  [this]() {
                    if (m_instance == argos_lib::RobotInstance::Competition) {
                      m_climberSubsystem.SetHeight(measure_up::climber::climberStagingHeight);
                    }
                  },
                  {&m_climberSubsystem})
                  .ToPtr());

  crossfieldShot.OnTrue(&m_CrossfieldShotCommand)
      .OnFalse(frc2::InstantCommand([this]() { m_ShooterSubSystem.SetFeedingShotActive(false); }, {&m_ShooterSubSystem})
                   .ToPtr());

  climberUp.OnTrue(frc2::InstantCommand(
                       [this]() {
                         m_climberSubsystem.SetClimberManualOverride(true);
                         m_climberSubsystem.ClimberMove(0.6);
                       },
                       {&m_climberSubsystem})
                       .ToPtr());
  climberDown.OnTrue(frc2::InstantCommand(
                         [this]() {
                           m_climberSubsystem.SetClimberManualOverride(true);
                           m_climberSubsystem.ClimberMove(-0.8);
                         },
                         {&m_climberSubsystem})
                         .ToPtr());
  (climberUp || climberDown)
      .OnFalse(frc2::InstantCommand([this]() { m_climberSubsystem.ClimberMove(0.0); }, {&m_climberSubsystem}).ToPtr());

  climberSequenceTrigger.OnTrue(&m_ClimberCommand);

  // ELEVATOR TRIGGER ACTIVATION
  elevatorLiftManualInput.OnTrue(
      frc2::InstantCommand([this]() { m_elevatorSubsystem.SetElevatorLiftManualOverride(true); }, {}).ToPtr());
  m_elevatorSubsystem.SetDefaultCommand(frc2::RunCommand(
                                            [this] {
                                              double elevatorSpeed = -m_controllers.OperatorController().GetY(
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
  feedForward.OnTrue(
      frc2::InstantCommand([this]() { m_ShooterSubSystem.Feed(0.5, true); }, {&m_ShooterSubSystem}).ToPtr());
  feedBackward.OnTrue(frc2::InstantCommand([this]() { m_ShooterSubSystem.Feed(-0.5); }, {&m_ShooterSubSystem}).ToPtr());
  (feedForward || feedBackward)
      .OnFalse(frc2::InstantCommand([this]() { m_ShooterSubSystem.Feed(0.0); }, {&m_ShooterSubSystem}).ToPtr());

  fireTrigger.OnTrue(frc2::InstantCommand(
                         [this]() {
                           m_ledSubSystem.FireEverywhere();
                           m_ledSubSystem.SetDisableAnimation([this]() { m_ledSubSystem.FireEverywhere(false); });
                         },
                         {&m_ledSubSystem})
                         .ToPtr());

  // SWAP CONTROLLERS TRIGGER ACTIVATION
  (driverTriggerSwapCombo || operatorTriggerSwapCombo)
      .WhileTrue(argos_lib::SwapControllersCommand(&m_controllers).ToPtr());

  // closedLoopSet.OnTrue(frc2::InstantCommand(
  //                          [this]() {
  //                            m_ShooterSubSystem.ShooterGoToSpeed(units::revolutions_per_minute_t(
  //                                frc::SmartDashboard::GetNumber("shooter/Speed (rpm)", 3000)));
  //                            m_elevatorSubsystem.ElevatorMoveToHeight(
  //                                units::inch_t(frc::SmartDashboard::GetNumber("elevator/Height (in)", 5.0)));
  //                            m_elevatorSubsystem.SetCarriageAngle(
  //                                units::degree_t(frc::SmartDashboard::GetNumber("elevator/Angle (deg)", 0.0)));
  //                          },
  //                          {&m_ShooterSubSystem, &m_elevatorSubsystem})
  //                          .ToPtr());
  ampPositionTrigger.OnTrue(&m_GoToAmpPositionCommand);
  highPodiumPositionTrigger.OnTrue(&m_GoToHighPodiumPositionCommand);
  lowPodiumPositionTrigger.OnTrue(&m_GoToLowPodiumPositionCommand);
  subwooferPositionTrigger.OnTrue(&m_GoToSubwooferPositionCommand);
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
