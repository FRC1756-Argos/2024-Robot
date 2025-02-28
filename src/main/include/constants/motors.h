/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "addresses.h"
#include "argos_lib/config/status_frame_config.h"
#include "control_loops.h"
#include "units/current.h"
#include "units/time.h"
#include "units/voltage.h"
#include "utils/sensor_conversions.h"

namespace motorConfig {
  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configuration settings shared by all robot configurations
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace common {
    constexpr static auto neutralDeadband = 0.001;
  }  // namespace common

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to competition robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace comp_bot {
    namespace drive {
      struct genericDrive {
        constexpr static auto inverted = true;
        constexpr static auto statorCurrentLimit = 80_A;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::drive::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::drive::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::drive::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::drive::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::drive::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::drive::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::drive::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::drive::gravityType;
      };

      struct frontLeftTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::frontLeftEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
      struct frontRightTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::frontRightEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
      struct backRightTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::backRightEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
      struct backLeftTurn {
        constexpr static auto inverted = false;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::backLeftEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = sensor_conversions::swerve_drive::turn::turnGearRatio;
        constexpr static auto sensorToMechanismRatio = sensor_conversions::swerve_drive::turn::sensorConversionFactor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::drive::rotate::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::drive::rotate::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::drive::rotate::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::drive::rotate::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::drive::rotate::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::drive::rotate::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::drive::rotate::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::drive::rotate::gravityType;
      };
    }  // namespace drive
    namespace shooter {
      struct primaryMotor {
        constexpr static auto inverted = true;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::shooter::shoot::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::shooter::shoot::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::shooter::shoot::kD;
        constexpr static auto pid0_kV = controlLoop::comp_bot::shooter::shoot::kV;
      };
      struct secondaryMotor {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
      };
      struct feedMotor {
        constexpr static auto inverted = false;
        constexpr static auto continuousCurrentLimit = 20_A;
        constexpr static auto voltCompSat = 11_V;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto forwardLimitSwitchSource =
            ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
        constexpr static auto forwardLimitSwitchNormal =
            ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
      };
    }  // namespace shooter

    namespace intake {
      struct primaryIntake {
        constexpr static auto inverted = false;
        constexpr static auto continuousCurrentLimit = 30_A;
        constexpr static auto voltCompSat = 11_V;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Brake;
        constexpr static auto forwardLimitSwitchSource =
            ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
        constexpr static auto forwardLimitSwitchNormal =
            ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
      };
      struct secondaryIntake {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      };
    }  // namespace intake
    namespace climber {
      struct primaryClimbing {
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto inverted = true;
        constexpr static auto statorCurrentLimit = 30_A;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::climber::climber::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::climber::climber::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::climber::climber::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::climber::climber::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::climber::climber::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::climber::climber::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::climber::climber::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::climber::climber::gravityType;
      };
      struct secondaryClimbing {
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto inverted = false;
      };
    }  // namespace climber
    namespace elevator {
      struct primaryElevator {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 40_A;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
        constexpr static auto pid0_kP = controlLoop::comp_bot::elevator::lift::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::elevator::lift::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::elevator::lift::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::elevator::lift::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::elevator::lift::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::elevator::lift::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::elevator::lift::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::elevator::lift::gravityType;
      };
      struct secondaryElevator {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      };
      struct carriageRotation {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statorCurrentLimit = 30_A;
        constexpr static auto selectedSensor_addr = address::comp_bot::encoders::shooterEncoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        constexpr static auto rotorToSensorRatio = 1 / sensor_conversions::elevator::carriage::sensorConversionFactor;
        constexpr static auto sensorToMechanismRatio = 1.0;
        constexpr static auto pid0_kP = controlLoop::comp_bot::elevator::carriage::kP;
        constexpr static auto pid0_kI = controlLoop::comp_bot::elevator::carriage::kI;
        constexpr static auto pid0_kD = controlLoop::comp_bot::elevator::carriage::kD;
        constexpr static auto pid0_kS = controlLoop::comp_bot::elevator::carriage::kS;
        constexpr static auto pid0_kV = controlLoop::comp_bot::elevator::carriage::kV;
        constexpr static auto pid0_kA = controlLoop::comp_bot::elevator::carriage::kA;
        constexpr static auto pid0_kG = controlLoop::comp_bot::elevator::carriage::kG;
        constexpr static auto pid0_gravityType = controlLoop::comp_bot::elevator::carriage::gravityType;
      };
    }  // namespace elevator
  }    // namespace comp_bot

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Motor configurations specific to practice robot
  //////////////////////////////////////////////////////////////////////////////////////////////////
  namespace practice_bot {
    namespace drive {
      using genericDrive = motorConfig::comp_bot::drive::genericDrive;

      using frontLeftTurn = motorConfig::comp_bot::drive::frontLeftTurn;
      using frontRightTurn = motorConfig::comp_bot::drive::frontRightTurn;
      using backRightTurn = motorConfig::comp_bot::drive::backRightTurn;
      using backLeftTurn = motorConfig::comp_bot::drive::backLeftTurn;
    }  // namespace drive
    namespace shooter {
      using primaryMotor = motorConfig::comp_bot::shooter::primaryMotor;
      using secondaryMotor = motorConfig::comp_bot::shooter::secondaryMotor;
      using feedMotor = motorConfig::comp_bot::shooter::feedMotor;
    }  // namespace shooter
    namespace intake {
      using primaryIntake = motorConfig::comp_bot::intake::primaryIntake;
      using secondaryIntake = motorConfig::comp_bot::intake::secondaryIntake;
    }  // namespace intake
    namespace climber {
      using primaryClimbing = motorConfig::comp_bot::climber::primaryClimbing;
      using secondaryClimbing = motorConfig::comp_bot::climber::secondaryClimbing;
    }  // namespace climber
    namespace elevator {
      using primaryElevator = motorConfig::comp_bot::elevator::primaryElevator;
      using secondaryElevator = motorConfig::comp_bot::elevator::secondaryElevator;
      using carriageRotation = motorConfig::comp_bot::elevator::carriageRotation;
    }  // namespace elevator
  }    // namespace practice_bot
}  // namespace motorConfig
