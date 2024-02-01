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
        constexpr static bool sensorPhase = false;
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
        constexpr static auto inverted = true;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
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
        constexpr static auto inverted = true;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontRightEncoder;
        constexpr static auto remoteFilter0_type = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
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
        constexpr static auto inverted = true;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::backRightEncoder;
        constexpr static auto remoteFilter0_type = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
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
        constexpr static auto inverted = true;
        constexpr static bool sensorPhase = true;
        constexpr static auto neutralDeadband = motorConfig::common::neutralDeadband;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        constexpr static auto statusFrameMotorMode = argos_lib::status_frame_config::MotorPresetMode::LeaderFX;
        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::backLeftEncoder;
        constexpr static auto remoteFilter0_type = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
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
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
      };
      struct secondaryMotor {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
      };
      struct feedMotor {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      };
    }  // namespace shooter

    namespace intake {
      struct primaryIntake {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      };
      struct secondaryIntake {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix::motorcontrol::NeutralMode::Coast;
      };
    }  // namespace intake
    namespace climber {
      struct primaryClimbing {
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        constexpr static auto inverted = false;
      };
      struct secondaryClimbing {
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        constexpr static auto inverted = false;
      };
    }  // namespace climber
    namespace elevator {
      struct primaryElevator {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      };
      struct secondaryElevator {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      };
      struct carriageRotation {
        constexpr static auto inverted = false;
        constexpr static auto neutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

        constexpr static auto remoteFilter0_addr = address::comp_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        constexpr static auto selectedSensor = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
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

      struct frontLeftTurn {
        constexpr static auto inverted = motorConfig::practice_bot::drive::frontLeftTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::practice_bot::drive::frontLeftTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::practice_bot::drive::frontLeftTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::practice_bot::drive::frontLeftTurn::neutralMode;
        constexpr static auto statusFrameMotorMode = motorConfig::practice_bot::drive::frontLeftTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::frontLeftEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::practice_bot::drive::frontLeftTurn::remoteFilter0_type;
        constexpr static auto selectedSensor = motorConfig::practice_bot::drive::frontLeftTurn::selectedSensor;
        constexpr static auto pid0_kP = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kD;
        constexpr static auto pid0_kS = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kS;
        constexpr static auto pid0_kV = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kV;
        constexpr static auto pid0_kA = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kA;
        constexpr static auto pid0_kG = motorConfig::practice_bot::drive::frontLeftTurn::pid0_kG;
        constexpr static auto pid0_gravityType = motorConfig::practice_bot::drive::frontLeftTurn::pid0_gravityType;
      };
      struct frontRightTurn {
        constexpr static auto inverted = motorConfig::practice_bot::drive::frontRightTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::practice_bot::drive::frontRightTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::practice_bot::drive::frontRightTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::practice_bot::drive::frontRightTurn::neutralMode;
        constexpr static auto statusFrameMotorMode = motorConfig::practice_bot::drive::frontRightTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::frontRightEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::practice_bot::drive::frontRightTurn::remoteFilter0_type;
        constexpr static auto selectedSensor = motorConfig::practice_bot::drive::frontRightTurn::selectedSensor;
        constexpr static auto pid0_kP = motorConfig::practice_bot::drive::frontRightTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::practice_bot::drive::frontRightTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::practice_bot::drive::frontRightTurn::pid0_kD;
        constexpr static auto pid0_kS = motorConfig::practice_bot::drive::frontRightTurn::pid0_kS;
        constexpr static auto pid0_kV = motorConfig::practice_bot::drive::frontRightTurn::pid0_kV;
        constexpr static auto pid0_kA = motorConfig::practice_bot::drive::frontRightTurn::pid0_kA;
        constexpr static auto pid0_kG = motorConfig::practice_bot::drive::frontRightTurn::pid0_kG;
        constexpr static auto pid0_gravityType = motorConfig::practice_bot::drive::frontRightTurn::pid0_gravityType;
      };
      struct backRightTurn {
        constexpr static auto inverted = motorConfig::practice_bot::drive::backRightTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::practice_bot::drive::backRightTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::practice_bot::drive::backRightTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::practice_bot::drive::backRightTurn::neutralMode;
        constexpr static auto statusFrameMotorMode = motorConfig::practice_bot::drive::backRightTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::backRightEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::practice_bot::drive::backRightTurn::remoteFilter0_type;
        constexpr static auto selectedSensor = motorConfig::practice_bot::drive::backRightTurn::selectedSensor;
        constexpr static auto pid0_kP = motorConfig::practice_bot::drive::backRightTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::practice_bot::drive::backRightTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::practice_bot::drive::backRightTurn::pid0_kD;
        constexpr static auto pid0_kS = motorConfig::practice_bot::drive::backRightTurn::pid0_kS;
        constexpr static auto pid0_kV = motorConfig::practice_bot::drive::backRightTurn::pid0_kV;
        constexpr static auto pid0_kA = motorConfig::practice_bot::drive::backRightTurn::pid0_kA;
        constexpr static auto pid0_kG = motorConfig::practice_bot::drive::backRightTurn::pid0_kG;
        constexpr static auto pid0_gravityType = motorConfig::practice_bot::drive::backRightTurn::pid0_gravityType;
      };
      struct backLeftTurn {
        constexpr static auto inverted = motorConfig::practice_bot::drive::backLeftTurn::inverted;
        constexpr static bool sensorPhase = motorConfig::practice_bot::drive::backLeftTurn::sensorPhase;
        constexpr static auto neutralDeadband = motorConfig::practice_bot::drive::backLeftTurn::neutralDeadband;
        constexpr static auto neutralMode = motorConfig::practice_bot::drive::backLeftTurn::neutralMode;
        constexpr static auto statusFrameMotorMode = motorConfig::practice_bot::drive::backLeftTurn::statusFrameMotorMode;
        constexpr static auto remoteFilter0_addr = address::practice_bot::encoders::backLeftEncoder;
        constexpr static auto remoteFilter0_type = motorConfig::practice_bot::drive::backLeftTurn::remoteFilter0_type;
        constexpr static auto selectedSensor = motorConfig::practice_bot::drive::backLeftTurn::selectedSensor;
        constexpr static auto pid0_kP = motorConfig::practice_bot::drive::backLeftTurn::pid0_kP;
        constexpr static auto pid0_kI = motorConfig::practice_bot::drive::backLeftTurn::pid0_kI;
        constexpr static auto pid0_kD = motorConfig::practice_bot::drive::backLeftTurn::pid0_kD;
        constexpr static auto pid0_kS = motorConfig::practice_bot::drive::backLeftTurn::pid0_kS;
        constexpr static auto pid0_kV = motorConfig::practice_bot::drive::backLeftTurn::pid0_kV;
        constexpr static auto pid0_kA = motorConfig::practice_bot::drive::backLeftTurn::pid0_kA;
        constexpr static auto pid0_kG = motorConfig::practice_bot::drive::backLeftTurn::pid0_kG;
        constexpr static auto pid0_gravityType = motorConfig::practice_bot::drive::backLeftTurn::pid0_gravityType;
      };

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
