/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/elevator_subsystem.h"

#include "argos_lib/config/falcon_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

ElevatorSubsystem::ElevatorSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::elevator::primaryElevator, address::practice_bot::elevator::primaryElevator, robotInstance))
    , m_carriageMotor(GetCANAddr(
          address::comp_bot::elevator::primaryElevator, address::practice_bot::elevator::primaryElevator, robotInstance))
    , m_robotInstance(robotInstance) {
      argos_lib::falcon_config::FalconConfig<motorConfig::comp_bot::elevator::primaryElevator,
                                             motorConfig::practice_bot::elevator::primaryElevator>(
            m_primaryMotor, 100_ms, robotInstance);
      argos_lib::falcon_config::FalconConfig<motorConfig::practice_bot::elevator::carriageRotation,
                                             motorConfig::comp_bot::elevator::carriageRotation>(
            m_carriageMotor, 100_ms, robotInstance);
    }
      // This method will be called once per scheduler run
      void ElevatorSubsystem::Periodic() {}
