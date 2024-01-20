/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/elevator_subsystem.h"

#include "argos_lib/config/talonsrx_config.h"
#include "constants/addresses.h"
#include "constants/motors.h"

ElevatorSubsystem::ElevatorSubsystem(argos_lib::RobotInstance robotInstance)
    : m_primaryMotor(GetCANAddr(
          address::comp_bot::intake::primaryIntake, address::practice_bot::intake::primaryIntake, robotInstance))
    , m_secondaryMotor(GetCANAddr(
          address::comp_bot::intake::secondaryIntake, address::practice_bot::intake::secondaryIntake, robotInstance))
    , m_carriageMotor(GetCANAddr(
          address::comp_bot::intake::primaryIntake, address::practice_bot::intake::primaryIntake, robotInstance))
    , m_robotInstance(robotInstance) {}
