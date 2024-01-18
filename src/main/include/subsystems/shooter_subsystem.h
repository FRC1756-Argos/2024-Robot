// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <argos_lib/config/falcon_config.h>
#include <argos_lib/general/angle_utils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CorePigeon2.hpp"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include <string>

#include "constants/addresses.h"
#include "constants/measure_up.h"
#include "constants/motors.h"
#include "utils/sensor_conversions.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  explicit ShooterSubsystem(const argos_lib::RobotInstance instance);


  void Periodic() override;

 private:

 ctre::phoenix6::motorcontrol::can::TalonFX m_primaryMotor;
 ctre::phoenix6::motorcontrol::can::TalonFX m_secondaryMotor;
 ctre::phoenix::motorcontrol::can::TalonSRX m_feedMotor;
 argos_lib::RobotInstance m_instance;
};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

