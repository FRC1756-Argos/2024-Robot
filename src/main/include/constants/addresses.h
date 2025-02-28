/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/I2C.h>

#include "argos_lib/config/config_types.h"

namespace address {
  namespace comp_bot {
    struct drive {
      constexpr static argos_lib::CANAddress frontLeftDrive{1, "drive"};
      constexpr static argos_lib::CANAddress frontLeftTurn{2, "drive"};
      constexpr static argos_lib::CANAddress frontRightDrive{3, "drive"};
      constexpr static argos_lib::CANAddress frontRightTurn{4, "drive"};
      constexpr static argos_lib::CANAddress backRightDrive{5, "drive"};
      constexpr static argos_lib::CANAddress backRightTurn{6, "drive"};
      constexpr static argos_lib::CANAddress backLeftDrive{7, "drive"};
      constexpr static argos_lib::CANAddress backLeftTurn{8, "drive"};
    };

    struct intake {
      constexpr static argos_lib::CANAddress primaryIntake{9, "rio"};
      constexpr static argos_lib::CANAddress secondaryIntake{10, "rio"};
    };

    struct shooter {
      constexpr static argos_lib::CANAddress primaryMotor{11, "rio"};
      constexpr static argos_lib::CANAddress secondaryMotor{12, "rio"};
      constexpr static argos_lib::CANAddress feedMotor{13, "rio"};
    };

    struct climber {
      constexpr static argos_lib::CANAddress primaryClimbing{17, "climb"};
      constexpr static argos_lib::CANAddress secondaryClimbing{18, "climb"};
    };

    struct elevator {
      constexpr static argos_lib::CANAddress primaryElevator{14, "rio"};
      constexpr static argos_lib::CANAddress secondaryElevator{15, "rio"};
      constexpr static argos_lib::CANAddress carriageRotation{16, "rio"};
    };

    struct encoders {
      constexpr static argos_lib::CANAddress frontLeftEncoder{1, "drive"};
      constexpr static argos_lib::CANAddress frontRightEncoder{2, "drive"};
      constexpr static argos_lib::CANAddress backRightEncoder{3, "drive"};
      constexpr static argos_lib::CANAddress backLeftEncoder{4, "drive"};
      constexpr static argos_lib::CANAddress shooterEncoder{5, "rio"};
    };

    struct controllers {
      constexpr static const char driver = 0;
      constexpr static const char secondary = 1;
    };

    struct solenoids {};

    struct sensors {
      constexpr static argos_lib::CANAddress pigeonIMU{1, "drive"};
    };

    struct led {
      constexpr static argos_lib::CANAddress CANdle{1, "drive"};
    };
  }  // namespace comp_bot
  namespace practice_bot {
    using drive = address::comp_bot::drive;
    using intake = address::comp_bot::intake;
    using shooter = address::comp_bot::shooter;
    using climber = address::comp_bot::climber;
    using elevator = address::comp_bot::elevator;
    using encoders = address::comp_bot::encoders;
    using controllers = address::comp_bot::controllers;
    using solenoids = address::comp_bot::solenoids;
    using sensors = address::comp_bot::sensors;
    using led = address::comp_bot::led;
  }  // namespace practice_bot

}  // namespace address
