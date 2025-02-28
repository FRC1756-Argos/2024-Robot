/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <numbers>

#include "custom_units.h"

namespace sensor_conversions {
  namespace swerve_drive {
    namespace turn {
      constexpr double sensorConversionFactor = 1;  ///< multiply to convert raw sensor units to module degrees
      constexpr units::angle::turn_t ToSensorUnit(const units::degree_t degrees) {
        return degrees / sensorConversionFactor;
      }
      constexpr units::degree_t ToAngle(const units::angle::turn_t sensorunit) {
        return sensorunit * sensorConversionFactor;
      }
      constexpr double turnGearRatio = 12.8;
      constexpr units::turn_t MotorToWheelAngle(const units::angle::turn_t motorPosition) {
        return motorPosition / turnGearRatio;
      }
      constexpr units::turn_t WheelToMotorAngle(const units::angle::turn_t wheelPosition) {
        return wheelPosition * turnGearRatio;
      }
    }  // namespace turn
    namespace drive {
      constexpr auto wheelDiameter = 4.0_in / units::angle::turn_t{1};
      constexpr auto wheelCircumference = wheelDiameter * std::numbers::pi;
      constexpr auto driveGearRatio = 5.49;

      constexpr units::inch_t ToDistance(const units::angle::turn_t sensorunit) {
        return wheelCircumference * (sensorunit / driveGearRatio);
      }
      constexpr units::angle::turn_t ToSensorPosition(const units::inch_t distance) {
        return (distance / wheelCircumference) * driveGearRatio;
      }

      constexpr units::inches_per_second_t ToVelocity(
          const units::angular_velocity::revolutions_per_minute_t sensorVelocity) {
        return ToDistance(sensorVelocity * units::second_t{1}) / units::second_t{1};
      }
      constexpr units::angular_velocity::revolutions_per_minute_t ToSensorVelocity(
          const units::inches_per_second_t velocity) {
        return ToSensorPosition(velocity * units::second_t{1}) / units::second_t{1};
      }
    }  // namespace drive
  }    // namespace swerve_drive

  namespace climber {
    constexpr auto sensorConversionFactor = 4_in / 20_tr;
    // inches per sensor unit. 20:1 gear ratio, 4in/sprocket revolution -> 4in/20turns = 1inch/5turns

    constexpr units::inch_t ToHeight(units::angle::turn_t sensorunit) {  // 0 sensor units = minimum climber height
      return sensorunit * sensorConversionFactor;
    }
    constexpr units::angle::turn_t ToSensorUnit(units::inch_t distance) {
      return distance / sensorConversionFactor;
    }

  }  // namespace climber

  namespace elevator {
    namespace lift {
      constexpr auto sensorConversionFactor = 1_in / 3_tr;
      constexpr units::inch_t ToHeight(const units::angle::turn_t sensorUnit) {
        return sensorConversionFactor * sensorUnit;
      }

      constexpr units::angle::turn_t ToSensorUnit(const units::inch_t height) {
        return height / sensorConversionFactor;
      }
    }  // namespace lift
    namespace carriage {
      constexpr auto sensorConversionFactor = (1.0 / 9.0) * (15.0 / 72.0);
      constexpr units::angle::turn_t ToSensorUnit(const units::degree_t degrees) {
        return degrees / sensorConversionFactor;
      }
      constexpr units::degree_t ToAngle(const units::angle::turn_t sensorUnit) {
        return sensorUnit * sensorConversionFactor;
      }
    }  // namespace carriage
  }    // namespace elevator
}  // namespace sensor_conversions
