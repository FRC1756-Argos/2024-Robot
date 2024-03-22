/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/angle_utils.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>

namespace measure_up {
  constexpr auto bumperExtension = 3_in;  ///< Distance from frame to outer edge of bumpers
  namespace chassis {
    constexpr units::inch_t width{29.0};
    constexpr units::inch_t length{30.0};
  }  // namespace chassis
  namespace swerve_offsets {
    constexpr auto frontLeftLOffset = 3.25_in;
    constexpr auto frontLeftWOffset = 3.25_in;
    constexpr auto frontRightLOffset = 3.25_in;
    constexpr auto frontRightWOffset = 3.25_in;
    constexpr auto backRightWOffset = 3.25_in;
    constexpr auto backRightLOffset = 3.25_in;
    constexpr auto backLeftWOffset = 3.25_in;
    constexpr auto backLeftLOffset = 3.25_in;
  }  // namespace swerve_offsets
  namespace shooter_targets {
    constexpr auto speakerTagHeight = 58_in;  // needs update
    constexpr auto speakerOpeningHeightFromShooter = 54.5_in;
    constexpr auto trapOpeningHeight = 36.25_in;
    constexpr auto cameraOffsetFromShooter = 21.0_in;
    constexpr auto offsetDistanceThreshold = 140_in;
    constexpr auto offsetDistThresholdSecondaryCam = 210_in;
    constexpr auto offsetRotationThreshold = 50_deg;
    constexpr auto speakerOpeningHeightFromGround = 80.5_in;
    constexpr auto cameraHeightToAprilTag = 28.63_in;
    constexpr auto secondaryCameraToShooter = 18_in;
    constexpr double frontSideSpinFactor = 10.0;
    constexpr double longShotSpinFactor = 1.7;
  }  // namespace shooter_targets
  namespace camera_front {
    constexpr auto cameraX = 0_in;  /// @todo real mounting offsets
    constexpr auto cameraZ = 7.25_in;
    constexpr auto cameraMountAngle = 24.9_deg;
    constexpr auto cameraHeight = 28.5_in;
    constexpr auto vFov = 24.85_deg * 2;
    constexpr auto hFov = 29.8_deg * 2;
  }                         // namespace camera_front
  namespace camera_back {}  // namespace camera_back

  namespace climber {
    constexpr auto lowerLimit = 3.375_in;  // from floor to bottom of linear rail carrige
    constexpr auto upperLimit = 24.5_in;   // approx 19.5in of travel, get better value later
    constexpr auto climbRaisedHeight = 24_in;
    constexpr auto climbLoweredHeight = 4_in;
    constexpr auto climberStagingHeight = 15.5_in;
  }  // namespace climber

  namespace elevator {
    namespace lift {
      constexpr auto minHeight = 21.5_in;
      constexpr auto maxHeight = 41.5_in;
      constexpr auto intakeHeight = 21.5_in;
      constexpr auto ampHeight = 40_in;
      constexpr auto podiumLowHeight = intakeHeight;
      constexpr auto podiumHighHeight = 40_in;
      constexpr auto subwooferHeight = intakeHeight;
      constexpr auto trapHeight = 35_in;
    }  // namespace lift
    namespace carriage {
      constexpr auto minAngle = -110_deg;
      constexpr auto maxAngle = 270_deg;
      constexpr auto intakeAngle = 40_deg;
      constexpr auto ampAngle = 135_deg;
      constexpr auto podiumHighAngle = 23.5_deg;
      constexpr auto podiumLowAngle = 36_deg;
      constexpr auto subwooferAngle = 135_deg;
      constexpr auto trapAngle = -100_deg;
      constexpr auto skipShotAngle = 5_deg;
    }  // namespace carriage
  }    // namespace elevator

  namespace shooter {
    constexpr auto minSpeed = 0_tps;
    constexpr auto maxSpeed = 100_tps;
  }  // namespace shooter
}  // namespace measure_up
