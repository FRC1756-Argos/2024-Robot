/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/angle_utils.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
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
  namespace camera_front {
    constexpr auto cameraX = 0_in;  /// @todo real mounting offsets
    constexpr auto cameraZ = 7.25_in;
    constexpr auto cameraMountAngle = 13.5_deg;
    constexpr auto vFov = 24.85_deg * 2;
    constexpr auto hFov = 29.8_deg * 2;
  }                         // namespace camera_front
  namespace camera_back {}  // namespace camera_back
  namespace climber {
    constexpr auto lowerLimit = 3.375_in;   // from floor to bottom of linear rail carrige
    constexpr auto upperLimit = 22.875_in;  //approx 19.5in of travel, get better value later
  }                                         // namespace climber
}  // namespace measure_up
