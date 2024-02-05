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
  namespace shooter_targets {
    constexpr auto speakerTagHeight = 58_in;  // needs update
    constexpr auto trapOpeningHeight = 36.25_in;
  }  // namespace shooter_targets
  namespace camera_front {
    constexpr auto cameraX = 0_in;  /// @todo real mounting offsets
    constexpr auto cameraZ = 7.25_in;
    constexpr auto cameraMountAngle = 29.9_deg;
    constexpr auto cameraHeight = 28.5_in;
    constexpr auto vFov = 24.85_deg * 2;
    constexpr auto hFov = 29.8_deg * 2;
  }                         // namespace camera_front
  namespace camera_back {}  // namespace camera_back

  namespace elevator {
    namespace lift {
      constexpr auto minHeight = 21.5_in;
      constexpr auto maxHeight = 42.5_in;
    }  // namespace lift
    namespace carriage {
      constexpr auto minAngle = -270_deg;
      constexpr auto maxAngle = 90_deg;
      constexpr auto intakeAngle = 40_deg;
    }  // namespace carriage
  }    // namespace elevator
}  // namespace measure_up
