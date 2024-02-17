/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>

#include "argos_lib/general/interpolation.h"

namespace controllerMap {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array driveSpeed{InterpMapPoint{-1.0, -1.0},
                                                   //    InterpMapPoint{-0.75, -0.4},
                                                   InterpMapPoint{-0.15, 0.0},
                                                   InterpMapPoint{0.15, 0.0},
                                                   //    InterpMapPoint{0.75, 0.4},
                                                   InterpMapPoint{1.0, 1.0}};
  [[maybe_unused]] constexpr std::array driveRotSpeed{
      InterpMapPoint{-1.0, -1.0}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 1.0}};
  [[maybe_unused]] constexpr std::array elevatorSpeed{
      InterpMapPoint{-1.0, -0.5}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.5}};
  [[maybe_unused]] constexpr std::array elevatorRotateSpeed{
      InterpMapPoint{-1.0, -0.2}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.2}};
  [[maybe_unused]] constexpr std::array climberSpeed{
      InterpMapPoint{-1.0, -0.2}, InterpMapPoint{-0.15, 0.0}, InterpMapPoint{0.15, 0.0}, InterpMapPoint{1.0, 0.2}};
}  // namespace controllerMap

namespace shooterRange {
  using argos_lib::InterpMapPoint;

  [[maybe_unused]] constexpr std::array shooterSpeed{InterpMapPoint{36.0_in, 5000_rpm},
                                                     InterpMapPoint{60.0_in, 5000_rpm},
                                                     InterpMapPoint{84.0_in, 5000_rpm},
                                                     InterpMapPoint{120.0_in, 5000_rpm},
                                                     InterpMapPoint{180.0_in, 5000_rpm},
                                                     InterpMapPoint{228_in, 5000_rpm}};

  [[maybe_unused]] constexpr std::array shooterAngle{InterpMapPoint{36.0_in, 60.0_deg},
                                                     InterpMapPoint{60.0_in, 49.0_deg},
                                                     InterpMapPoint{84.0_in, 43.0_deg},
                                                     InterpMapPoint{120.0_in, 32.5_deg},
                                                     InterpMapPoint{180.0_in, 25.5_deg},
                                                     InterpMapPoint{228_in, 24.7_deg}};
}  // namespace shooterRange
