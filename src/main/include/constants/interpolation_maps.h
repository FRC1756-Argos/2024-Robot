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

  [[maybe_unused]] constexpr std::array shooterSpeed{InterpMapPoint{36.0_in, 5300_rpm},
                                                     InterpMapPoint{60.0_in, 5300_rpm},
                                                     InterpMapPoint{84.0_in, 5300_rpm},
                                                     InterpMapPoint{120.0_in, 5300_rpm},
                                                     InterpMapPoint{180.0_in, 5300_rpm},
                                                     InterpMapPoint{228_in, 5300_rpm}};

  [[maybe_unused]] constexpr std::array shooterAngle{InterpMapPoint{36.0_in, 60.0_deg},
                                                     InterpMapPoint{69.0_in, 48.0_deg},
                                                     InterpMapPoint{88.0_in, 43.0_deg},
                                                     InterpMapPoint{123.0_in, 33.0_deg},
                                                     InterpMapPoint{150.0_in, 29.4_deg},
                                                     InterpMapPoint{174.0_in, 26.2_deg},
                                                     InterpMapPoint{228_in, 25.0_deg}};

  [[maybe_unused]] constexpr std::array feederAngle{InterpMapPoint{10.0_in, 80.0_deg},
                                                    InterpMapPoint{50.0_in, 75.0_deg},
                                                    InterpMapPoint{100.0_in, 65.0_deg},
                                                    InterpMapPoint{150.0_in, 60.0_deg},
                                                    InterpMapPoint{200.0_in, 55.0_deg},
                                                    InterpMapPoint{250.0_in, 47.5_deg},
                                                    InterpMapPoint{300.0_in, 40.0_deg},
                                                    InterpMapPoint{350.0_in, 37.5_deg}};
}  // namespace shooterRange
