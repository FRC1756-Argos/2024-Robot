/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

namespace controlLoop {
  namespace comp_bot {
    namespace drive {
      struct rotate {
        constexpr static double kP = 137;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };  // namespace rotate
      struct drive {
        constexpr static double kP = 0.25;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.12;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.0;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };  // namespace drive
      struct linear_follower {
        constexpr static double kP = 0.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
      };  // namespace linear_follower
      struct rotational_follower {
        constexpr static double kP = 0.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static auto angularVelocity = units::degrees_per_second_t{360};
        constexpr static auto angularAcceleration = units::degrees_per_second_squared_t{360};
      };  // namespace rotational_follower
    }     // namespace drive
    namespace climber {
      struct climber {
        constexpr static double kP = 1.0;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.24;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };
    }  // namespace climber
    namespace elevator {
      struct lift {
        constexpr static double kP = 2.1;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.04;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = 0.4;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      };
      struct carriage {
        constexpr static double kP = 75;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kS = 0.0;
        constexpr static double kV = 0.0;
        constexpr static double kA = 0.0;
        constexpr static double kG = -0.45;
        constexpr static int gravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
      };
    }  // namespace elevator
    namespace shooter {
      struct shoot {
        constexpr static double kP = 0.5;
        constexpr static double kI = 0.0;
        constexpr static double kD = 0.0;
        constexpr static double kV = 0.12;
      };
    }  // namespace shooter
  }    // namespace comp_bot
  namespace practice_bot {
    namespace drive {
      using rotate = controlLoop::comp_bot::drive::rotate;
      using drive = controlLoop::comp_bot::drive::drive;
      using linear_follower = controlLoop::comp_bot::drive::linear_follower;
      using rotational_follower = controlLoop::comp_bot::drive::rotational_follower;
    }  // namespace drive
    namespace climber {
      using climber = controlLoop::comp_bot::climber::climber;
    }  // namespace climber
    namespace elevator {
      using lift = controlLoop::comp_bot::elevator::lift;
      using carriage = controlLoop::comp_bot::elevator::carriage;
    }  // namespace elevator
    namespace shooter {
      using shoot = controlLoop::comp_bot::shooter::shoot;
    }  // namespace shooter
  }    // namespace practice_bot
}  // namespace controlLoop
