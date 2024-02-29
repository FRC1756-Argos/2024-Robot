/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <ctre/phoenix6/CANcoder.hpp>

namespace encoder_conf {
  namespace comp_bot {
    namespace drive {
      struct genericTurn {
        constexpr static auto range = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
      };
    }  // namespace drive
    namespace elevator {
      struct elevatorEncoderConf {
        constexpr static auto range = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
        constexpr static auto direction = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive; ///@todo determine correct direction, range
        constexpr static auto magnetOffset = 0;
      };
    }  // namespace elevator
  }    // namespace comp_bot

  namespace practice_bot {
    namespace drive {
      using genericTurn = encoder_conf::comp_bot::drive::genericTurn;
    }  // namespace drive
    namespace elevator {
      struct elevatorEncoderConf {};
    }
  }  // namespace practice_bot
}  // namespace encoder_conf
