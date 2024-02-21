/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc2/command/sysid/SysIdRoutine.h>

namespace sys_id {
  namespace drive {
    namespace linear {
      frc2::sysid::Mechanism GetMechanism();
      frc2::sysid::SysIdRoutine GetRoutine();
    }                      // namespace linear
    namespace rotation {}  // namespace rotation
  }                        // namespace drive
}  // namespace sys_id
