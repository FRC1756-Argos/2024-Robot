/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/odometry_aim.h"

#include <cmath>

#include "units/math.h"

units::degree_t argos_lib::odometry_aim::GetAngleToTarget(const frc::Translation3d& currentEstimatedRobotPose,
                                                          const frc::Translation3d& targetPoseOnField) {
  double yawToTarget = std::atan2((targetPoseOnField.Y() - currentEstimatedRobotPose.Y()).to<double>(),
                                  (targetPoseOnField.X() - currentEstimatedRobotPose.X()).to<double>());

  return units::degree_t(yawToTarget * 180.0 / 3.14159265358);
}

units::meter_t argos_lib::odometry_aim::GetDistanceToTarget(const frc::Translation3d& currentEstimatedRobotPose,
                                                            const frc::Translation3d& targetPoseOnField) {
  double Ydiff = (targetPoseOnField.Y() - currentEstimatedRobotPose.Y()).to<double>();
  double Xdiff = (targetPoseOnField.X() - currentEstimatedRobotPose.X()).to<double>();
  double distToTarget = std::sqrt((Ydiff * Ydiff) + (Xdiff * Xdiff));

  return units::meter_t(distToTarget);
}
