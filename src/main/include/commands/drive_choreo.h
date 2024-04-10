/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <choreo/lib/ChoreoSwerveCommand.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>

#include <chrono>
#include <string>

#include "subsystems/swerve_drive_subsystem.h"

class DriveChoreo : public frc2::CommandHelper<frc2::Command, DriveChoreo> {
 public:
  DriveChoreo(SwerveDriveSubsystem& drive, const std::string& trajectoryName, const bool initializeOdometry = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  [[nodiscard]] static bool IsAtEndPoint(SwerveDriveSubsystem& drive,
                                         const std::string& trajectoryName,
                                         const units::inch_t translationalTolerance = 6_in,
                                         const units::degree_t rotationalTolerance = 1.0_deg);

  [[nodiscard]] static units::inch_t EndpointShotDistance(const std::string& trajectoryName);

 private:
  SwerveDriveSubsystem& m_Drive;
  const choreolib::ChoreoTrajectory m_trajectory;
  choreolib::ChoreoTrajectory m_orientedTrajectory;
  choreolib::ChoreoSwerveCommand m_ChoreoCommand;
  const bool m_initializeOdometry;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
  frc::Field2d m_field;
};
