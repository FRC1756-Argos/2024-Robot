/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <choreo/lib/ChoreoSwerveCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <string>

#include "subsystems/swerve_drive_subsystem.h"

class DriveChoreo : public frc2::CommandHelper<frc2::Command, DriveChoreo> {
 public:
  DriveChoreo(SwerveDriveSubsystem* drive, const std::string& trajectoryName);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDriveSubsystem* m_pDrive;
  choreolib::ChoreoSwerveCommand m_ChoreoCommand;
};
