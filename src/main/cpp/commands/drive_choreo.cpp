/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_choreo.h"

#include <choreo/lib/Choreo.h>
#include <frc/DriverStation.h>

DriveChoreo::DriveChoreo(SwerveDriveSubsystem& drive, const std::string& trajectoryName)
    : m_Drive{drive}
    , m_ChoreoCommand{choreolib::Choreo::GetTrajectory(trajectoryName),
                      [&drive]() { return drive.GetContinuousOdometry(); },
                      drive.GetChoreoControllerFunction(),
                      [&drive](frc::ChassisSpeeds speeds) { return drive.SwerveDrive(speeds); },
                      []() {
                        const auto alliance = frc::DriverStation::GetAlliance();
                        return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
                      },
                      {&m_Drive}} {}

// Called when the command is initially scheduled.
void DriveChoreo::Initialize() {
  m_ChoreoCommand.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void DriveChoreo::Execute() {
  m_ChoreoCommand.Execute();
}

// Called once the command ends or is interrupted.
void DriveChoreo::End(bool interrupted) {
  m_ChoreoCommand.End(interrupted);
}

// Returns true when the command should end.
bool DriveChoreo::IsFinished() {
  return m_ChoreoCommand.IsFinished();
}
