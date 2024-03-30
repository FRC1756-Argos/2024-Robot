/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/drive_choreo.h"

#include <choreo/lib/Choreo.h>
#include <frc/DriverStation.h>
#include <units/math.h>

#include "constants/field_points.h"

DriveChoreo::DriveChoreo(SwerveDriveSubsystem& drive, const std::string& trajectoryName, const bool initializeOdometry)
    : m_Drive{drive}
    , m_trajectory{choreolib::Choreo::GetTrajectory(trajectoryName)}
    , m_ChoreoCommand{m_trajectory,
                      [&drive]() { return drive.GetRawOdometry(); },
                      drive.GetChoreoControllerFunction(),
                      [&drive](frc::ChassisSpeeds speeds) { return drive.SwerveDrive(speeds); },
                      []() {
                        const auto alliance = frc::DriverStation::GetAlliance();
                        return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
                      },
                      {&m_Drive}}
    , m_initializeOdometry{initializeOdometry} {}

// Called when the command is initially scheduled.
void DriveChoreo::Initialize() {
  if (m_initializeOdometry) {  // Initial odometry changes base on alliance because choreo always uses odometry relative to blue alliance origin
    const auto alliance = frc::DriverStation::GetAlliance();
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
      m_Drive.InitializeOdometry(m_trajectory.GetFlippedInitialPose());
    } else {
      m_Drive.InitializeOdometry(m_trajectory.GetInitialPose());
    }
    // Driver still wants orientation relative to alliance station
    if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
      m_Drive.FieldHome(-m_trajectory.GetInitialPose().Rotation().Degrees(), false);
    } else {
      m_Drive.FieldHome(m_trajectory.GetInitialPose().Rotation().Degrees(), false);
    }
  }
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

bool DriveChoreo::IsAtEndPoint(SwerveDriveSubsystem& drive,
                               const std::string& trajectoryName,
                               const units::inch_t translationalTolerance,
                               const units::degree_t rotationalTolerance) {
  const auto position = drive.GetRawOdometry();
  const auto alliance = frc::DriverStation::GetAlliance();
  const auto needFlipped = alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
  const auto trajectory = choreolib::Choreo::GetTrajectory(trajectoryName);
  const auto desiredPosition = needFlipped ? trajectory.GetFlippedFinalPose() : trajectory.GetFinalPose();
  return units::math::abs((position.Rotation() - desiredPosition.Rotation()).Degrees()) <= rotationalTolerance &&
         units::math::abs((position.Translation() - desiredPosition.Translation()).Norm()) <= translationalTolerance;
}

units::inch_t DriveChoreo::EndpointShotDistance(const std::string& trajectoryName) {
  return units::math::abs((choreolib::Choreo::GetTrajectory(trajectoryName).GetFinalPose().Translation() -
                           field_points::blue_alliance::april_tags::speakerCenter.pose.ToTranslation2d())
                              .Norm()) +
         10_in;  // 10 inches converts camera position to center of robot
}
