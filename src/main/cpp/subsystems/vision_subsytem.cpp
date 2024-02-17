/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "constants/field_points.h"
#include "subsystems/vision_subsystem.h"

CameraInterface::CameraInterface() = default;

void CameraInterface::RequestTargetFilterReset() {
  m_target.ResetOnNextTarget();
}

VisionSubsystem::VisionSubsystem(const argos_lib::RobotInstance instance, SwerveDriveSubsystem* pDriveSubsystem)
    : m_instance(instance)
    , m_pDriveSubsystem(pDriveSubsystem)
    , m_usePolynomial(false)
    , m_shooterAngleMap{shooterRange::shooterAngle} {}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
  LimelightTarget::tValues targetValues = GetCameraTargetValues();  // Note that this will update the targets object

  if (targetValues.hasTargets) {
    frc::SmartDashboard::PutBoolean("(Vision - Periodic) Is Target Present?", targetValues.hasTargets);
    frc::SmartDashboard::PutNumber("(Vision - Periodic) Target Pitch", targetValues.m_pitch.to<double>());
    frc::SmartDashboard::PutNumber("(Vision - Periodic) Target Yaw", targetValues.m_yaw.to<double>());
    frc::SmartDashboard::PutNumber("(Vision - Periodic) Tag ID", targetValues.tagID);

    if (GetDistanceToSpeaker() != std::nullopt) {
      frc::SmartDashboard::PutNumber("(Vision - Periodic) Tag Distance from Camera",
                                     GetDistanceToSpeaker().value().to<double>());
    }

    if (GetCalculatedDistanceToSpeaker() != std::nullopt) {
      frc::SmartDashboard::PutNumber("(Vision - Periodic) Calculated Tag Distance from Camera",
                                     GetCalculatedDistanceToSpeaker().value().to<double>());
    }

    if (getShooterAngle() != std::nullopt) {
      frc::SmartDashboard::PutNumber("(Vision - Periodic) Shooter Angle", getShooterAngle().value().to<double>());
    }
  }
}

std::optional<units::degree_t> VisionSubsystem::GetHorizontalOffsetToTarget() {
  // Updates and retrieves new target values
  LimelightTarget::tValues targetValues = GetCameraTargetValues();

  // add more target validation after testing e.g. area, margin, skew etc
  // for now has target is enough as we will be fairly close to target
  // and will tune the pipeline not to combine detections and choose the highest area
  if (targetValues.hasTargets) {
    return targetValues.m_yaw;
  }

  return std::nullopt;
}

std::optional<units::degree_t> VisionSubsystem::getShooterAngle() {
  auto distance = GetDistanceToSpeaker();

  if (distance) {
    double d = distance.value().to<double>();
    if (m_usePolynomial) {
      d /= 12.0;
      return units::degree_t(83.4 - (9.38 * d) + (0.531 * d * d) - (0.0103 * d * d * d));
    } else {
      return m_shooterAngleMap.Map(distance.value());
    }
  }

  return std::nullopt;
}

std::optional<units::inch_t> VisionSubsystem::GetDistanceToSpeaker() {
  int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ?
                          field_points::blue_alliance::april_tags::speakerCenter.id :
                          field_points::red_alliance::april_tags::speakerCenter.id;
  const auto targetValues = GetCameraTargetValues();
  if (tagOfInterest == targetValues.tagID)
    return static_cast<units::inch_t>(targetValues.tagPose.Z());
  else
    return std::nullopt;
}

std::optional<units::inch_t> VisionSubsystem::GetCalculatedDistanceToSpeaker() {
  int tagOfInterest = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? 7 : 4;
  if (tagOfInterest == GetCameraTargetValues().tagID) {
    return (measure_up::shooter_targets::speakerTagHeight - measure_up::camera_front::cameraHeight) /
           std::tan(static_cast<units::radian_t>(measure_up::camera_front::cameraMountAngle +
                                                 GetCameraTargetValues().m_pitch)
                        .to<double>());
  } else {
    return std::nullopt;
  }
}

void VisionSubsystem::SetPipeline(uint16_t tag) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  uint16_t pipeline = 0;
  switch (tag) {
    case field_points::red_alliance::april_tags::speakerCenter.id:
      pipeline = 0;
      break;
    case field_points::red_alliance::april_tags::amp.id:
      pipeline = 2;
      break;
    case field_points::blue_alliance::april_tags::amp.id:
      pipeline = 3;
      break;
    case field_points::blue_alliance::april_tags::speakerCenter.id:
      pipeline = 1;
      break;
    default:
      break;
  }
  frc::SmartDashboard::PutNumber("(Vision) Setting Pipeline", tag);

  table->PutNumber("pipeline", tag);
}

void VisionSubsystem::RequestFilterReset() {
  m_cameraInterface.RequestTargetFilterReset();
}

LimelightTarget::tValues VisionSubsystem::GetCameraTargetValues() {
  return m_cameraInterface.m_target.GetTarget(true);
}

void VisionSubsystem::Disable() {
  SetPipeline(0);
}

// LIMELIGHT TARGET MEMBER FUNCTIONS ===============================================================

LimelightTarget::tValues LimelightTarget::GetTarget(bool filter) {
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  auto rawRobotPose = table->GetNumberArray("botpose", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPose = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotPose.at(0)),
                                               units::make_unit<units::meter_t>(rawRobotPose.at(1)),
                                               units::make_unit<units::meter_t>(rawRobotPose.at(2))),
                            frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotPose.at(3)),
                                            units::make_unit<units::radian_t>(rawRobotPose.at(4)),
                                            units::make_unit<units::radian_t>(rawRobotPose.at(5))));
  auto rawRobotPoseWPI = table->GetNumberArray(
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue ? "botpose_wpiblue" : "botpose_wpired",
      std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_robotPoseWPI = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(rawRobotPoseWPI.at(0)),
                                                  units::make_unit<units::meter_t>(rawRobotPoseWPI.at(1)),
                                                  units::make_unit<units::meter_t>(rawRobotPoseWPI.at(2))),
                               frc::Rotation3d(units::make_unit<units::radian_t>(rawRobotPoseWPI.at(3)),
                                               units::make_unit<units::radian_t>(rawRobotPoseWPI.at(4)),
                                               units::make_unit<units::radian_t>(rawRobotPoseWPI.at(5))));
  auto tagPoseCamSpace =
      table->GetNumberArray("targetpose_cameraspace", std::span<const double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  m_targetPose = frc::Pose3d(frc::Translation3d(units::make_unit<units::meter_t>(tagPoseCamSpace.at(0)),
                                                units::make_unit<units::meter_t>(tagPoseCamSpace.at(1)),
                                                units::make_unit<units::meter_t>(tagPoseCamSpace.at(2))),
                             frc::Rotation3d(units::make_unit<units::radian_t>(tagPoseCamSpace.at(3)),
                                             units::make_unit<units::radian_t>(tagPoseCamSpace.at(4)),
                                             units::make_unit<units::radian_t>(tagPoseCamSpace.at(5))));

  auto tagId = table->GetNumber("tid", 0.0);
  m_tid = tagId;
  m_hasTargets = (table->GetNumber("tv", 0) == 1);
  m_yaw = units::make_unit<units::degree_t>(table->GetNumber("tx", 0.0));
  m_pitch = units::make_unit<units::degree_t>(table->GetNumber("ty", 0.0));

  // * debugging
  frc::SmartDashboard::PutNumber("VisionSubsystem/RawPitch (deg)", m_pitch.to<double>());
  frc::SmartDashboard::PutNumber("VisionSubsystem/RawYaw (deg)", m_yaw.to<double>());
  // * end debugging

  // If filter needs to reset, reset filter
  if (m_hasTargets && m_resetFilterFlag) {
    ResetFilters();
  }

  // Filter incoming yaw & pitch if wanted
  if (filter && m_hasTargets) {
    m_yaw = m_txFilter.Calculate(m_yaw);
    m_pitch = m_tyFilter.Calculate(m_pitch);
    m_targetPose.Z() = m_zFilter.Calculate(m_targetPose.Z());

    // * debugging
    frc::SmartDashboard::PutNumber("VisionSubsystem/FilteredPitch (deg)", m_pitch.to<double>());
    frc::SmartDashboard::PutNumber("VisionSubsystem/FilteredYaw (deg)", m_yaw.to<double>());
    // * end debugging
  }

  m_area = (table->GetNumber("ta", 0.0));
  m_totalLatency = units::make_unit<units::millisecond_t>(rawRobotPose.at(6));

  return tValues{
      m_robotPose, m_robotPoseWPI, m_targetPose, m_hasTargets, m_pitch, m_yaw, m_area, m_tid, m_totalLatency};
}

bool LimelightTarget::HasTarget() {
  return m_hasTargets;
}

void LimelightTarget::ResetOnNextTarget() {
  m_resetFilterFlag = true;
}

void LimelightTarget::ResetFilters() {
  m_resetFilterFlag = false;
  m_txFilter.Reset();
  m_tyFilter.Reset();
  m_zFilter.Reset();
  LimelightTarget::tValues currentValue = GetTarget(false);
  // Hackily rest filter with initial value
  /// @todo name the filter values
  uint32_t samples = 0.7 / 0.02;
  for (size_t i = 0; i < samples; i++) {
    m_txFilter.Calculate(currentValue.m_yaw);
    m_tyFilter.Calculate(currentValue.m_pitch);
    m_zFilter.Calculate(currentValue.tagPose.Z());
  }
}
