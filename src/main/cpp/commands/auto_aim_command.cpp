/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_aim_command.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "constants/measure_up.h"

AutoAimCommand::AutoAimCommand(SwerveDriveSubsystem* swerveDrive,
                               ShooterSubsystem* shooter,
                               ElevatorSubsystem* elevator,
                               VisionSubsystem* vision,
                               argos_lib::SwappableControllersSubsystem* controllers,
                               SimpleLedSubsystem* leds)
    : m_pSwerveDrive{swerveDrive}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_pVision{vision}
    , m_pControllers{controllers}
    , m_pLeds{leds} {
  AddRequirements({m_pSwerveDrive, m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void AutoAimCommand::Initialize() {
  m_pShooter->ShooterGoToSpeed(5000_rpm);
  m_pShooter->SetAmpAndTrapMode(false);
  m_pSwerveDrive->StopDrive();
}

// Called repeatedly when this Command is scheduled to run
void AutoAimCommand::Execute() {
  auto angle = m_pVision->getShooterAngle();
  if (angle != std::nullopt) {
    frc::SmartDashboard::PutNumber("(AIM) angle", angle.value().to<double>());
    m_pElevator->SetCarriageAngle(angle.value());
  }

  auto horzOffset = m_pVision->GetHorizontalOffsetToTarget();
  auto cameraOffset = m_pVision->getShooterOffset();

  if (horzOffset != std::nullopt && cameraOffset != std::nullopt) {
    frc::SmartDashboard::PutNumber("(AIM) offset", horzOffset.value().to<double>());
    double offset = horzOffset.value().to<double>();
    offset -= cameraOffset.value().to<double>();
    m_pSwerveDrive->SwerveDrive(0.0, 0.0, -offset * 0.016);
    frc::SmartDashboard::PutNumber("(AIM) offset2", offset);

    if (m_pLeds) {
      if (std::abs(offset) < 5 && m_pShooter->ShooterAtSpeed()) {
        m_pLeds->TemporaryAnimate(
            [this]() { m_pLeds->SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyGreen, false); }, 200_ms);
      }
    }
    if (m_pShooter->ShooterAtSpeed() && std::abs(offset) < 5) {
      if (m_pControllers) {
        m_pControllers->DriverController().SetVibration(argos_lib::VibrationAlternatePulse(500_ms, 1.0, 0.0));
      }
      if (m_pLeds) {
        m_pLeds->TemporaryAnimate(
            [this]() { m_pLeds->SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyGreen, false); }, 200_ms);
      }
    } else if (m_pLeds) {
      m_pLeds->TemporaryAnimate(
          [this]() { m_pLeds->SetAllGroupsColor(argos_lib::gamma_corrected_colors::kCatYellow, false); }, 200_ms);
    }
  } else if (m_pLeds) {
    m_pLeds->TemporaryAnimate(
        [this]() { m_pLeds->SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyRed, false); }, 200_ms);
  }
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {
  if (m_pControllers) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationOff());
  }
}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  return false;
}
