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
                               SimpleLedSubsystem* leds,
                               bool endWhenAimed)
    : m_pSwerveDrive{swerveDrive}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_pVision{vision}
    , m_pControllers{controllers}
    , m_pLeds{leds}
    , m_endWhenAimed{endWhenAimed}
    , m_aimedDebouncer{{150_ms, 0_ms}} {
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
  auto speed = m_pVision->getShooterSpeed();
  if (angle != std::nullopt) {
    frc::SmartDashboard::PutNumber("(AIM) angle", angle.value().to<double>());
    m_pElevator->SetCarriageAngle(angle.value());
  }

  if (speed != std::nullopt) {
    frc::SmartDashboard::PutNumber("(AIM) speed",
                                   units::angular_velocity::revolutions_per_minute_t{speed.value()}.to<double>());
    m_pShooter->ShooterGoToSpeed(speed.value());
  }

  auto horzOffset = m_pVision->GetHorizontalOffsetToTarget();
  auto cameraOffset = m_pVision->getShooterOffset();

  bool aimed = false;

  if (horzOffset != std::nullopt && cameraOffset != std::nullopt) {
    frc::SmartDashboard::PutNumber("(AIM) offset", horzOffset.value().to<double>());
    double offset = horzOffset.value().to<double>();
    offset -= cameraOffset.value().to<double>();

    m_pSwerveDrive->SwerveDrive(0, 0, -offset * 0.016);
    if (m_pVision->IsStaticRotationEnabled()) {
      frc::SmartDashboard::PutNumber("(AIM) offset2", offset);
    }

    aimed = std::abs(offset) < 5 && m_pShooter->ShooterAtSpeed();

    if (m_pLeds) {
      if (aimed) {
        m_pLeds->TemporaryAnimate(
            [this]() { m_pLeds->SetAllGroupsColor(argos_lib::gamma_corrected_colors::kReallyGreen, false); }, 200_ms);
      }
    }
    if (aimed) {
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
  (void)m_aimedDebouncer(aimed);
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {
  if (m_pControllers) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationOff());
  }
}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  return m_endWhenAimed && m_aimedDebouncer.GetDebouncedStatus();
}
