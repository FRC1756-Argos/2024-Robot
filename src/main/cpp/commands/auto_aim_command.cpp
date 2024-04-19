/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/auto_aim_command.h"

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
    , m_aimedDebouncer{{300_ms, 0_ms}}
    , m_aimed{false} {
  AddRequirements({m_pSwerveDrive, m_pShooter, m_pElevator});
}

// Called when the command is initially scheduled.
void AutoAimCommand::Initialize() {
  m_pShooter->SetAmpMode(false);
  m_pShooter->SetTrapMode(false);
  m_pSwerveDrive->StopDrive();
  // Start aiming early so we can finish early if possible
  const auto aimParams = GetAimParams();
  if (aimParams) {
    m_aimedDebouncer.Reset(Aim(aimParams));  // might already be aimed
    SetOperatorFeedback(aimParams);
  } else {
    m_pShooter->ShooterGoToSpeed(5300_rpm);
    m_aimedDebouncer.Reset(false);
  }
  m_aimed = m_aimedDebouncer.GetRawStatus();
}

// Called repeatedly when this Command is scheduled to run
void AutoAimCommand::Execute() {
  const auto aimParams = GetAimParams();

  SetOperatorFeedback(aimParams);
  (void)m_aimedDebouncer(Aim(aimParams) || m_aimed);
}

// Called once the command ends or is interrupted.
void AutoAimCommand::End(bool interrupted) {
  if (m_pControllers) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationOff());
  }
  m_pSwerveDrive->StopDrive();
}

// Returns true when the command should end.
bool AutoAimCommand::IsFinished() {
  return m_endWhenAimed && m_aimedDebouncer.GetDebouncedStatus();
}
std::optional<AutoAimCommand::AimParams> AutoAimCommand::GetAimParams() {
  auto angle = m_pVision->getShooterAngle();
  auto speed = m_pVision->getShooterSpeed();
  auto horzOffset = m_pVision->GetHorizontalOffsetToTarget();
  auto cameraOffset = m_pVision->getShooterOffset();

  if (angle && speed && horzOffset && cameraOffset) {
    return AimParams{angle.value(), speed.value(), horzOffset.value(), cameraOffset.value()};
  }
  return std::nullopt;
}

std::optional<units::degree_t> AutoAimCommand::GetAdjustmentOffset(const std::optional<AimParams>& params) {
  if (!params) {
    return std::nullopt;
  }
  return params.value().targetAngleOffset - params.value().cameraAngleOffset;
}

bool AutoAimCommand::Aimed(const std::optional<AimParams>& params) {
  const auto offset = GetAdjustmentOffset(params);
  if (!offset) {
    return false;
  }

  return std::abs(offset.value().to<double>()) < 5.0 && m_pShooter->ShooterAtSpeed() &&
         m_pElevator->IsElevatorAtSetPoint();
}

bool AutoAimCommand::Aim(const std::optional<AimParams>& params) {
  if (params) {
    const auto offset = GetAdjustmentOffset(params);
    if (m_pVision->IsOdometryAimingActive()) {
      m_pSwerveDrive->SwerveDrive(offset.value(), 0.0, 0.2);
    } else {
      m_pSwerveDrive->SwerveDrive(0, 0, -offset.value().to<double>() * 0.016);
    }
    m_pElevator->SetCarriageAngle(params.value().carriageAngle);
    m_pShooter->ShooterGoToSpeed(params.value().shooterSpeed);
  }

  bool aimedThisCycle = Aimed(params);
  m_aimed |= aimedThisCycle;
  return aimedThisCycle;
}

void AutoAimCommand::SetOperatorFeedback(const std::optional<AimParams>& params) {
  if (params) {
    const auto aimed = Aimed(params);
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
}
