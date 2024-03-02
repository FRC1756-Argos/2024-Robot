/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/trap_aim_command.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "constants/measure_up.h"

TrapAimCommand::TrapAimCommand(SwerveDriveSubsystem* swerveDrive,
                               VisionSubsystem* vision,
                               argos_lib::SwappableControllersSubsystem* controllers,
                               SimpleLedSubsystem* leds,
                               bool endWhenAligned)
    // Use addRequirements() here to declare subsystem dependencies.
    : m_pSwerveDrive{swerveDrive}
    , m_pVision{vision}
    , m_pControllers{controllers}
    , m_pLeds{leds}
    , m_endWhenAligned{endWhenAligned} {
  AddRequirements(m_pSwerveDrive);
}

// Called when the command is initially scheduled.
void TrapAimCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TrapAimCommand::Execute() {
  auto horzOffset = m_pVision->GetHorizontalOffsetToTrap();
  auto lateralOffset = m_pVision->GetLateralOffsetToTrap();
  auto orientation = m_pVision->GetDistanceToTrap();
  bool aligned = false;

  if (horzOffset != std::nullopt && lateralOffset != std::nullopt && orientation != std::nullopt) {
    frc::SmartDashboard::PutNumber("(TRAP) HorzOffset", horzOffset.value().to<double>());
    frc::SmartDashboard::PutNumber("(TRAP) LatOffset", lateralOffset.value().to<double>());
    frc::SmartDashboard::PutNumber("(TRAP) Distance", orientation.value().to<double>());
    double hOffset = horzOffset.value().to<double>();
    hOffset -= 6.0;
    double lOffset =
        lateralOffset.value().to<double>() + measure_up::shooter_targets::cameraOffsetFromShooter.to<double>();
    double orien = orientation.value().to<double>();

    frc::SmartDashboard::PutNumber("(TRAP) hOffset", hOffset);
    frc::SmartDashboard::PutNumber("(TRAP) lOffset", lOffset);
    if (std::abs(hOffset) > 9.0) {
      m_pSwerveDrive->SwerveDrive(0, hOffset * 0.016, 0);
    } else if (std::abs(hOffset) < 9.0) {
      // m_pSwerveDrive->SwerveDrive(0, 0, -hOffset * 0.016);
      if (orien > 40.0) {
        m_pSwerveDrive->SwerveDrive(-orien * 0.002, 0, 0);
      } else {
        aligned = true;
      }
    }
  }

  // aligned = std::abs(hOffset) < 1;
  if (aligned) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationAlternatePulse(500_ms, 1.0, 0.0));
  }
}

// Called once the command ends or is interrupted.
void TrapAimCommand::End(bool interrupted) {
  if (m_pControllers) {
    m_pControllers->DriverController().SetVibration(argos_lib::VibrationOff());
  }
}

// Returns true when the command should end.
bool TrapAimCommand::IsFinished() {
  return m_endWhenAligned;
}
