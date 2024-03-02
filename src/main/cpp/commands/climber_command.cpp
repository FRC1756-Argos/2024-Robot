/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climber_command.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitCommand.h>

ClimberCommand::ClimberCommand(ClimberSubsystem* climber,
                               ShooterSubsystem* shooter,
                               ElevatorSubsystem* elevator,
                               argos_lib::SwappableControllersSubsystem* controllers)
    : m_pClimber{climber}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_pControllers{controllers}
    , m_ReadyForClimbCommand{ReadyForClimbCommand{shooter, elevator}}
    , m_RaiseClimberCommand{RaiseClimberCommand{climber}}
    , m_LowerClimberCommand{LowerClimberCommand{climber}}
    , m_TrapCommand{GoToTrapPositionCommand{shooter, elevator}}
    // , m_SeqCommand{LowerClimberCommand{m_pClimber}, GoToTrapPositionCommand{m_pShooter, m_pElevator}, frc2::WaitCommand(400_ms), ShooterCommand{shooter}}
    , m_SeqCommand{LowerClimberCommand{m_pClimber}, GoToTrapPositionCommand{m_pShooter, m_pElevator}}
    , m_ShootCommand{m_pShooter} {}

// Called when the command is initially scheduled.
void ClimberCommand::Initialize() {
  end_command = false;
  m_ReadyForClimbCommand.Schedule();
  button_count = 0;
  m_TrapCommand.ResetIsTrapDone();
}

// Called repeatedly when this Command is scheduled to run
void ClimberCommand::Execute() {
  if (m_pControllers->OperatorController().GetRawButtonPressed(argos_lib::XboxController::Button::kBumperLeft)) {
    switch (button_count) {
      case 0:
        m_RaiseClimberCommand.Schedule();
        ++button_count;
        break;
      case 1:
        m_SeqCommand.Schedule();
        ++button_count;
        break;
      case 2:
        m_ShootCommand.Schedule();
        ++button_count;
        end_command = true;
        break;
      default:
        button_count = 0;
    }
  }
}

// Called once the command ends or is interrupted.
void ClimberCommand::End(bool interrupted) {
  button_count = 0;
}

// Returns true when the command should end.
bool ClimberCommand::IsFinished() {
  return end_command;
}
