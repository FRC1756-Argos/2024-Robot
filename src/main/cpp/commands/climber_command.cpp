/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climber_command.h"

#include <frc2/command/SequentialCommandGroup.h>

ClimberCommand::ClimberCommand(ClimberSubsystem* climber, ShooterSubsystem* shooter, ElevatorSubsystem* elevator,
argos_lib::SwappableControllersSubsystem* controllers, ReadyForClimbCommand* ready, RaiseClimberCommand* raise, LowerClimberCommand* lower, GoToTrapPositionCommand* trap)
    : m_pClimber{climber}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_pControllers{controllers}
    , m_pReadyForClimbCommand{ready}
    , m_pRaiseClimberCommand{raise}
    , m_pLowerClimberCommand{lower}
    , m_pTrapCommand{trap} {
  AddRequirements({m_pClimber, m_pShooter, m_pElevator, m_pControllers});
}

// Called when the command is initially scheduled.
void ClimberCommand::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ClimberCommand::Execute() {
  if(m_pControllers->OperatorController().GetRawButtonPressed(argos_lib::XboxController::Button::kBumperRight)){
    if(!(m_pReadyForClimbCommand->GetIsReadyCLimbFinished())){
      m_pReadyForClimbCommand->Schedule();
    }
    else if(m_pReadyForClimbCommand->GetIsReadyCLimbFinished() && !(m_pRaiseClimberCommand->GetIsRaiseCLimbFinished())){
      m_pRaiseClimberCommand->Schedule();
    }
    else if(m_pRaiseClimberCommand->GetIsRaiseCLimbFinished() && !(m_pLowerClimberCommand->GetIsLowerCLimbFinished())){
      m_pLowerClimberCommand->Schedule();
    }
    else if(m_pLowerClimberCommand->GetIsLowerCLimbFinished() && !(m_pTrapCommand->GetIsTrapDone())){
      m_pTrapCommand->Schedule();
    }
  }
}

// Called once the command ends or is interrupted.
void ClimberCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberCommand::IsFinished() {
  return m_pTrapCommand->GetIsTrapDone();
}
