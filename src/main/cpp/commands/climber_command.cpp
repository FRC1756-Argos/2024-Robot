/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/climber_command.h"

#include <frc2/command/SequentialCommandGroup.h>

#include <frc/smartdashboard/SmartDashboard.h>

ClimberCommand::ClimberCommand(ClimberSubsystem* climber,
                               ShooterSubsystem* shooter,
                               ElevatorSubsystem* elevator,
                               argos_lib::SwappableControllersSubsystem* controllers,
                               ReadyForClimbCommand* ready,
                               RaiseClimberCommand* raise,
                               LowerClimberCommand* lower,
                               GoToTrapPositionCommand* trap)
    : m_pClimber{climber}
    , m_pShooter{shooter}
    , m_pElevator{elevator}
    , m_pControllers{controllers}
    , m_pReadyForClimbCommand{ready}
    , m_pRaiseClimberCommand{raise}
    , m_pLowerClimberCommand{lower}
    , m_pTrapCommand{trap}
    {
}

// Called when the command is initially scheduled.
void ClimberCommand::Initialize() {
  m_pReadyForClimbCommand->Schedule();
  button_count = 0;
  m_pTrapCommand->ResetIsTrapDone();
}

// Called repeatedly when this Command is scheduled to run
void ClimberCommand::Execute() {

  if(m_pControllers->OperatorController().GetRawButtonPressed(argos_lib::XboxController::Button::kBumperLeft)){

    switch(button_count){
      case 0:
        m_pRaiseClimberCommand->Schedule();
        ++button_count;
        break;
      case 1:
        m_pLowerClimberCommand->Schedule();
        ++button_count;
        break;
      case 2:
        m_pTrapCommand->Schedule();
        ++button_count;
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
  return m_pTrapCommand->GetIsTrapDone();
}
