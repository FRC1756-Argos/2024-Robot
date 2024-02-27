/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "commands/autonomous/autonomous_choreo_test.h"

#include "commands/drive_choreo.h"
#include "commands/initialize_odometry_command.h"

AutonomousChoreoTest::AutonomousChoreoTest(ElevatorSubsystem& elevator,
                                           IntakeSubsystem& intake,
                                           ShooterSubsystem& shooter,
                                           SwerveDriveSubsystem& swerve,
                                           VisionSubsystem& vision)
    : m_Elevator{elevator}
    , m_Intake{intake}
    , m_Shooter{shooter}
    , m_Swerve{swerve}
    , m_Vision{vision}
    , m_allCommands{InitializeOdometryCommand{&m_Swerve, {1.45_m, 7.026_m, 0_deg}}, DriveChoreo{m_Swerve, "TestPath"}} {
}

// Called when the command is initially scheduled.
void AutonomousChoreoTest::Initialize() {
  m_allCommands.Initialize();
}

// Called repeatedly when this Command is scheduled to run
void AutonomousChoreoTest::Execute() {
  m_allCommands.Execute();
}

// Called once the command ends or is interrupted.
void AutonomousChoreoTest::End(bool interrupted) {
  m_allCommands.End(interrupted);
}

// Returns true when the command should end.
bool AutonomousChoreoTest::IsFinished() {
  return m_allCommands.IsFinished();
}

std::string AutonomousChoreoTest::GetName() const {
  return "99. Choreo Test";
}

frc2::Command* AutonomousChoreoTest::GetCommand() {
  return dynamic_cast<frc2::Command*>(this);
}
