// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/test.h"

test::test() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void test::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void test::Execute() {}

// Called once the command ends or is interrupted.
void test::End(bool interrupted) {}

// Returns true when the command should end.
bool test::IsFinished() {
  return false;
}
