// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // m_masterController.B().OnTrue(m_drive.GetResetHeadingCmd());
  // m_drive.SetDefaultCommand(
  //   frc2::cmd::Run([this] () { m_drive.Drive(
  //     -m_masterController.GetLeftY(),
  //     -m_masterController.GetLeftX(),
  //     -m_masterController.GetRightX(),
  //     true,
  //     20_ms); },
  //   {&m_drive}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
