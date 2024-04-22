// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#ifndef _UTIL_SWERVE_MODULE_H_
#define _UTIL_SWERVE_MODULE_H_

#include <string>

// #include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkRelativeEncoder.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "../include/Constants.h"

using namespace ctre::phoenix6;
using namespace units;

class SwerveModule {
 public:
  // Can default the bus name to "rio" for "roboRIO"
  SwerveModule(int drivePort,
               int turnPort,
               int canCoderPort,
               double angleOffset = 0.0,
               bool isDriveInverted = false,
               std::string const &driveBusName = "rio",
               std::string const &canCoderBusName = "rio");
  void SetDesiredState(frc::SwerveModuleState const &desiredState);
  rev::REVLibError ZeroTurnEncoder();

 private:
  hardware::TalonFX m_driveMotor;
  rev::CANSparkMax m_turnMotor;
  hardware::CANcoder m_canCoder;
  rev::SparkRelativeEncoder m_turnEncoder;

  void ConfigMotors(bool isInverted);
  void ConfigDriveMotor(bool isDriveInverted);
  void ConfigTurnMotor(bool isInverted);
  void ConfigCANcoder(double angleOffset);
  angle::turn_t GetAbsoluteNumTurns();
  angular_velocity::turns_per_second_t GetDriveVelocity();
};

#endif // #ifndef _UTIL_SWERVE_MODULE_H_
