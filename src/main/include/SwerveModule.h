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
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

namespace util {

class SwerveModule {
 public:
  // Can default the bus name to "rio" for "roboRIO"
  SwerveModule(int drivePort,
               int turnPort,
               int canCoderPort,
               double angleOffset = 0.0,
               bool isDriveInverted = false,  // Deprecated for removal
               std::string const &driveBusName = "rio",
               std::string const &canCoderBusName = "rio");
  
  // For use in Drivebase.cpp to minimize object creation of configuration objects
  void ApplyConfigs(ctre::phoenix6::configs::TalonFXConfiguration const &motorCfg,
                    ctre::phoenix6::configs::MagnetSensorConfigs &magnetCfg,
                    double angleOffset);
  void SetDesiredState(frc::SwerveModuleState const &desiredState);
  rev::REVLibError ZeroTurnEncoder();

 private:
  ctre::phoenix6::hardware::TalonFX m_driveMotor;
  rev::CANSparkMax m_turnMotor;
  ctre::phoenix6::hardware::CANcoder m_canCoder;
  rev::SparkRelativeEncoder m_turnEncoder;

  /** Deprecated for removal */
  void ConfigMotors(bool isInverted);
  void ConfigDriveMotor(bool isDriveInverted);
  void ConfigCANcoder(double angleOffset);
  /** ********************** */
  void ConfigTurnMotor(bool isInverted);
  units::turn_t GetAbsoluteNumTurns();
  units::turns_per_second_t GetDriveVelocity();
};
} // namespace util

#endif // #ifndef _UTIL_SWERVE_MODULE_H_
