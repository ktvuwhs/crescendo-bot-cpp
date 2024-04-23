// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <AHRS.h>

#include <frc/MathUtil.h> // frc::ApplyDeadband()
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Translation2d.h>
#include <frc/SPI.h>
#include <frc2/command/CommandPtr.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/time.h>

#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "../include/Constants.h"
#include "../include/SwerveModule.h"

class Drivebase : public frc2::SubsystemBase {
 public:
  Drivebase();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void Drive(double xSpeed,
             double ySpeed,
             double rot,
             bool fieldRelative,
             units::second_t period);
  void ResetHeading();
  frc2::CommandPtr GetResetHeadingCmd();

 private:
  AHRS m_gyro = AHRS{frc::SPI::Port::kMXP};

  // Swerve modules
  util::SwerveModule m_modules[4] = {
    util::SwerveModule{SwerveModuleConstants::kBotRightDrivePort,
                       SwerveModuleConstants::kBotRightTurnPort,
                       SwerveModuleConstants::kBotRightCancoderPort},
    util::SwerveModule{SwerveModuleConstants::kBotLeftDrivePort,
                       SwerveModuleConstants::kBotLeftTurnPort,
                       SwerveModuleConstants::kBotLeftCancoderPort},
    util::SwerveModule{SwerveModuleConstants::kTopRightDrivePort,
                       SwerveModuleConstants::kTopRightTurnPort,
                       SwerveModuleConstants::kTopRightCancoderPort},
    util::SwerveModule{SwerveModuleConstants::kTopLeftDrivePort,
                       SwerveModuleConstants::kTopLeftTurnPort,
                       SwerveModuleConstants::kTopLeftCancoderPort}};
  
  // SwerveDriveKinematics
  // Note that relative to the robot, +x is up, and +y is left
  frc::Translation2d m_botRight = frc::Translation2d{
    -SwerveModuleConstants::kWheelbase/2,
    -SwerveModuleConstants::kTrackwidth/2};
  frc::Translation2d m_botLeft = frc::Translation2d{
    -SwerveModuleConstants::kWheelbase/2,
    SwerveModuleConstants::kTrackwidth/2};
  frc::Translation2d m_topRight = frc::Translation2d{
    SwerveModuleConstants::kWheelbase/2,
    -SwerveModuleConstants::kTrackwidth/2};
  frc::Translation2d m_topLeft = frc::Translation2d{
    SwerveModuleConstants::kWheelbase/2,
    SwerveModuleConstants::kTrackwidth/2};
  frc::SwerveDriveKinematics<4> m_kinematics = 
    frc::SwerveDriveKinematics<4>{
      m_botRight,
      m_botLeft,
      m_topRight,
      m_topLeft};
    
  // SwerveDriveOdometry
  frc::SwerveDriveOdometry<4> m_odometry = 
    frc::SwerveDriveOdometry<4>{
      m_kinematics,
      m_gyro.GetRotation2d(),
      wpi::array<frc::SwerveModulePosition, 4>{
        m_modules[0].GetPosition(),
        m_modules[1].GetPosition(),
        m_modules[2].GetPosition(),
        m_modules[3].GetPosition()}};
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{DrivebaseConstants::kXSlewRate};
  frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{DrivebaseConstants::kYSlewRate};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{DrivebaseConstants::kRotSlewRate};
  
  ctre::phoenix6::configs::TalonFXConfiguration GetTalonFXConfiguration(bool isDriveInverted) const;
  ctre::phoenix6::configs::MagnetSensorConfigs GetCANCoderConfiguration() const;
  void ConfigureModules(ctre::phoenix6::configs::TalonFXConfiguration const &motorCfg,
                        ctre::phoenix6::configs::MagnetSensorConfigs &magnetCfg);
};
