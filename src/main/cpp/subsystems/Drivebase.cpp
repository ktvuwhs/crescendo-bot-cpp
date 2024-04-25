// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivebase.h"

#include <functional>

#include <AHRS.h>

#include <frc/MathUtil.h> // frc::ApplyDeadband()
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc2/command/CommandPtr.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "../include/Constants.h"
#include "../include/SwerveModule.h"

namespace configs = ctre::phoenix6::configs;
namespace signals = ctre::phoenix6::signals;
namespace SMConst = SwerveModuleConstants;

Drivebase::Drivebase() {
  m_gyro.Reset();
  configs::TalonFXConfiguration krakenCfg = GetTalonFXConfiguration(true);
  configs::MagnetSensorConfigs magnetCfg = GetCANCoderConfiguration();
  ConfigureModules(krakenCfg, magnetCfg);
}

configs::TalonFXConfiguration Drivebase::GetTalonFXConfiguration(bool const isInverted) const {
  configs::TalonFXConfiguration talonCfg;

  talonCfg.Slot0
    .WithKP(SMConst::kPDrive)
    .WithKD(SMConst::kDDrive)
    .WithKV(SMConst::kVDrive);
  
  talonCfg.CurrentLimits
    .WithSupplyCurrentLimitEnable(true)
    .WithSupplyCurrentLimit(SMConst::kDriveSupplyCurrentLimit);

  talonCfg.Feedback
    .WithSensorToMechanismRatio(SMConst::kDriveGearRatio);

  talonCfg.MotorOutput
    .WithInverted(isInverted ? signals::InvertedValue::Clockwise_Positive
      : signals::InvertedValue::CounterClockwise_Positive)
    .WithNeutralMode(signals::NeutralModeValue::Brake);

  return talonCfg;
}

configs::MagnetSensorConfigs Drivebase::GetCANCoderConfiguration() const {
  configs::MagnetSensorConfigs magnetCfg;
  magnetCfg
    .WithAbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf)
    .WithSensorDirection(signals::SensorDirectionValue::CounterClockwise_Positive);
  return magnetCfg;
}

void Drivebase::ConfigureModules(configs::TalonFXConfiguration const &motorCfg,
                                 configs::MagnetSensorConfigs &magnetCfg) {
  m_modules[0].ApplyConfigs(motorCfg, magnetCfg.WithMagnetOffset(SMConst::kBotRightMagnetOffset));
  m_modules[1].ApplyConfigs(motorCfg, magnetCfg.WithMagnetOffset(SMConst::kBotLeftMagnetOffset));
  m_modules[2].ApplyConfigs(motorCfg, magnetCfg.WithMagnetOffset(SMConst::kTopRightMagnetOffset));
  m_modules[3].ApplyConfigs(motorCfg, magnetCfg.WithMagnetOffset(SMConst::kTopLeftMagnetOffset));
}

void Drivebase::Drive(double const xSpeed, double const ySpeed,
                 double rot, bool const fieldRelative,
                 units::second_t period) {
  // Apply slew rate limiting to xSpeed for smoother driving
  units::meters_per_second_t xSpeedLimited = 
    m_xSpeedLimiter.Calculate(
      frc::ApplyDeadband(xSpeed, DrivebaseConstants::kDeadband)) * DrivebaseConstants::kMaxSpeed;
  
  // Apply slew rate limiting to ySpeed for smoother driving
  units::meters_per_second_t ySpeedLimited =
    m_ySpeedLimiter.Calculate(
      frc::ApplyDeadband(ySpeed, DrivebaseConstants::kDeadband)) * DrivebaseConstants::kMaxSpeed;
  
  // Apply slew rate limiting to rotational speed for smoother driving
  units::radians_per_second_t rotLimited =
    m_rotLimiter.Calculate(
      frc::ApplyDeadband(rot, DrivebaseConstants::kDeadband)) * DrivebaseConstants::kMaxAngularSpeed;

  wpi::array<frc::SwerveModuleState, 4> states = m_kinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::Discretize(
      fieldRelative ?
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          xSpeedLimited,
          ySpeedLimited,
          rotLimited,
          frc::Rotation2d{units::turn_t{-m_gyro.GetYaw()}})
      : frc::ChassisSpeeds{xSpeedLimited, ySpeedLimited, rotLimited},
      period));
  // if (fieldRelative) {
  //   wpi::array<frc::SwerveModuleState, 4> temp = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
  //     frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedLimited,
  //       ySpeedLimited,
  //       rotLimited,
  //       frc::Rotation2d{units::turn_t{-m_gyro.GetYaw()}}),  // NavX is CCW negative, when FRC is CCW positive
  //     period));
  //   states = &temp;
  // } else {
  //   wpi::array<frc::SwerveModuleState, 4> temp = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
  //     frc::ChassisSpeeds{xSpeedLimited, ySpeedLimited, rotLimited},
  //     period));
  //   states = &temp;
  // }

  // According to WPILib, module states are not normalized, and user input can cause module speeds to go
  // above attainable max velocity.  Desaturating wheel speeds will fix this issue.  Desaturating the
  // wheel speeds makes sure all module speeds are at or below the maximum speed while maintaining the
  // ratio of speeds between modules.
  m_kinematics.DesaturateWheelSpeeds(&states, DrivebaseConstants::kMaxSpeed);
  
  m_modules[0].SetDesiredState(states[0]);
  m_modules[1].SetDesiredState(states[1]);
  m_modules[2].SetDesiredState(states[2]);
  m_modules[3].SetDesiredState(states[3]);
}

frc2::CommandPtr Drivebase::GetResetHeadingCmd() {
  return Run(std::function<void()>([this] () { ResetHeading(); }));
}

void Drivebase::ResetHeading() {
  m_gyro.ZeroYaw();
}

// This method will be called once per scheduler run
void Drivebase::Periodic() {}