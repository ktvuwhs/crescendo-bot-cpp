// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SwerveModule.h"

#include <string>

// #include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/angle.h>
#include <units/length.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "../include/Constants.h"

using namespace ctre::phoenix6;
using namespace SwerveModuleConstants;
using namespace units;

SwerveModule::SwerveModule(int const drivePort,
                           int const turnPort,
                           int const canCoderPort,
                           double const angleOffset = 0.0,
                           bool const isDriveInverted = false,
                           std::string const &driveBusName = "rio",
                           std::string const &canCoderBusName = "rio")
  : m_driveMotor{drivePort, driveBusName},
    m_turnMotor{turnPort, rev::CANSparkLowLevel::MotorType::kBrushless},
    m_canCoder{canCoderPort, canCoderBusName},
    m_turnEncoder{m_turnMotor.GetEncoder()} {
      ConfigMotors(isDriveInverted);
      ConfigCANcoder(angleOffset);
      ZeroTurnEncoder();
      m_driveMotor.SetPosition(angle::turn_t{0});
}

void SwerveModule::ConfigMotors(bool const isDriveInverted) {
  ConfigDriveMotor(isDriveInverted);
  ConfigTurnMotor(true);
}

void SwerveModule::ConfigDriveMotor(bool const isInverted) {
  configs::TalonFXConfiguration m_driveControllerCfg;
  
  m_driveControllerCfg.CurrentLimits
    .WithSupplyCurrentLimitEnable(true)
    .WithSupplyCurrentLimit(kDriveSupplyCurrentLimit);

  m_driveControllerCfg.Feedback
    .WithSensorToMechanismRatio(kDriveGearRatio / kCircumference.value());

  m_driveControllerCfg.MotorOutput
    .WithInverted(isInverted ? signals::InvertedValue::Clockwise_Positive
      : signals::InvertedValue::CounterClockwise_Positive)
    .WithNeutralMode(signals::NeutralModeValue::Brake);

  m_driveMotor.GetConfigurator().Apply(m_driveControllerCfg);
}

void SwerveModule::ConfigTurnMotor(bool const isInverted) {
  m_turnMotor.RestoreFactoryDefaults();
  m_turnMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_turnMotor.SetInverted(isInverted);
  m_turnMotor.SetSmartCurrentLimit(kStallLimit, kFreeLimit);
  m_turnMotor.BurnFlash();
}

void SwerveModule::ConfigCANcoder(double const angleOffset) {
  configs::CANcoderConfiguration m_cfg;
  m_cfg.MagnetSensor
    .WithAbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf)
    .WithSensorDirection(signals::SensorDirectionValue::CounterClockwise_Positive)
    .WithMagnetOffset(angleOffset);

  m_canCoder.GetConfigurator().Apply(m_cfg);
}

angle::turn_t SwerveModule::GetAbsoluteNumTurns() {
  return m_canCoder.GetAbsolutePosition().GetValue();
}

rev::REVLibError SwerveModule::ZeroTurnEncoder() {
  return m_turnEncoder.SetPosition(GetAbsoluteNumTurns().value() * kTurnGearRatio);
}

angular_velocity::turns_per_second_t SwerveModule::GetDriveVelocity() {
  return m_driveMotor.GetVelocity().GetValue();
}