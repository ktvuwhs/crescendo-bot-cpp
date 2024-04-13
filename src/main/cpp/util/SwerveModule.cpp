// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/SwerveModule.h"

SwerveModule::SwerveModule(int const drivePort,
  std::string const driveBusName,
  int const canCoderPort,
  std::string const canCoderBusName,
  int const turnPort,
  double const angleOffset,
  bool const isDriveInverted)
  : m_driveMotor{drivePort, driveBusName},
    m_canCoder{canCoderPort, canCoderBusName},
    m_turnMotor{turnPort, rev::CANSparkLowLevel::MotorType::kBrushless},
    m_turnEncoder{m_turnMotor.GetEncoder()} {
      ConfigMotors(isDriveInverted);
      ConfigCANcoder(angleOffset);
      ZeroTurnMotor();
      ZeroDriveMotor();
}

void SwerveModule::ZeroDriveMotor() {
  m_driveMotor.SetPosition(angle::turn_t{0});
}

void SwerveModule::ZeroTurnMotor() {
  m_turnEncoder.SetPosition(GetAbsoluteNumTurns().value() * kTurnGearRatio);
}

void SwerveModule::ConfigMotors(bool const isDriveInverted) {
  ConfigDriveMotor(isDriveInverted);
  ConfigTurnMotor(true);
}

void SwerveModule::ConfigDriveMotor(bool const isInverted) {
  configs::TalonFXConfiguration m_driveControllerCfg;
  m_driveControllerCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
  m_driveControllerCfg.CurrentLimits.SupplyCurrentLimit = kDriveSupplyCurrentLimit;
  m_driveControllerCfg.Feedback.SensorToMechanismRatio = kDriveGearRatio / kCircumference.value();
  m_driveControllerCfg.MotorOutput.Inverted = isInverted ?
    signals::InvertedValue::Clockwise_Positive
    : signals::InvertedValue::CounterClockwise_Positive;
    m_driveControllerCfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;
  
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
  m_cfg.MagnetSensor.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
  m_cfg.MagnetSensor.SensorDirection = signals::SensorDirectionValue::CounterClockwise_Positive;
  m_cfg.MagnetSensor.MagnetOffset = angleOffset;

  m_canCoder.GetConfigurator().Apply(m_cfg);
}

angle::turn_t SwerveModule::GetAbsoluteNumTurns() {
  return m_canCoder.GetAbsolutePosition().GetValue();
}

angular_velocity::turns_per_second_t SwerveModule::GetDriveVelocity() {
  return m_driveMotor.GetVelocity().GetValue();
}