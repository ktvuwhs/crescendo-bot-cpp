// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <string>

// #include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

namespace util {

namespace SMConst = SwerveModuleConstants;
namespace configs = ctre::phoenix6::configs;
namespace signals = ctre::phoenix6::signals;

SwerveModule::SwerveModule(int const drivePort,
                           int const turnPort,
                           int const canCoderPort,
                           double const angleOffset,
                           bool const isDriveInverted,  // Deprecated for removal
                           std::string const &driveBusName,
                           std::string const &canCoderBusName)
  : m_driveMotor{drivePort, driveBusName},
    m_turnMotor{turnPort, rev::CANSparkLowLevel::MotorType::kBrushless},
    m_canCoder{canCoderPort, canCoderBusName},
    m_turnEncoder{m_turnMotor.GetEncoder()} {
      ConfigTurnMotor(true);
      /** Deprecated for removal once Drivebase.cpp is implemented */
      ConfigMotors(isDriveInverted);
      ConfigCANcoder(angleOffset);
      ZeroTurnEncoder();
      /** ******************************************************** */
}

void SwerveModule::ApplyConfigs(configs::TalonFXConfiguration const &motorCfg,
                                configs::MagnetSensorConfigs &magnetCfg,
                                double const angleOffset) {
  m_driveMotor.GetConfigurator().Apply(motorCfg);
  m_driveMotor.SetPosition(units::turn_t{0});
  m_canCoder.GetConfigurator().Apply(magnetCfg.WithMagnetOffset(angleOffset));
  ZeroTurnEncoder();
}

void SwerveModule::ConfigMotors(bool const isDriveInverted) {
  ConfigDriveMotor(isDriveInverted);
  ConfigTurnMotor(true);
}

void SwerveModule::ConfigDriveMotor(bool const isInverted) {
  configs::TalonFXConfiguration m_driveControllerCfg;
  
  m_driveControllerCfg.CurrentLimits
    .WithSupplyCurrentLimitEnable(true)
    .WithSupplyCurrentLimit(SMConst::kDriveSupplyCurrentLimit);

  m_driveControllerCfg.Feedback
    .WithSensorToMechanismRatio(SMConst::kDriveGearRatio / SMConst::kCircumference.value());

  m_driveControllerCfg.MotorOutput
    .WithInverted(isInverted ? signals::InvertedValue::Clockwise_Positive
      : signals::InvertedValue::CounterClockwise_Positive)
    .WithNeutralMode(signals::NeutralModeValue::Brake);

  m_driveMotor.GetConfigurator().Apply(m_driveControllerCfg);
  m_driveMotor.SetPosition(units::turn_t{0});
}

void SwerveModule::ConfigTurnMotor(bool const isInverted) {
  m_turnMotor.RestoreFactoryDefaults();
  m_turnMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_turnMotor.SetInverted(isInverted);
  m_turnMotor.SetSmartCurrentLimit(SMConst::kStallLimit, SMConst::kFreeLimit);
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

units::turn_t SwerveModule::GetAbsoluteNumTurns() {
  return m_canCoder.GetAbsolutePosition().GetValue();
}

rev::REVLibError SwerveModule::ZeroTurnEncoder() {
  return m_turnEncoder.SetPosition(GetAbsoluteNumTurns().value() * SMConst::kTurnGearRatio);
}

units::turns_per_second_t SwerveModule::GetDriveVelocity() {
  return m_driveMotor.GetVelocity().GetValue();
}

} // namespace util