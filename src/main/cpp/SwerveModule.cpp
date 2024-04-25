// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <string>

// #include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>
#include <rev/SparkPIDController.h>
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
                           std::string const &driveBusName,
                           std::string const &canCoderBusName)
  : m_driveMotor{drivePort, driveBusName},
    m_turnMotor{turnPort, rev::CANSparkLowLevel::MotorType::kBrushless},
    m_canCoder{canCoderPort, canCoderBusName},
    m_turnEncoder{m_turnMotor.GetEncoder()},
    m_turnPIDController{m_turnMotor.GetPIDController()} {}

void SwerveModule::ApplyConfigs(configs::TalonFXConfiguration const &motorCfg,
                                configs::MagnetSensorConfigs &magnetCfg) {
  m_driveMotor.GetConfigurator().Apply(motorCfg);
  m_driveMotor.SetPosition(units::turn_t{0});
  m_canCoder.GetConfigurator().Apply(magnetCfg);
  ConfigTurnMotor(true);
  ZeroTurnEncoder();
}

void SwerveModule::ConfigTurnMotor(bool const isInverted) {
  m_turnMotor.RestoreFactoryDefaults();
  m_turnMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  m_turnMotor.SetInverted(isInverted);
  m_turnMotor.SetSmartCurrentLimit(SMConst::kStallLimit, SMConst::kFreeLimit);
  // (motor_turn) * (wheel_turn / motor_turn) = wheel_turn
  m_turnEncoder.SetPositionConversionFactor(1/SMConst::kTurnGearRatio);
  // TODO: update velocity conversion factor after it has been tested on the robot
  m_turnEncoder.SetVelocityConversionFactor(1/SMConst::kTurnGearRatio);
  m_turnPIDController.SetP(SMConst::kPTurn);
  m_turnMotor.BurnFlash();
}

rev::REVLibError SwerveModule::ZeroTurnEncoder() {
  return m_turnEncoder.SetPosition(GetAbsoluteNumTurns().value());
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return frc::SwerveModulePosition{
    m_driveMotor.GetPosition().GetValue().value() * SMConst::kCircumference,
    frc::Rotation2d{units::turn_t{m_turnEncoder.GetPosition()}}};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return frc::SwerveModuleState{
    units::meters_per_second_t{
      m_driveMotor.GetVelocity().GetValue().value() * SMConst::kCircumference.value()},
    frc::Rotation2d{units::turn_t{m_turnEncoder.GetPosition()}}};
}

units::turn_t SwerveModule::GetAbsoluteNumTurns() {
  return m_canCoder.GetAbsolutePosition().GetValue();
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState const &desiredState) {
  frc::Rotation2d currAngle = frc::Rotation2d{units::turn_t{m_turnEncoder.GetPosition()}};
  frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(desiredState, currAngle);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  // Note that both optimizedState.angle and turnEncoderRotation are
  // frc::Rotation2d objects, and that the "minus" (-) operator for that
  // class has been overridden.
  optimizedState.speed *= (optimizedState.angle - currAngle).Cos();

  m_driveMotor.SetControl(
    m_VelDutyCycle.WithVelocity(
      // (m/s) * (wheel_turn / m) * (motor_turn / wheel_turn)
      units::turns_per_second_t{optimizedState.speed.value() / SMConst::kCircumference.value()}));
  // (wheel_turns) * (motor_turn / wheel_turn) = motor_turn
  m_turnPIDController.SetReference(
    units::turn_t{optimizedState.angle.Radians()}.value(),
    rev::CANSparkLowLevel::ControlType::kPosition);
}

} // namespace util