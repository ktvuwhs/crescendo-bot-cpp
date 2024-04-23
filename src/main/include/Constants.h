#pragma once
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include <numbers>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

namespace SwerveModuleConstants {
// Gear ratios found here:
// @{link} https://www.swervedrivespecialties.com/products/mk4i-swerve-module
// See here for example calculating gear ratios
// @{link} https://www.smlease.com/entries/mechanism/gear-train-gear-ratio-torque-and-speed-calculation/
// Drive gear ratio is calculated by taking the MK4i L2 gear ratios (the one we used)
// and swapping out the 14 teeth drive pinion gear on stage 1 with a 16 teeth
// drive pinion gear instead.  The resulting gear ratio is approximately 5.9028.
inline double constexpr kDriveGearRatio = 5.9028;     // (50/16) * (17/27) * (45/15)
inline double constexpr kTurnGearRatio = 150.0 / 7.0; // 12.8

inline unsigned int constexpr kStallLimit = 40;
inline unsigned int constexpr kFreeLimit = 40;
inline double constexpr kDriveSupplyCurrentLimit = 80.0;
inline double constexpr kPDrive = 0.5;
inline double constexpr kDDrive = 0.001;
inline double constexpr kVDrive = 0.0001;
inline double constexpr kPTurn = 0.5;

inline units::meter_t constexpr kWheelbase = 24_in;
inline units::meter_t constexpr kTrackwidth = 24_in;
inline units::meter_t constexpr kWheelDiameter{0.1016};
inline units::meter_t const kCircumference = kWheelDiameter * std::numbers::pi_v<double>;

inline int constexpr kBotRightDrivePort = 3;
inline int constexpr kBotLeftDrivePort = 1;
inline int constexpr kTopRightDrivePort = 2;
inline int constexpr kTopLeftDrivePort = 4;

inline int constexpr kBotRightTurnPort = 8;
inline int constexpr kBotLeftTurnPort = 10;
inline int constexpr kTopRightTurnPort = 6;
inline int constexpr kTopLeftTurnPort = 11;

inline int constexpr kBotRightCancoderPort = 12;
inline int constexpr kBotLeftCancoderPort = 9;
inline int constexpr kTopRightCancoderPort = 11;
inline int constexpr kTopLeftCancoderPort = 10;

inline double constexpr kBotRightMagnetOffset = 0.309326;
inline double constexpr kBotLeftMagnetOffset = -0.107666;
inline double constexpr kTopRightMagnetOffset = -0.155518;
inline double constexpr kTopLeftMagnetOffset = 0.019287;
} // namespace SwerveModuleConstants

namespace DrivebaseConstants {
inline constexpr units::hertz_t kXSlewRate{3/1_s};
inline constexpr units::hertz_t kYSlewRate{3/1_s};
inline constexpr units::hertz_t kRotSlewRate{3/1_s};
inline constexpr double kDeadband = 0.02;
static units::meters_per_second_t constexpr kMaxSpeed = 3.0_mps;
static units::radians_per_second_t constexpr kMaxAngularSpeed = units::radians_per_second_t{std::numbers::pi};
} // namespace DrivebaseConstants

#endif  // #ifndef _CONSTANTS_H_