#include <numbers>
#include <units/length.h>

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

inline units::meter_t constexpr kWheelDiameter{0.1016};
inline units::meter_t constexpr kCircumference{kWheelDiameter * std::numbers::pi_v<double>};
} // SwerveModuleConstants