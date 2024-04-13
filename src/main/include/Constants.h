#include <numbers>
#include <units/length.h>

namespace SwerveModuleConstants {
  static double constexpr kDriveGearRatio = 5.9028;//8.14;//6.12;//6.75;
  static double constexpr kTurnGearRatio = 150.0 / 7.0; // 12.8
  static unsigned int constexpr kStallLimit = 40;
  static unsigned int constexpr kFreeLimit = 40;
  static double constexpr kDriveSupplyCurrentLimit = 80.0;

  static units::meter_t constexpr kWheelDiameter{0.1016};
  static units::meter_t constexpr kCircumference{kWheelDiameter * std::numbers::pi_v<double>};
}