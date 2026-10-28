// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

// Pure battery return-to-home logic, no ROS dependency.

namespace kvn_orchestrator
{

struct BatteryReturnParams
{
  /// Pack voltage at or below which the rover returns home whatever the distance.
  double critical_voltage{12.4};
  /// Number of cells in series.
  int cells{4};
  /// Usable pack energy between empty (first curve point) and full (last curve point).
  double capacity_wh{40.0};
  /// Cell voltage at evenly spaced states of charge, from 0 % to 100 %.
  std::vector<double> cell_voltage_curve{
    3.00, 3.30, 3.45, 3.55, 3.62, 3.68, 3.75, 3.83, 3.92, 4.05, 4.20};
  /// Energy the rover uses per metre driven, measured or estimated from motor current.
  double wh_per_m{0.02};
  /// Driven distance over straight-line distance to home (obstacles, path detours).
  double path_factor{1.5};
  /// Energy left over on arrival at home.
  double reserve_wh{4.0};
};

enum class ReturnReason
{
  kNone,
  /// Pack voltage at or below the fixed threshold (or unreadable).
  kCriticalVoltage,
  /// Remaining energy at or below what the trip home needs, reserve included.
  kEnergy,
};

struct ReturnCheck
{
  ReturnReason reason{ReturnReason::kNone};
  double state_of_charge{0.0};
  double remaining_wh{0.0};
  double required_wh{0.0};

  bool required() const {return reason != ReturnReason::kNone;}
};

/// State of charge in [0, 1] from a pack voltage, by linear interpolation of the cell curve.
inline double state_of_charge_from_voltage(
  double pack_voltage, int cells, const std::vector<double> & cell_curve)
{
  if (cells <= 0 || cell_curve.size() < 2 || !std::isfinite(pack_voltage)) {
    return 0.0;
  }
  const double cell = pack_voltage / cells;
  if (cell <= cell_curve.front()) {
    return 0.0;
  }
  if (cell >= cell_curve.back()) {
    return 1.0;
  }
  const auto upper = std::upper_bound(cell_curve.begin(), cell_curve.end(), cell);
  const auto i = static_cast<std::size_t>(upper - cell_curve.begin());
  const double v0 = cell_curve[i - 1];
  const double v1 = cell_curve[i];
  const double step = 1.0 / static_cast<double>(cell_curve.size() - 1);
  const double fraction = v1 > v0 ? (cell - v0) / (v1 - v0) : 0.0;
  return (static_cast<double>(i - 1) + fraction) * step;
}

/// Energy needed to drive \p distance_m (straight line) home, reserve included.
inline double return_energy_wh(double distance_m, const BatteryReturnParams & params)
{
  return std::max(0.0, distance_m) * params.path_factor * params.wh_per_m + params.reserve_wh;
}

/// Whether the rover must return home now.
///
/// \p percentage is BatteryState.percentage in [0, 1], or NaN when the driver does not estimate
/// it; the voltage curve is used then. Voltage sags under load, so the curve underestimates the
/// charge while driving: the check errs on the early side.
inline ReturnCheck check_battery_return(
  double pack_voltage, double percentage, double distance_home_m,
  const BatteryReturnParams & params)
{
  ReturnCheck check;
  check.state_of_charge = std::isfinite(percentage) ?
    std::clamp(percentage, 0.0, 1.0) :
    state_of_charge_from_voltage(pack_voltage, params.cells, params.cell_voltage_curve);
  check.remaining_wh = check.state_of_charge * params.capacity_wh;
  check.required_wh = return_energy_wh(distance_home_m, params);
  if (!std::isfinite(pack_voltage) || pack_voltage <= params.critical_voltage) {
    check.reason = ReturnReason::kCriticalVoltage;
  } else if (check.remaining_wh <= check.required_wh) {
    check.reason = ReturnReason::kEnergy;
  }
  return check;
}

}  // namespace kvn_orchestrator
