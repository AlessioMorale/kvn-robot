// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "kvn_status/status_reducer.hpp"

namespace kvn_status
{
namespace
{

using std::chrono::milliseconds;
using std::chrono::seconds;

Condition fault(const std::string & code, TimePoint onset)
{
  return Condition{Severity::kFault, code, onset};
}

Condition warn(const std::string & code, TimePoint onset)
{
  return Condition{Severity::kWarn, code, onset};
}

Condition ok() { return Condition{Severity::kOk, "", TimePoint{0}}; }

// --- reduce_status: each severity -------------------------------------------

TEST(ReduceStatus, NoConditionsIsReady)
{
  EXPECT_EQ(reduce_status({}, OperatingState::kReady), "RDY");
}

TEST(ReduceStatus, OkConditionsAreIgnored)
{
  EXPECT_EQ(reduce_status({ok(), ok()}, OperatingState::kReady), "RDY");
}

TEST(ReduceStatus, DrivingWhenEnabled)
{
  EXPECT_EQ(reduce_status({ok()}, OperatingState::kDriving), "DRV");
}

TEST(ReduceStatus, WarnBeatsOperatingState)
{
  EXPECT_EQ(reduce_status({ok(), warn("TEMP", seconds(1))}, OperatingState::kDriving), "WRN:TEMP");
}

TEST(ReduceStatus, FaultReported)
{
  EXPECT_EQ(reduce_status({fault("MOTOR_L", seconds(1))}, OperatingState::kReady), "FLT:MOTOR_L");
}

TEST(ReduceStatus, FaultBeatsWarnEvenIfWarnIsOlder)
{
  const std::vector<Condition> conditions{warn("TEMP", seconds(1)), fault("BATT", seconds(5))};
  EXPECT_EQ(reduce_status(conditions, OperatingState::kDriving), "FLT:BATT");
}

// --- ties go to the oldest ---------------------------------------------------

TEST(ReduceStatus, OldestFaultWins)
{
  const std::vector<Condition> conditions{
    fault("NEW", seconds(10)), fault("OLDEST", seconds(2)), fault("MID", seconds(5))};
  EXPECT_EQ(reduce_status(conditions, OperatingState::kReady), "FLT:OLDEST");
}

TEST(ReduceStatus, OldestWarnWins)
{
  const std::vector<Condition> conditions{warn("B", seconds(3)), warn("A", seconds(1))};
  EXPECT_EQ(reduce_status(conditions, OperatingState::kReady), "WRN:A");
}

TEST(ReduceStatus, EqualOnsetGoesToFirstListed)
{
  const std::vector<Condition> conditions{fault("FIRST", seconds(1)), fault("SECOND", seconds(1))};
  EXPECT_EQ(reduce_status(conditions, OperatingState::kReady), "FLT:FIRST");
}

// --- format / 15-character limit ----------------------------------------------

TEST(ReduceStatus, TruncatesToFifteenCharacters)
{
  const auto status =
    reduce_status({fault("VERY_LONG_FAULT_CODE", seconds(1))}, OperatingState::kReady);
  EXPECT_EQ(status.size(), kMaxStatusLength);
  EXPECT_EQ(status, "FLT:VERY_LONG_F");
}

TEST(ReduceStatus, ExactlyFifteenCharactersIsKept)
{
  EXPECT_EQ(
    reduce_status({warn("ABCDEFGHIJK", seconds(1))}, OperatingState::kReady), "WRN:ABCDEFGHIJK");
}

TEST(ReduceStatus, NonAsciiCodeIsSanitised)
{
  EXPECT_EQ(reduce_status({warn("T\xC2\xB0\n", seconds(1))}, OperatingState::kReady), "WRN:T???");
}

TEST(FormatStatus, EmptyCodeHasNoColon) { EXPECT_EQ(format_status("RDY", ""), "RDY"); }

// --- InputMonitor: staleness ---------------------------------------------------

TEST(InputMonitor, OkDuringStartupGrace)
{
  InputMonitor monitor("HWMON", seconds(3), seconds(100));
  EXPECT_EQ(monitor.evaluate(seconds(102)).severity, Severity::kOk);
  EXPECT_FALSE(monitor.has_data());
}

TEST(InputMonitor, NeverReceivedBecomesStaleAfterGrace)
{
  InputMonitor monitor("HWMON", seconds(3), seconds(100));
  const auto condition = monitor.evaluate(seconds(104));
  EXPECT_EQ(condition.severity, Severity::kFault);
  EXPECT_EQ(condition.code, "HWMON");
  EXPECT_EQ(condition.onset, seconds(103));
  EXPECT_EQ(reduce_status({condition}, OperatingState::kDriving), "FLT:HWMON");
}

TEST(InputMonitor, StaleAfterUpdatesStop)
{
  InputMonitor monitor("JOY", milliseconds(500), seconds(0));
  monitor.update(Severity::kOk, "", seconds(1));
  EXPECT_EQ(monitor.evaluate(milliseconds(1400)).severity, Severity::kOk);
  const auto condition = monitor.evaluate(milliseconds(1600));
  EXPECT_EQ(condition.severity, Severity::kFault);
  EXPECT_EQ(condition.code, "JOY");
  EXPECT_EQ(condition.onset, milliseconds(1500));
}

TEST(InputMonitor, StaleInputBeatsDrivingAndWarn)
{
  InputMonitor stale("JOY", milliseconds(500), seconds(0));
  InputMonitor warning("BATT", seconds(10), seconds(0));
  warning.update(Severity::kWarn, "BATT", seconds(0));
  const auto now = seconds(2);
  EXPECT_EQ(
    reduce_status({warning.evaluate(now), stale.evaluate(now)}, OperatingState::kDriving),
    "FLT:JOY");
}

TEST(InputMonitor, RecoversWhenUpdatesResume)
{
  InputMonitor monitor("HWMON", seconds(1), seconds(0));
  EXPECT_EQ(monitor.evaluate(seconds(5)).severity, Severity::kFault);
  monitor.update(Severity::kOk, "", seconds(5));
  EXPECT_EQ(monitor.evaluate(seconds(5)).severity, Severity::kOk);
}

TEST(InputMonitor, OnsetKeptWhileConditionUnchanged)
{
  InputMonitor monitor("BATT", seconds(5), seconds(0));
  monitor.update(Severity::kWarn, "BATT", seconds(1));
  monitor.update(Severity::kWarn, "BATT", seconds(2));
  EXPECT_EQ(monitor.evaluate(seconds(2)).onset, seconds(1));
  monitor.update(Severity::kFault, "BATT", seconds(3));
  EXPECT_EQ(monitor.evaluate(seconds(3)).onset, seconds(3));
}

TEST(InputMonitor, OnsetRestartsAfterStalePeriod)
{
  InputMonitor monitor("BATT", seconds(1), seconds(0));
  monitor.update(Severity::kWarn, "BATT", seconds(1));
  monitor.update(Severity::kWarn, "BATT", seconds(5));
  EXPECT_EQ(monitor.evaluate(seconds(5)).onset, seconds(5));
}

TEST(InputMonitor, OldestConditionAcrossInputsWins)
{
  InputMonitor hwmon("HWMON", seconds(10), seconds(0));
  InputMonitor battery("BATT", seconds(10), seconds(0));
  battery.update(Severity::kWarn, "BATT", seconds(1));
  hwmon.update(Severity::kWarn, "HWMON", seconds(2));
  const auto now = seconds(3);
  EXPECT_EQ(
    reduce_status({hwmon.evaluate(now), battery.evaluate(now)}, OperatingState::kReady),
    "WRN:BATT");
}

}  // namespace
}  // namespace kvn_status
