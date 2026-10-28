// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include "kvn_joy_teleop/command_arbiter.hpp"

using kvn_joy_teleop::ArbiterConfig;
using kvn_joy_teleop::ArbiterInputs;
using kvn_joy_teleop::select_source;
using kvn_joy_teleop::Source;

namespace
{
ArbiterInputs all_live()
{
  ArbiterInputs in;
  in.joy_fresh = true;
  in.auto_fresh = true;
  in.autonomy_permitted = true;
  return in;
}
}  // namespace

TEST(CommandArbiter, EnableSwitchOverridesAutonomy)
{
  auto in = all_live();
  in.enable_pressed = true;
  EXPECT_EQ(select_source(in, ArbiterConfig{}), Source::kTeleop);
}

TEST(CommandArbiter, AutonomyDrivesWhenAllowed)
{
  EXPECT_EQ(select_source(all_live(), ArbiterConfig{}), Source::kAuto);
}

TEST(CommandArbiter, NoPermitNoAutonomy)
{
  auto in = all_live();
  in.autonomy_permitted = false;
  EXPECT_EQ(select_source(in, ArbiterConfig{}), Source::kNone);
  ArbiterConfig config;
  config.require_autonomy_permit = false;
  EXPECT_EQ(select_source(in, config), Source::kAuto);
}

TEST(CommandArbiter, LostJoyStopsEverything)
{
  auto in = all_live();
  in.joy_fresh = false;
  in.enable_pressed = true;  // last message had it held, but the link is gone
  EXPECT_EQ(select_source(in, ArbiterConfig{}), Source::kNone);
  ArbiterConfig config;
  config.require_joy_for_auto = false;
  EXPECT_EQ(select_source(in, config), Source::kAuto);
}

TEST(CommandArbiter, StaleAutonomousCommandsStop)
{
  auto in = all_live();
  in.auto_fresh = false;
  EXPECT_EQ(select_source(in, ArbiterConfig{}), Source::kNone);
}
