// Copyright 2026 LiteArm VR teleop authors
//
// Licensed under the BSD-3-Clause license. See LICENSE for details.
//
// The rule that decides when the inverse kinematics has to be damped, on its
// own: it is a comparison, and a comparison is worth a test that needs no robot
// model, no ROS and no solver.

#include <gtest/gtest.h>

#include "litearm_vr_teleop/conditioning_guard.hpp"

namespace litearm_vr_teleop
{
namespace
{

TEST(ConditioningGuard, AsksForTheDampedSolverOnlyBelowTheThreshold)
{
  // The guard is a threshold, not a policy: below it the damped backend is used,
  // at or above it the configured one is. Zero turns it off, which is what a
  // workspace with no damping to fall back on wants.
  EXPECT_TRUE(conditioningNeedsDamping(0.01, 0.05));
  EXPECT_FALSE(conditioningNeedsDamping(0.05, 0.05));
  EXPECT_FALSE(conditioningNeedsDamping(0.2, 0.05));
  EXPECT_FALSE(conditioningNeedsDamping(0.01, 0.0));
  EXPECT_FALSE(conditioningNeedsDamping(0.01, -1.0));
}

}  // namespace
}  // namespace litearm_vr_teleop
