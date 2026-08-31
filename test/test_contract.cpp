#include <gtest/gtest.h>

#include "crane_msgs/msg/collision_scene.hpp"
#include "crane_msgs/msg/payload.hpp"
#include "crane_msgs/msg/solver_health.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
#include "crane_msgs/msg/sway_settled.hpp"
#include "crane_msgs/srv/plan_grip.hpp"

TEST(CraneMsgsContract, PayloadShapesAreFrozen)
{
  EXPECT_EQ(crane_msgs::msg::Payload::SHAPE_NONE, 0U);
  EXPECT_EQ(crane_msgs::msg::Payload::SHAPE_BOX, 1U);
  EXPECT_EQ(crane_msgs::msg::Payload::SHAPE_CYLINDER, 2U);
}

TEST(CraneMsgsContract, SupervisorModesAndFaultsAreFrozen)
{
  EXPECT_EQ(crane_msgs::msg::SupervisorStatus::MODE_IDLE, 0U);
  EXPECT_EQ(crane_msgs::msg::SupervisorStatus::MODE_MPC, 3U);
  EXPECT_EQ(crane_msgs::msg::SupervisorStatus::FAULT_NONE, 0U);
  EXPECT_EQ(crane_msgs::msg::SupervisorStatus::FAULT_INTERLOCK, 8U);
  EXPECT_EQ(crane_msgs::msg::SupervisorStatus::FAULT_NOT_COMMISSIONED, 9U);
}

TEST(CraneMsgsContract, SwaySettledStatesAreFrozenAndUnknownIsZero)
{
  EXPECT_EQ(crane_msgs::msg::SwaySettled::SETTLED_UNKNOWN, 0U);
  EXPECT_EQ(crane_msgs::msg::SwaySettled::SETTLED_NO, 1U);
  EXPECT_EQ(crane_msgs::msg::SwaySettled::SETTLED_YES, 2U);

  // Zero means unknown, so an unset value cannot mean settled.
  const crane_msgs::msg::SwaySettled fresh;
  EXPECT_EQ(fresh.settled, crane_msgs::msg::SwaySettled::SETTLED_UNKNOWN);
  EXPECT_EQ(fresh.velocity.size(), 2U);
}

TEST(CraneMsgsContract, SolverHealthCarriesSupervisorFaultCodesUnrenumbered)
{
  // SolverHealth reuses SupervisorStatus fault numbering without translation.
  crane_msgs::msg::SolverHealth health;
  health.fault = crane_msgs::msg::SupervisorStatus::FAULT_SOLVER;
  EXPECT_EQ(health.fault, 3U);
  EXPECT_EQ(crane_msgs::msg::SupervisorStatus::FAULT_SOLVER, 3U);

  // Zero means unknown, preventing an unset result from appearing converged.
  const crane_msgs::msg::SolverHealth fresh;
  EXPECT_EQ(fresh.outcome, crane_msgs::msg::SolverHealth::SOLVE_UNKNOWN);
  EXPECT_EQ(crane_msgs::msg::SolverHealth::SOLVE_UNKNOWN, 0U);
  EXPECT_EQ(fresh.constraint_violation.size(), 4U);
  EXPECT_EQ(fresh.cost_term.size(), 8U);
  EXPECT_EQ(fresh.joint_names.size(), 6U);
}

TEST(CraneMsgsContract, CollisionShapesAndGripPhasesAreFrozen)
{
  EXPECT_EQ(crane_msgs::msg::CollisionScene::SHAPE_BOX, 1U);
  EXPECT_EQ(crane_msgs::msg::CollisionScene::SHAPE_SPHERE, 3U);
  EXPECT_EQ(crane_msgs::srv::PlanGrip::Request::PHASE_DESCEND, 1U);
  EXPECT_EQ(crane_msgs::srv::PlanGrip::Request::PHASE_LIFT, 4U);
}
