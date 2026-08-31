#include <gtest/gtest.h>

#include "crane_msgs/msg/collision_scene.hpp"
#include "crane_msgs/msg/payload.hpp"
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

  // Zero is the *unknown* state and not the settled one, which is the whole
  // three-valued design on the wire: a default-constructed message, a field a
  // producer forgot to fill and a consumer that read the wrong byte all say "not
  // answerable" rather than "the load is hanging still".
  const crane_msgs::msg::SwaySettled fresh;
  EXPECT_EQ(fresh.settled, crane_msgs::msg::SwaySettled::SETTLED_UNKNOWN);
  EXPECT_EQ(fresh.velocity.size(), 2U);
}

TEST(CraneMsgsContract, CollisionShapesAndGripPhasesAreFrozen)
{
  EXPECT_EQ(crane_msgs::msg::CollisionScene::SHAPE_BOX, 1U);
  EXPECT_EQ(crane_msgs::msg::CollisionScene::SHAPE_SPHERE, 3U);
  EXPECT_EQ(crane_msgs::srv::PlanGrip::Request::PHASE_DESCEND, 1U);
  EXPECT_EQ(crane_msgs::srv::PlanGrip::Request::PHASE_LIFT, 4U);
}
