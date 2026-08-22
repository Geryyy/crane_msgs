#include <gtest/gtest.h>

#include "crane_msgs/msg/collision_scene.hpp"
#include "crane_msgs/msg/payload.hpp"
#include "crane_msgs/msg/supervisor_status.hpp"
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

TEST(CraneMsgsContract, CollisionShapesAndGripPhasesAreFrozen)
{
  EXPECT_EQ(crane_msgs::msg::CollisionScene::SHAPE_BOX, 1U);
  EXPECT_EQ(crane_msgs::msg::CollisionScene::SHAPE_SPHERE, 3U);
  EXPECT_EQ(crane_msgs::srv::PlanGrip::Request::PHASE_DESCEND, 1U);
  EXPECT_EQ(crane_msgs::srv::PlanGrip::Request::PHASE_LIFT, 4U);
}
