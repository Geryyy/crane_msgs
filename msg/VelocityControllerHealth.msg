# What the inner velocity loop found about itself on one control cycle.
#
# `crane_velocity_controller` is the only element in the stack that knows which
# axes the active tool has an identified valve map for, so it is the only one
# that can report a missing calibration. This is how that report leaves the
# controller manager (wiki/implementation/ros2_interfaces.md 4, 6).
#
# `fault` is a crane_msgs/SupervisorStatus FAULT_* constant and is deliberately
# not renumbered here: the supervisor merges the code rather than translating it,
# and an operator panel renders one numbering. The three the loop can raise are
# FAULT_STATE_HEALTH, FAULT_REFERENCE_STALE and FAULT_NOT_COMMISSIONED.
#
# The two arrays are parallel and fixed at the six actuated axes, which are a
# property of the machine (ROS 2 Interfaces 3.2). `joint_names` is carried rather
# than assumed because the sixth valve channel drives a different joint per tool,
# and because a panel that says "gripper" is worth more than one that says a
# count. `feedforward_applied` is false on an axis that ran PI only.
std_msgs/Header header
uint8 fault
string[6] joint_names
bool[6] feedforward_applied
