"""Exact source-schema contracts for every crane_msgs interface."""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _schema_lines(relative_path):
    """Return non-empty, comment-free schema lines in source order."""
    path = PACKAGE_ROOT / relative_path
    lines = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            lines.append(line)
    return lines


EXPECTED_SCHEMAS = {
    "msg/CollisionPrimitive.msg": [
        "string id",
        "uint8 shape",
        "geometry_msgs/Pose pose",
        "geometry_msgs/Vector3 dimensions",
        "bool structural",
    ],
    "msg/CollisionScene.msg": [
        "uint8 SHAPE_BOX=1",
        "uint8 SHAPE_CYLINDER=2",
        "uint8 SHAPE_SPHERE=3",
        "std_msgs/Header header",
        "crane_msgs/CollisionPrimitive[] primitives",
    ],
    "msg/Payload.msg": [
        "uint8 SHAPE_NONE=0",
        "uint8 SHAPE_BOX=1",
        "uint8 SHAPE_CYLINDER=2",
        "uint8 shape",
        "geometry_msgs/Vector3 dimensions",
        "float64 mass",
        "geometry_msgs/Point com",
    ],
    "msg/PayloadEstimate.msg": [
        "std_msgs/Header header",
        "float64 mass",
        "float64 m_r_x",
        "float64 m_r_y",
        "float64 r_z",
        "bool valid",
    ],
    "msg/PendulumState.msg": [
        "std_msgs/Header header",
        "float64[2] position",
        "float64[2] velocity",
        "float64[4] position_covariance",
        "float64[4] velocity_covariance",
        "bool valid",
        "string status",
    ],
    "msg/SolverHealth.msg": [
        "uint8 SOLVE_UNKNOWN=0",
        "uint8 SOLVE_CONVERGED=1",
        "uint8 SOLVE_BUDGET_EXCEEDED=2",
        "uint8 SOLVE_FAILED=3",
        "uint8 CONSTRAINT_SWAY=0",
        "uint8 CONSTRAINT_SWAY_RATE=1",
        "uint8 CONSTRAINT_CYLINDER_FORCE=2",
        "uint8 CONSTRAINT_PUMP_FLOW=3",
        "uint8 COST_ACTUATED_POSITION=0",
        "uint8 COST_ACTUATED_VELOCITY=1",
        "uint8 COST_SWAY=2",
        "uint8 COST_SWAY_RATE=3",
        "uint8 COST_EFFORT=4",
        "uint8 COST_INPUT=5",
        "uint8 COST_TERMINAL=6",
        "uint8 COST_SLACK=7",
        "std_msgs/Header header",
        "uint8 outcome",
        "uint8 fault",
        "int32 status",
        "string status_word",
        "int32 iterations",
        "int32 qp_status",
        "int32 qp_iterations",
        "float64 solve_time",
        "float64 solve_budget",
        "bool applied_previous_solution",
        "string[6] joint_names",
        "bool used_slack",
        "float64[4] constraint_violation",
        "float64 slack_penalty",
        "float64[8] cost_term",
        "string message",
    ],
    "msg/SupervisorStatus.msg": [
        "uint8 MODE_IDLE=0",
        "uint8 MODE_MANUAL=1",
        "uint8 MODE_FOLLOW=2",
        "uint8 MODE_MPC=3",
        "uint8 FAULT_NONE=0",
        "uint8 FAULT_TRACKING=1",
        "uint8 FAULT_WORKING_CELL=2",
        "uint8 FAULT_SOLVER=3",
        "uint8 FAULT_SWAY=4",
        "uint8 FAULT_STATE_HEALTH=5",
        "uint8 FAULT_REFERENCE_STALE=6",
        "uint8 FAULT_ESTOP=7",
        "uint8 FAULT_INTERLOCK=8",
        "uint8 FAULT_NOT_COMMISSIONED=9",
        "std_msgs/Header header",
        "uint8 mode",
        "uint8 fault",
        "float64 tracking_error",
        "bool inside_working_cell",
        "bool deadman_held",
        "string message",
    ],
    "msg/SwaySettled.msg": [
        "uint8 SETTLED_UNKNOWN=0",
        "uint8 SETTLED_NO=1",
        "uint8 SETTLED_YES=2",
        "std_msgs/Header header",
        "uint8 settled",
        "float64[2] velocity",
        "string message",
    ],
    "msg/VelocityControllerHealth.msg": [
        "std_msgs/Header header",
        "uint8 fault",
        "string[6] joint_names",
        "bool[6] feedforward_applied",
    ],
    "srv/PlanGrip.srv": [
        "uint8 PHASE_DESCEND=1",
        "uint8 PHASE_CLOSE=2",
        "uint8 PHASE_OPEN=3",
        "uint8 PHASE_LIFT=4",
        "uint8 phase",
        "geometry_msgs/PoseStamped goal",
        "crane_msgs/Payload payload",
        "float64 speed_scale 1.0",
        "---",
        "bool success",
        "string message",
        "trajectory_msgs/JointTrajectory trajectory",
    ],
    "srv/PlanMotion.srv": [
        "geometry_msgs/PoseStamped goal",
        "crane_msgs/Payload payload",
        "bool avoid_collisions true",
        "float64 speed_scale 1.0",
        "---",
        "bool success",
        "string message",
        "trajectory_msgs/JointTrajectory trajectory",
        "geometry_msgs/PoseStamped[] tcp_path",
    ],
    "srv/SetMode.srv": [
        "uint8 mode",
        "---",
        "bool success",
        "string message",
        "uint8 active_mode",
    ],
}


def test_all_interfaces_have_exact_frozen_schema():
    """Fail on any field, order, type, constant, or default drift."""
    assert set(EXPECTED_SCHEMAS) == {
        *[
            f"msg/{name}.msg"
            for name in (
                "CollisionPrimitive",
                "CollisionScene",
                "Payload",
                "PayloadEstimate",
                "PendulumState",
                "SolverHealth",
                "SupervisorStatus",
                "SwaySettled",
                "VelocityControllerHealth",
            )
        ],
        *[f"srv/{name}.srv" for name in ("PlanGrip", "PlanMotion", "SetMode")],
    }
    for relative_path, expected in EXPECTED_SCHEMAS.items():
        assert _schema_lines(relative_path) == expected, relative_path
