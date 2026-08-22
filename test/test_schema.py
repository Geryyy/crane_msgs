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
                "SupervisorStatus",
            )
        ],
        *[f"srv/{name}.srv" for name in ("PlanGrip", "PlanMotion", "SetMode")],
    }
    for relative_path, expected in EXPECTED_SCHEMAS.items():
        assert _schema_lines(relative_path) == expected, relative_path
