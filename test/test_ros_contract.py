"""Hardware-free ROS graph tests for the crane_msgs contract."""

import os
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

# Ignore an empty Fast DDS profile path; this test needs no custom profile.
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
    os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)

import rclpy
from crane_msgs.msg import (
    CollisionScene,
    JointPath,
    PayloadEstimate,
    SolverHealth,
    SupervisorStatus,
    SwaySettled,
    VelocityControllerHealth,
)
from crane_msgs.srv import PlanMotion, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

TEST_NODE_NAME = "crane_msgs_ros_contract_test"


@dataclass(frozen=True)
class StreamContract:
    """Expected topic type, QoS, message, and frame."""

    name: str
    type_name: str
    message_type: type
    qos: QoSProfile
    message: object
    frame_id: str | None


def _qos(*, transient_local=False):
    """Build the contract QoS."""
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


def _header(frame_id):
    header = Header()
    header.frame_id = frame_id
    return header


def _message(message_type, frame_id=""):
    """Construct a message with an optional frame."""
    message = message_type()
    message.header = _header(frame_id)
    return message


def _stream_contracts():
    """Return all streamed contracts."""
    return (
        StreamContract(
            "/crane/payload_estimate",
            "crane_msgs/msg/PayloadEstimate",
            PayloadEstimate,
            _qos(transient_local=True),
            _message(PayloadEstimate, "K8_rotator_lower_part"),
            "K8_rotator_lower_part",
        ),
        StreamContract(
            "/crane/mpc/horizon",
            "trajectory_msgs/msg/JointTrajectory",
            JointTrajectory,
            _qos(),
            _message(JointTrajectory),
            "",
        ),
        StreamContract(
            "/crane/reference",
            "trajectory_msgs/msg/JointTrajectory",
            JointTrajectory,
            _qos(transient_local=True),
            _message(JointTrajectory),
            "",
        ),
        StreamContract(
            "/crane/joint_path",
            "crane_msgs/msg/JointPath",
            JointPath,
            _qos(transient_local=True),
            _message(JointPath),
            "",
        ),
        StreamContract(
            "/crane/mpc/solver_health",
            "crane_msgs/msg/SolverHealth",
            SolverHealth,
            _qos(),
            _message(SolverHealth),
            "",
        ),
        StreamContract(
            "/crane/velocity_controller/health",
            "crane_msgs/msg/VelocityControllerHealth",
            VelocityControllerHealth,
            _qos(),
            _message(VelocityControllerHealth),
            "",
        ),
        StreamContract(
            "/crane/supervisor/status",
            "crane_msgs/msg/SupervisorStatus",
            SupervisorStatus,
            _qos(),
            _message(SupervisorStatus),
            "",
        ),
        StreamContract(
            "/crane/sway_settled",
            "crane_msgs/msg/SwaySettled",
            SwaySettled,
            _qos(),
            _message(SwaySettled),
            "",
        ),
        StreamContract(
            "/crane/collision_scene",
            "crane_msgs/msg/CollisionScene",
            CollisionScene,
            _qos(transient_local=True),
            _message(CollisionScene, "K0_mounting_base"),
            "K0_mounting_base",
        ),
    )


def _wait_until(node, predicate, timeout_sec=5.0):
    """Spin until a DDS condition is true or times out."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    assert predicate(), "timed out waiting for ROS graph or message delivery"


@pytest.fixture(scope="module")
def contract_node():
    """Create the shared test node."""
    fastdds_profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    scrubbed_profile = False
    if not fastdds_profile or not Path(fastdds_profile).is_file():
        # Ignore an empty or stale profile path during node initialization.
        os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        scrubbed_profile = True
    rclpy.init(args=None)
    node = Node(TEST_NODE_NAME)
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if scrubbed_profile and fastdds_profile is not None:
            os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = fastdds_profile


def test_streamed_topics_have_exact_graph_qos_types_and_frames(contract_node):
    """Check topic types, QoS, delivery, and frames."""
    contracts = _stream_contracts()
    received = {}
    publishers = []
    subscriptions = []

    for contract in contracts:
        publishers.append(
            contract_node.create_publisher(
                contract.message_type, contract.name, contract.qos
            )
        )
        subscriptions.append(
            contract_node.create_subscription(
                contract.message_type,
                contract.name,
                lambda message, name=contract.name: received.__setitem__(name, message),
                contract.qos,
            )
        )

    for contract in contracts:

        def endpoint_is_present(contract=contract):
            return any(
                endpoint.node_name == TEST_NODE_NAME
                and endpoint.topic_type == contract.type_name
                for endpoint in contract_node.get_publishers_info_by_topic(
                    contract.name
                )
            )

        _wait_until(contract_node, endpoint_is_present)
        endpoints = [
            endpoint
            for endpoint in contract_node.get_publishers_info_by_topic(contract.name)
            if endpoint.node_name == TEST_NODE_NAME
        ]
        assert len(endpoints) == 1
        qos = endpoints[0].qos_profile
        # Some RMWs report graph depth as UNKNOWN/0; verify local depth too.
        assert publishers[contracts.index(contract)].qos_profile.depth == 1
        assert subscriptions[contracts.index(contract)].qos_profile.depth == 1
        if qos.depth:
            assert qos.depth == 1
        assert qos.reliability == contract.qos.reliability
        assert qos.durability == contract.qos.durability
        publishers[contracts.index(contract)].publish(contract.message)

    _wait_until(contract_node, lambda: len(received) == len(contracts))
    for contract in contracts:
        message = received[contract.name]
        assert isinstance(message, contract.message_type)
        assert message.header.frame_id == contract.frame_id

    # Keep entities alive through delivery; the fixture destroys them.
    assert len(publishers) == len(subscriptions) == len(contracts)


@pytest.mark.parametrize(
    ("service_type", "service_name", "type_name"),
    [
        (PlanMotion, "/crane/plan_motion", "crane_msgs/srv/PlanMotion"),
        (SetMode, "/crane/set_mode", "crane_msgs/srv/SetMode"),
    ],
)
def test_service_servers_have_exact_names_and_explicit_failure_defaults(
    contract_node, service_type, service_name, type_name
):
    """Check service names, types, and failure responses."""

    def callback(_request, response):
        response.success = False
        response.message = "mock endpoint: no implementation installed"
        return response

    server = contract_node.create_service(service_type, service_name, callback)
    assert server is not None

    def service_is_present():
        return service_name in dict(contract_node.get_service_names_and_types())

    _wait_until(contract_node, service_is_present)
    names_and_types = dict(contract_node.get_service_names_and_types())
    assert service_name in names_and_types
    assert names_and_types[service_name] == [type_name]

    client = contract_node.create_client(service_type, service_name)
    _wait_until(contract_node, client.service_is_ready)
    future = client.call_async(service_type.Request())
    _wait_until(contract_node, future.done)
    response = future.result()
    assert response.success is False
    assert response.message

    contract_node.destroy_client(client)
    contract_node.destroy_service(server)
