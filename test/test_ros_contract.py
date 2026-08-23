"""Hardware-free ROS graph tests for the crane_msgs contract.

These tests intentionally create only in-process publishers and service
servers.  They do not start planner, controller, supervisor, or hardware
nodes.  The DDS graph and delivered messages are the contract under test.
"""

import os
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

# Some developer shells export this variable as an empty string. Fast DDS
# treats an empty-but-present path as a profile filename and logs a realpath
# failure during participant creation. The contract test does not require a
# custom transport profile, so isolate it from that ambient shell setting.
if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
    os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)

import rclpy
from crane_msgs.msg import (
    CollisionScene,
    PayloadEstimate,
    PendulumState,
    SupervisorStatus,
    VelocityControllerHealth,
)
from crane_msgs.srv import PlanGrip, PlanMotion, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

TEST_NODE_NAME = "crane_msgs_ros_contract_test"


@dataclass(frozen=True)
class StreamContract:
    """One streamed endpoint and the message/frame expected on it."""

    name: str
    type_name: str
    message_type: type
    qos: QoSProfile
    message: object
    frame_id: str | None


def _qos(*, transient_local=False):
    """Build the exact reliable depth-one contract QoS."""
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
    """Construct a message without relying on generated kwarg support."""
    message = message_type()
    message.header = _header(frame_id)
    return message


def _stream_contracts():
    """Return every new streamed crane_msgs/trajectory contract."""
    return (
        StreamContract(
            "/crane/pendulum_state",
            "crane_msgs/msg/PendulumState",
            PendulumState,
            _qos(),
            _message(PendulumState),
            "",
        ),
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
            "/crane/collision_scene",
            "crane_msgs/msg/CollisionScene",
            CollisionScene,
            _qos(transient_local=True),
            _message(CollisionScene, "K0_mounting_base"),
            "K0_mounting_base",
        ),
    )


def _wait_until(node, predicate, timeout_sec=5.0):
    """Spin a test node until a DDS condition becomes true."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    assert predicate(), "timed out waiting for ROS graph or message delivery"


@pytest.fixture(scope="module")
def contract_node():
    """Create one ROS node for all in-process graph assertions."""
    fastdds_profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    scrubbed_profile = False
    if not fastdds_profile or not Path(fastdds_profile).is_file():
        # An empty/stale profile variable makes Fast DDS emit a realpath
        # error before discovery.  It is an environment defect, not part of
        # this hardware-free contract test, so isolate it during init.
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
    """Assert graph endpoint metadata and delivered frame semantics."""
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
        # Some Humble RMWs report endpoint history/depth as UNKNOWN/0 even
        # though the created entities retain the requested depth.  Check the
        # graph's transport policies and both local entity profiles, while
        # retaining a strict depth check when the RMW exposes it.
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

    # Keep the local entities alive through delivery and make the test's
    # lifetime explicit; destruction is handled by the node fixture.
    assert len(publishers) == len(subscriptions) == len(contracts)


@pytest.mark.parametrize(
    ("service_type", "service_name", "type_name"),
    [
        (PlanMotion, "/crane/plan_motion", "crane_msgs/srv/PlanMotion"),
        (PlanGrip, "/crane/plan_grip", "crane_msgs/srv/PlanGrip"),
        (SetMode, "/crane/set_mode", "crane_msgs/srv/SetMode"),
    ],
)
def test_service_servers_have_exact_names_and_explicit_failure_defaults(
    contract_node, service_type, service_name, type_name
):
    """Call each mock server and require a useful failure response."""

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
