# crane_msgs

Frozen cross-layer contracts for the new concrete-block crane architecture.
The definitions are governed by
[`wiki/implementation/ros2_interfaces.md`](../../wiki/implementation/ros2_interfaces.md)
§1 and §6. They use SI units and the absolute `/crane/...` namespace. Generated
code is build output and is not committed.

`epsilon_crane_msgs` remains the machine-telemetry boundary (`Hydraulics`,
`RemoteCtrlStates`, and `CraneData`). The existing
`concrete_block_world_model_interfaces` and
`concrete_block_assembly_interfaces` packages remain the CBS domain
boundaries. Their types are intentionally not copied into this package.

Public fields, field order, constants, defaults, QoS, and frame semantics are
frozen before algorithm packages depend on them. The schema test compares all
nine source definitions, including every field and constant. Breaking a
contract requires updating the wiki and this package in the same change.

## Topic contract

The following is the settled graph contract. “Reliable” and “best effort” are
DDS reliability policies; “transient-local” is the latched configuration
policy. Every listed depth is `keep last`.

| Absolute topic | Type | Publisher → subscriber | Rate | QoS | Frame semantics |
|---|---|---|---:|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` → all | 100 Hz | reliable, depth 1 | no frame field |
| `/crane/pendulum_state` | `crane_msgs/PendulumState` | `pendulum_state_broadcaster` → MPC, supervisor | 100 Hz | reliable, depth 1 | joint space; `header.frame_id` empty |
| `/crane/hydraulics` | `epsilon_crane_msgs/Hydraulics` | `sensor_data_broadcaster` → estimator, diagnostics | 100 Hz | best effort, depth 1 | no geometric frame |
| `/crane/payload_estimate` | `crane_msgs/PayloadEstimate` | `payload_estimator` → planner, MPC, world model | 10 Hz | reliable, transient-local | `K8_rotator_lower_part`; `m_r_x/m_r_y` are in K8 |
| `/crane/mpc/horizon` | `trajectory_msgs/JointTrajectory` | `crane_mpc` → `crane_velocity_controller` | 25 Hz | reliable, depth 1 | joint space; `header.frame_id` empty |
| `/crane/reference` | `trajectory_msgs/JointTrajectory` | action frontend/planner adapter → MPC, supervisor | per motion | reliable, transient-local | joint space; `header.frame_id` empty |
| `/crane/supervisor/status` | `crane_msgs/SupervisorStatus` | supervisor → task, operator | 20 Hz | reliable, depth 1 | status data; `header.frame_id` empty |
| `/crane/remote_ctrl_states` | `epsilon_crane_msgs/RemoteCtrlStates` | GPIO controller → supervisor, task | 20 Hz | reliable, depth 1 | no geometric frame |
| `/crane/collision_scene` | `crane_msgs/CollisionScene` | CBS world model → planner | on change | reliable, transient-local | geometric data in `K0_mounting_base` |
| `/cbs/block_world_model` | `concrete_block_world_model_interfaces/BlockArray` | CBS world model → task, planner | on change | reliable, transient-local | geometric data in `world` |

`/joint_states` and `/crane/pendulum_state` are reliable despite being sensor
data because they feed control. QoS is selected by the consumer's safety and
timing role, not by the fact that the producer is a sensor.

`/crane/mpc/horizon` is the only topic that moves the crane. A trajectory
follower is an alternative producer of the same type on that same topic, not
a second command path. The velocity controller reads hydraulic pressures from
its claimed state interfaces, never from `/crane/hydraulics`.

### Time and frames

- A streamed message with a header sets `header.stamp`.
- A `JointTrajectory` header stamp is the absolute time its first point is
  valid, not the publication time.
- Joint-space `JointTrajectory`, `PendulumState`, and status data have no
  geometric expression frame; their `header.frame_id` is the empty string.
- Geometric data must name its frame. Planning geometry is in
  `K0_mounting_base`; task and perception geometry is in `world`.
- The assembly planner performs the only `world` → `K0_mounting_base`
  conversion when handing goals to `/crane/plan_motion`. The world model
  publishes `/crane/collision_scene` already in `K0_mounting_base`; no
  downstream consumer converts it again.
- `Payload.com` is expressed in `K8`; payload-estimate moment fields are also
  expressed in K8. `CollisionPrimitive.pose` and all primitives in a
  `CollisionScene` use the scene header frame.

## Service contract

All new service responses contain `bool success` and `string message`. Failure
must set `success=false` and explain itself; an empty result is never a
successful result.

| Absolute service | Type | Server | Purpose |
|---|---|---|---|
| `/crane/plan_motion` | `crane_msgs/PlanMotion` | `crane_planner` | native path and timing entry point |
| `/crane/plan_grip` | `crane_msgs/PlanGrip` | `crane_planner` | descend, close/open, and lift primitives |
| `/crane/set_mode` | `crane_msgs/SetMode` | `crane_supervisor` | request a control mode |
| `/crane/clear_fault` | `std_srvs/Trigger` | `crane_supervisor` | acknowledge and clear a latched fault |
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | ros2_control | serialized controller ownership; requests go through the supervisor |
| `/cbs/world_model/*` | existing CBS interfaces | CBS world model | carried over unchanged |
| `/cbs/wall_plan/get_next_assembly_task` | `concrete_block_assembly_interfaces/GetNextAssemblyTask` | assembly planner | carried over unchanged |

`PlanMotion.goal` and `PlanGrip.goal` are `PoseStamped` geometry. Callers hold
task/perception goals in `world`; the assembly boundary converts a motion goal
to `K0_mounting_base` before calling the native planner. `PlanMotion.tcp_path`
is visualization-only and is not a command stream.

## Frozen schemas

The source files are the normative field order and defaults:

| Definition | Purpose |
|---|---|
| `msg/PendulumState.msg` | passive tip/tilt position, velocity, covariance, and health |
| `msg/PayloadEstimate.msg` | mass and K8 payload moment estimate |
| `msg/SupervisorStatus.msg` | mode, fault, tracking, working-cell, deadman state |
| `msg/Payload.msg` | payload-agnostic box/cylinder description |
| `msg/CollisionScene.msg` | framed obstacle collection |
| `msg/CollisionPrimitive.msg` | one obstacle shape, pose, dimensions, provenance |
| `srv/PlanMotion.srv` | native motion planning request and trajectory response |
| `srv/PlanGrip.srv` | grip-phase planning request and trajectory response |
| `srv/SetMode.srv` | supervisor mode request and active-mode response |

`Payload.shape` uses `SHAPE_NONE=0`, `SHAPE_BOX=1`, and
`SHAPE_CYLINDER=2`. `CollisionPrimitive.shape` uses the corresponding scene
shape values `SHAPE_BOX=1`, `SHAPE_CYLINDER=2`, and `SHAPE_SPHERE=3`.
`PlanGrip.phase` uses `PHASE_DESCEND=1`, `PHASE_CLOSE=2`,
`PHASE_OPEN=3`, and `PHASE_LIFT=4`. Supervisor mode and fault constants are
defined in `msg/SupervisorStatus.msg` and are intentionally not duplicated in
service definitions.

## Compatibility boundaries

The timber stack's `CalcMovement` service remains a separate compatibility
boundary. Native callers use `crane_msgs/Payload`; an adapter translates to the
legacy cylinder-shaped request where needed. Existing machine telemetry and
CBS domain interfaces remain unchanged.
