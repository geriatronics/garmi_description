<!--
Copyright 2026 Technical University of Munich
SPDX-License-Identifier: Apache-2.0
-->

# The real Garmi robot

This package is a **portable model**, not a driver. The joint names, link (TF)
frames and controller names in the URDF and MJCF were chosen to match the
physical Garmi robot, so a model you drive here lines up with the real robot's
`/joint_states`, TF tree and controller topics.

This page documents the real robot's ROS 2 interface (namespaces, controller
managers and topics) as a reference for anyone integrating this model with the
hardware. Nothing here is required to *use* the model — the illustrative RViz,
Gazebo and MuJoCo examples are self-contained.

## Controller managers

The real robot runs **two separate `controller_manager` instances**, each in its
own namespace, plus a servo driver for the head that is *not* a
`controller_manager` at all:

| Namespace | Subsystems | Rate |
| --- | --- | --- |
| `/garmi/arms` | both FR3 arms + the lift | 1000 Hz (lift 10 Hz) |
| `/r100_0603` | mecanum mobile base | 40 Hz |
| *(olive servo)* | pan/tilt head | — |

Each manager runs its own `joint_state_broadcaster`, so the raw state is split
across several partial topics; the robot aggregates them into a single
whole-robot `/garmi/joint_states` — see
[Aggregated `/joint_states`](#aggregated-joint_states) below.

### `/garmi/arms` — arms + lift

The arms are **effort-controlled** on the real robot. The default command path
is a chained stack: a velocity integrator feeds a joint-space PID that outputs
effort. A gravity-compensation controller and an effort-based trajectory
controller are configured but inactive by default.

| Controller | Type | Default state |
| --- | --- | --- |
| `left_arm_joint_velocity_controller` | `garmi_controllers/JointVelocityIntegratorController` | active |
| `left_arm_joint_position_controller` | `pid_controller/PidController` (effort out) | active |
| `left_arm_joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` (effort) | inactive |
| `left_arm_gravity_compensation_controller` | `garmi_controllers/GravityCompensationController` | inactive |
| `right_arm_*` | *(same four controllers as the left arm)* | — |
| `lift_0_position_controller` | `position_controllers/JointGroupPositionController` (10 Hz) | active |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |

The velocity controller is chained onto the position PID, i.e. it writes the
PID's `.../position` and `.../velocity` reference interfaces; the PID claims the
joints' `effort` command interfaces. The trajectory controller is likewise
effort-based (not position, as in the Gazebo example here). The
`joint_state_broadcaster` additionally publishes the Franka semantic state
interfaces (`*/robot_state`, `*/cartesian_pose_state`, TCP wrench, elbow, …).

Joints (`/garmi/arms`): `left_fr3_joint1..7`, `right_fr3_joint1..7`,
`lift_0_lower_joint`.

### `/r100_0603` — mobile base

| Controller | Type | Default state |
| --- | --- | --- |
| `platform_velocity_controller` | `mecanum_drive_controller/MecanumDriveController` | active |
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |

The base is commanded by a planar twist (`linear/x`, `linear/y`, `angular/z`) on
the controller's reference topic
(`/r100_0603/platform_velocity_controller/reference`, `geometry_msgs/TwistStamped`),
which the mecanum kinematics convert to the four wheel velocities. This is the
same interface the joystick plugin in the Gazebo example uses.

Joints (`/r100_0603`): `front_left_wheel_joint`, `front_right_wheel_joint`,
`rear_left_wheel_joint`, `rear_right_wheel_joint`.

### Head — `olive` servo

The pan/tilt head is driven by an `olive` servo, **not** a `controller_manager`:

- Joints: `o1_motor_1`, `o1_motor_2`.
- Command: `/olive/olixO1/id004/head_cmd` (and `head_goal`).
- State: `/olive/olixO1/id004/head_state` (`sensor_msgs/JointState`, names
  `o1_motor_1`, `o1_motor_2`).

In the sim examples the head is instead exposed through an ordinary
`head_controller` (a `JointTrajectoryController`) so it is easy to jog from the
GUI. Only the *command path* differs; the joint names and state match.

## Aggregated `/joint_states`

The two controller managers each broadcast their own partial
`sensor_msgs/JointState` (arms + lift under `/garmi/arms`, wheels under
`/r100_0603`), and the head publishes a third. The robot merges these into a
single whole-robot topic, **`/garmi/joint_states`**, which is what a consumer
such as `robot_state_publisher` or RViz should subscribe to.

The aggregation is done with a `joint_state_publisher` and a `source_list`,
which keeps the latest value per joint name and republishes one unified topic:

```yaml
joint_state_publisher:
  ros__parameters:
    source_list:
      - /garmi/arms/joint_states
      - /r100_0603/joint_states
      - /olive/olixO1/id004/head_state
```

All joint names are globally unique across the subsystems, so the merge is
unambiguous. This is a monitoring/visualisation convenience; each controller
manager reads the interfaces it owns directly and does not need the merged
topic.

## How the sim examples map to the real robot

The examples in this repo favour simple, stable controllers over exactly
reproducing the real effort-based stack (see the *Effort Interfaces Disabled*
caveat in the [README](../README.md)). Names and joints match; types differ:

| This repo (sim) | Real robot | Notes |
| --- | --- | --- |
| `left/right_arm_joint_trajectory_controller` (position) | `..._joint_trajectory_controller` (effort) | sim uses position for stability |
| `left/right_arm_joint_velocity_controller` (`velocity_controllers/JointGroupVelocityController`) | `..._joint_velocity_controller` (`garmi_controllers/JointVelocityIntegratorController`) | sim forwards velocity directly |
| `lift_0_position_controller` (`JointTrajectoryController`) | `lift_0_position_controller` (`JointGroupPositionController`) | same joint, same name |
| `platform_velocity_controller` (`MecanumDriveController`) | `platform_velocity_controller` (`MecanumDriveController`) | same type and reference topic |
| `head_controller` (`JointTrajectoryController`) | `olive` servo (`head_cmd`) | different command path |

The sim uses a single, un-namespaced `controller_manager`; the real robot splits
arms/lift and base across `/garmi/arms` and `/r100_0603`. An integrator can
namespace the controllers at deployment — the joint and controller names already
line up.
