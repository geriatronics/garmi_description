#!/usr/bin/env python3
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0

"""A minimal example node that drives Garmi in simulation.

It continuously commands a simple, periodic motion so colleagues can see the
whole robot move and use this as a starting point for their own controllers:

  * the mobile base drives in a steady circle,
  * the lift slowly travels up and down,
  * both arms perform a gentle sine motion on every joint.

Run it together with the Gazebo simulation, e.g.:

    docker compose up demo

or, in an environment that already has the simulation running:

    ros2 run garmi_description demo_motion.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Joint names, matching config/ros2_controllers.yaml and the URDF.
ARM_0_JOINTS = [f'left_fr3_joint{i}' for i in range(1, 8)]
ARM_1_JOINTS = [f'right_fr3_joint{i}' for i in range(1, 8)]
LIFT_JOINTS = ['lift_0_lower_joint']


class GarmiDemo(Node):
    """Publishes periodic references to all of Garmi's controllers."""

    def __init__(self):
        super().__init__('garmi_demo_controller')

        # Publishers, one per controller.
        #  * The arms use forward velocity controllers and take a
        #    Float64MultiArray of joint velocities on <controller>/commands.
        #  * The lift uses a joint-trajectory controller (streamed single
        #    points).
        #  * The mecanum base takes a velocity reference as a TwistStamped.
        self.arm_0_pub = self.create_publisher(
            Float64MultiArray, '/left_arm_joint_velocity_controller/commands', 10)
        self.arm_1_pub = self.create_publisher(
            Float64MultiArray, '/right_arm_joint_velocity_controller/commands', 10)
        self.lift_pub = self.create_publisher(
            JointTrajectory, '/lift_0_position_controller/joint_trajectory', 10)
        self.base_pub = self.create_publisher(
            TwistStamped, '/platform_velocity_controller/reference', 10)

        # Motion parameters (gentle, but clearly visible).
        self.arm_amplitude = 0.3       # rad, sets the joint sweep (peak 2*A)
        self.arm_period = 5.0          # s
        self.lift_center = 0.2         # m, mid-stroke (limits: 0.0 .. 0.4)
        self.lift_amplitude = 0.15     # m
        self.lift_period = 12.0        # s (stays under the 0.088 m/s limit)
        self.base_linear = 0.3         # m/s forward
        self.base_angular = 0.5        # rad/s -> circle radius ~0.6 m

        self.dt = 0.1                  # s, 10 Hz command rate
        self.start_time = None
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('Garmi demo controller started.')

    def update(self):
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t = (now - self.start_time).nanoseconds * 1e-9

        self._command_arms(t)
        self._command_lift(t)
        self._command_base()

    def _command_arms(self, t):
        omega = 2.0 * math.pi / self.arm_period
        # Velocity is the time-derivative of an A*(1-cos(wt)) position profile,
        # so it starts smoothly at zero and the joints sweep from their initial
        # pose out to +2A and back. Both arms move in unison.
        speed = self.arm_amplitude * omega
        vel_0 = speed * math.sin(omega * t)
        vel_1 = -speed * math.sin(omega * t + math.pi)
        self.arm_0_pub.publish(self._velocities(ARM_0_JOINTS, vel_0))
        self.arm_1_pub.publish(self._velocities(ARM_1_JOINTS, vel_1))

    def _command_lift(self, t):
        omega = 2.0 * math.pi / self.lift_period
        height = self.lift_center + self.lift_amplitude * math.sin(omega * t)
        self.lift_pub.publish(self._trajectory(LIFT_JOINTS, [height]))

    def _command_base(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.base_linear
        msg.twist.angular.z = self.base_angular
        self.base_pub.publish(msg)

    def _velocities(self, joint_names, value):
        """Build a Float64MultiArray with the same velocity for every joint."""
        msg = Float64MultiArray()
        msg.data = [float(value)] * len(joint_names)
        return msg

    def _trajectory(self, joint_names, positions):
        """Build a single-point trajectory reached a few control steps ahead."""
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(seconds=2.0 * self.dt).to_msg()
        traj.points = [point]
        return traj


def main(args=None):
    rclpy.init(args=args)
    node = GarmiDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
