# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""rqt plugin: a joystick to drive the Garmi mecanum base by twist.

Publishes a geometry_msgs/TwistStamped to the mecanum base controller from a
draggable joystick (linear x/y) plus a vertical slider (yaw rate). Unlike
rqt_robot_steering (vx + yaw only), this commands the full planar twist, so the
base can strafe. Loaded inside rqt, it can sit in the same window as
rqt_joint_trajectory_controller (see config/garmi_teleop.perspective).

This is the Qt counterpart of the Tk joystick used by the MuJoCo teleop
(garmi_description/scripts/twist_joystick.py); the two are kept deliberately
similar.
"""
import math

from python_qt_binding.QtCore import Qt, QTimer, QPointF
from python_qt_binding.QtGui import QPainter, QPalette, QPen
from python_qt_binding.QtWidgets import QHBoxLayout, QLabel, QSlider, QVBoxLayout, QWidget

from geometry_msgs.msg import TwistStamped
from rqt_gui_py.plugin import Plugin

TOPIC = "/garmi_base_controller/reference"
VMAX = 0.7      # m/s
WMAX = 1.2      # rad/s
PUBLISH_HZ = 30.0


class JoystickPad(QWidget):
    """Draggable 2D pad. up = +x (forward), left = +y. Springs back on release."""

    def __init__(self, radius=95):
        super().__init__()
        self._R = radius
        self.vx = 0.0
        self.vy = 0.0
        self._knob = QPointF(0, 0)
        self.setMinimumSize(2 * (radius + 14), 2 * (radius + 14))

    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def _set_from_pos(self, pos):
        c = self._center()
        dx, dy = pos.x() - c.x(), pos.y() - c.y()
        dist = math.hypot(dx, dy)
        if dist > self._R:
            dx, dy = dx * self._R / dist, dy * self._R / dist
        self._knob = QPointF(dx, dy)
        self.vx = -dy / self._R * VMAX   # up = forward
        self.vy = -dx / self._R * VMAX   # left = +y
        self.update()

    def mousePressEvent(self, e):
        self._set_from_pos(e.pos())

    def mouseMoveEvent(self, e):
        self._set_from_pos(e.pos())

    def mouseReleaseEvent(self, e):
        self._knob = QPointF(0, 0)
        self.vx = self.vy = 0.0
        self.update()

    def paintEvent(self, _):
        # Use the system palette so the widget matches rqt's theme (light/dark).
        pal = self.palette()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = self._center()
        p.setPen(QPen(pal.color(QPalette.Mid), 2))
        p.drawEllipse(c, self._R, self._R)
        p.setPen(QPen(pal.color(QPalette.Midlight), 1))
        p.drawLine(QPointF(c.x(), c.y() - self._R), QPointF(c.x(), c.y() + self._R))
        p.drawLine(QPointF(c.x() - self._R, c.y()), QPointF(c.x() + self._R, c.y()))
        p.setBrush(pal.color(QPalette.Highlight))
        p.setPen(Qt.NoPen)
        p.drawEllipse(c + self._knob, 14, 14)


class TwistJoystickWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setObjectName("GarmiTwistJoystick")
        # No hardcoded colors: inherit rqt's system theme (light or dark).
        self.pad = JoystickPad()

        self.slider = QSlider(Qt.Vertical)
        self.slider.setMinimum(-100)
        self.slider.setMaximum(100)
        self.slider.setValue(0)
        self.slider.sliderReleased.connect(lambda: self.slider.setValue(0))

        left = QVBoxLayout()
        left.addWidget(self.pad)
        left.addWidget(QLabel("drag: forward / strafe"), alignment=Qt.AlignHCenter)
        right = QVBoxLayout()
        right.addWidget(QLabel("turn"), alignment=Qt.AlignHCenter)
        right.addWidget(self.slider)

        layout = QHBoxLayout(self)
        layout.addLayout(left)
        layout.addLayout(right)

    def twist(self):
        return (self.pad.vx, self.pad.vy, self.slider.value() / 100.0 * WMAX)


class TwistJoystickPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("GarmiTwistJoystickPlugin")
        self._node = context.node
        self._pub = self._node.create_publisher(TwistStamped, TOPIC, 10)

        self._widget = TwistJoystickWidget()
        self._widget.setWindowTitle("Garmi Twist Joystick")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"{self._widget.windowTitle()} ({context.serial_number()})")
        context.add_widget(self._widget)

        self._timer = QTimer()
        self._timer.timeout.connect(self._publish)
        self._timer.start(int(1000 / PUBLISH_HZ))

    def _publish(self):
        vx, vy, wz = self._widget.twist()
        msg = TwistStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self._pub.publish(msg)

    def shutdown_plugin(self):
        self._timer.stop()
        self._node.destroy_publisher(self._pub)
