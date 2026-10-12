import socket
import sys
import re
import time

from PyQt6.QtCore import QSocketNotifier, Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
    QWidget,
)

NUCLEO_PORT = 5001
LOCAL_PORT = 5000
CAMERA_HOST = "127.0.0.1"
CAMERA_PORT = 5005
TRACK_DEADBAND_DEG = 0.5
TRACK_STEPS_PER_DEG = 4.0  # Tune after calibration
TRACK_MAX_STEPS = 10
TRACK_SETTLE_MS = 150
TRACK_ACK_TIMEOUT_MS = 1500
TRACK_DATA_TIMEOUT_MS = 500
TRACK_H_SIGN = 1  # Reverse to -1 if horizontal motion increases error
TRACK_V_SIGN = 1  # Reverse to -1 if vertical motion increases error
CAMERA_PATTERN = re.compile(r"^H:([+-]?\d+(?:\.\d+)?),\s*V:([+-]?\d+(?:\.\d+)?)$")

START_M1 = -4088
START_M2 = 4650
START_M3 = -4267


SILENT_COMMANDS = {
    "M1_LEFT",
    "M1_RIGHT",
    "M2_UP",
    "M2_DOWN",
    "M3_UP",
    "M3_DOWN",
    "STOP",
    "GET_POS",
}


class RobotGUI(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle(
            "Robot Arm Control"
        )

        self.resize(
            500,
            520
        )

        # Robot state

        self.connected = False
        self.manual_mode = False
        self.zero_set = False
        self.auto_moving = False

        self.nucleo_ip = None

        # Tracking state: one movement per new post-settle camera measurement.
        self.tracking = False
        self.track_waiting_ack = False
        self.track_settle_until = 0.0
        self.track_last_camera_time = 0.0
        self.track_last_command_time = 0.0
        self.track_h = None
        self.track_v = None
        self.track_camera_packets = 0
        self.track_required_packet = 0
        self.track_last_command = None
        self.track_timer = QTimer(self)
        self.track_timer.setInterval(50)
        self.track_timer.timeout.connect(self.track_watchdog)
        self.track_timer.start()

        # =====================================================
        # JOG CONTROL
        # =====================================================

        self.jog_command = None

        self.jog_timer = QTimer(
            self
        )

        self.jog_timer.setInterval(
            50
        )

        self.jog_timer.timeout.connect(
            self.refresh_jog
        )

        # =====================================================
        # UDP SOCKET
        # =====================================================

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.bind(
            ("", LOCAL_PORT)
        )

        self.sock.setblocking(
            False
        )

        self.notifier = QSocketNotifier(
            self.sock.fileno(),
            QSocketNotifier.Type.Read
        )

        self.notifier.activated.connect(
            self.receive_udp
        )

        # Independent camera UDP input, not the Nucleo control socket.
        self.camera_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.camera_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.camera_sock.bind((CAMERA_HOST, CAMERA_PORT))
        #self.camera_sock.setblocking(False)
        self.camera_notifier = QSocketNotifier(
            self.camera_sock.fileno(), QSocketNotifier.Type.Read
        )
        self.camera_notifier.activated.connect(self.receive_camera)

        # =====================================================
        # CONNECTION
        # =====================================================

        self.ip_label = QLabel(
            "Nucleo IP:"
        )

        self.ip_edit = QLineEdit(
            "192.168.10.2"
        )

        self.connect_button = QPushButton(
            "CONNECT"
        )

        self.connect_button.clicked.connect(
            self.connect_device
        )

        self.connection_status = QLabel(
            "DISCONNECTED"
        )

        self.connect_timer = QTimer(
            self
        )

        self.connect_timer.setSingleShot(
            True
        )

        self.connect_timer.timeout.connect(
            self.connection_timeout
        )

        # =====================================================
        # MANUAL MODE
        # =====================================================

        self.manual_button = QPushButton(
            "MANUAL MODE"
        )

        self.manual_button.setCheckable(
            True
        )

        self.manual_button.clicked.connect(
            self.manual_mode_changed
        )

        self.manual_button.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        # =====================================================
        # MOTOR SELECTION
        # =====================================================

        self.motor1 = QRadioButton(
            "Motor 1"
        )

        self.motor2 = QRadioButton(
            "Motor 2"
        )

        self.motor3 = QRadioButton(
            "Motor 3"
        )

        self.position_set = QRadioButton(
            "Position set"
        )

        self.motor_group = QButtonGroup(
            self
        )

        self.motor_group.addButton(
            self.motor1
        )

        self.motor_group.addButton(
            self.motor2
        )

        self.motor_group.addButton(
            self.motor3
        )

        self.motor_group.addButton(
            self.position_set
        )

        self.position_set.setChecked(
            True
        )

        self.motor1.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.motor2.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.motor3.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.position_set.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        # =====================================================
        # ZERO SET
        # =====================================================

        self.zero_set_button = QPushButton(
            "ZERO SET"
        )

        self.zero_set_button.clicked.connect(
            self.set_zero
        )

        self.zero_set_button.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        # =====================================================
        # START POSITION
        # =====================================================

        self.start_position_label = QLabel(
            "START:  "
            f"M1 {START_M1}    "
            f"M2 {START_M2}    "
            f"M3 {START_M3}"
        )

        # =====================================================
        # GO TO START
        # =====================================================

        self.go_start_button = QPushButton(
            "GO TO START"
        )

        self.go_start_button.clicked.connect(
            self.go_to_start
        )

        self.go_start_button.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.go_start_button.setEnabled(
            False
        )

        # =====================================================
        # TRACKING (lowest priority)
        # =====================================================
        self.tracking_button = QPushButton("TRACKING")
        self.tracking_button.setCheckable(True)
        self.tracking_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.tracking_button.clicked.connect(self.tracking_changed)
        self.tracking_button.setEnabled(False)
        self.camera_status = QLabel("Camera: waiting for H/V packets on 127.0.0.1:5005")

        # =====================================================
        # STOP
        # =====================================================

        self.stop_button = QPushButton(
            "STOP"
        )

        self.stop_button.clicked.connect(
            self.stop_robot
        )

        self.stop_button.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.stop_button.setEnabled(
            False
        )

        # =====================================================
        # POSITION OUTPUT
        # =====================================================

        self.position_title = QLabel(
            "Motor position (steps)"
        )

        self.m1_position_label = QLabel(
            "M1:"
        )

        self.m1_position_edit = QLineEdit(
            "0"
        )

        self.m1_position_edit.setReadOnly(
            True
        )

        self.m1_position_edit.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.m2_position_label = QLabel(
            "M2:"
        )

        self.m2_position_edit = QLineEdit(
            "0"
        )

        self.m2_position_edit.setReadOnly(
            True
        )

        self.m2_position_edit.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        self.m3_position_label = QLabel(
            "M3:"
        )

        self.m3_position_edit = QLineEdit(
            "0"
        )

        self.m3_position_edit.setReadOnly(
            True
        )

        self.m3_position_edit.setFocusPolicy(
            Qt.FocusPolicy.NoFocus
        )

        # =====================================================
        # STATUS
        # =====================================================

        self.status = QLabel(
            "Robot disconnected"
        )

        # =====================================================
        # LAYOUT
        # =====================================================

        connection_layout = QHBoxLayout()

        connection_layout.addWidget(
            self.ip_label
        )

        connection_layout.addWidget(
            self.ip_edit
        )

        connection_layout.addWidget(
            self.connect_button
        )

        motor_layout = QHBoxLayout()

        motor_layout.addWidget(
            self.motor1
        )

        motor_layout.addWidget(
            self.motor2
        )

        motor_layout.addWidget(
            self.motor3
        )

        m1_position_layout = QHBoxLayout()

        m1_position_layout.addWidget(
            self.m1_position_label
        )

        m1_position_layout.addWidget(
            self.m1_position_edit
        )

        m2_position_layout = QHBoxLayout()

        m2_position_layout.addWidget(
            self.m2_position_label
        )

        m2_position_layout.addWidget(
            self.m2_position_edit
        )

        m3_position_layout = QHBoxLayout()

        m3_position_layout.addWidget(
            self.m3_position_label
        )

        m3_position_layout.addWidget(
            self.m3_position_edit
        )

        automatic_layout = QHBoxLayout()

        automatic_layout.addWidget(
            self.go_start_button
        )

        automatic_layout.addWidget(
            self.stop_button
        )
        automatic_layout.addWidget(self.tracking_button)

        layout = QVBoxLayout()

        layout.addLayout(
            connection_layout
        )

        layout.addWidget(
            self.connection_status
        )

        layout.addWidget(
            self.manual_button
        )

        layout.addLayout(
            motor_layout
        )

        layout.addWidget(
            self.position_set
        )

        layout.addWidget(
            self.zero_set_button
        )

        layout.addWidget(
            self.start_position_label
        )

        layout.addLayout(
            automatic_layout
        )

        layout.addWidget(
            self.position_title
        )

        layout.addLayout(
            m1_position_layout
        )

        layout.addLayout(
            m2_position_layout
        )

        layout.addLayout(
            m3_position_layout
        )

        layout.addWidget(self.camera_status)
        layout.addWidget(
            self.status
        )

        self.setLayout(
            layout
        )

        self.disable_robot_controls()

        self.setFocusPolicy(
            Qt.FocusPolicy.StrongFocus
        )

    # =========================================================
    # CONNECT / DISCONNECT
    # =========================================================

    def connect_device(self):

        if self.connected:

            self.cancel_tracking(send_stop=True)
            self.jog_timer.stop()

            self.jog_command = None

            self.send_command(
                "STOP"
            )

            if self.manual_mode:

                self.send_command(
                    "MANUAL_MODE_OFF"
                )

            self.connected = False
            self.manual_mode = False
            self.zero_set = False
            self.auto_moving = False

            self.manual_button.setChecked(
                False
            )

            self.connection_status.setText(
                "DISCONNECTED"
            )

            self.connect_button.setText(
                "CONNECT"
            )

            self.ip_edit.setEnabled(
                True
            )

            self.disable_robot_controls()

            self.status.setText(
                "Robot disconnected"
            )

            print(
                "Disconnected"
            )

            return

        ip = self.ip_edit.text().strip()

        if not ip:
            return

        self.nucleo_ip = ip

        print(
            "Connecting to",
            self.nucleo_ip
        )

        self.connection_status.setText(
            "CONNECTING..."
        )

        self.send_raw(
            "CONNECT"
        )

        self.connect_timer.start(
            1000
        )

    # =========================================================
    # CONNECTION TIMEOUT
    # =========================================================

    def connection_timeout(self):

        if not self.connected:

            self.connection_status.setText(
                "NO RESPONSE"
            )

            self.status.setText(
                "Robot disconnected"
            )

            print(
                "Connection failed: no response"
            )

    # =========================================================
    # UDP SEND
    # =========================================================

    def send_raw(
        self,
        command
    ):

        if self.nucleo_ip is None:
            return

        try:

            self.sock.sendto(
                command.encode(),
                (
                    self.nucleo_ip,
                    NUCLEO_PORT
                )
            )

            if command not in SILENT_COMMANDS:

                print(
                    "TX:",
                    command
                )

        except OSError as error:

            print(
                "UDP send error:",
                error
            )

    # =========================================================
    # SEND COMMAND
    # =========================================================

    def send_command(
        self,
        command
    ):

        if not self.connected:

            print(
                "Command blocked: not connected"
            )

            return

        self.send_raw(
            command
        )

    # =========================================================
    # REQUEST POSITION
    # =========================================================

    def request_position(self):

        if not self.connected:
            return

        self.send_command(
            "GET_POS"
        )

    # =========================================================
    # UDP RECEIVE
    # =========================================================

    def receive_udp(self):

        while True:

            try:

                data, address = self.sock.recvfrom(
                    1024
                )

            except BlockingIOError:
                break

            except OSError:
                break

            message = data.decode(
                errors="replace"
            ).strip()

            if (
                self.nucleo_ip is not None
                and address[0] != self.nucleo_ip
            ):
                continue

            if message.startswith(
                "POS,"
            ):

                self.process_position_message(
                    message
                )

                continue

            if message not in SILENT_COMMANDS:

                print(
                    "RX:",
                    message
                )

            if message == "CONNECT_OK":

                self.connect_timer.stop()

                self.connected = True

                self.connection_status.setText(
                    "CONNECTED"
                )

                self.connect_button.setText(
                    "DISCONNECT"
                )

                self.ip_edit.setEnabled(
                    False
                )

                self.manual_button.setEnabled(
                    True
                )

                self.stop_button.setEnabled(
                    True
                )

                self.status.setText(
                    "Connected - robot inactive"
                )

                print(
                    "Nucleo connected"
                )

                self.request_position()

            elif message == "MANUAL_MODE_ON":

                self.manual_mode = True

                self.manual_button.setChecked(
                    True
                )

                self.update_controls()

                self.status.setText(
                    "MANUAL MODE ACTIVE"
                )

                self.setFocus()

            elif message == "MANUAL_MODE_OFF":

                self.manual_mode = False

                self.manual_button.setChecked(
                    False
                )

                self.update_controls()

                self.status.setText(
                    "Manual mode off"
                )

            elif message == "TRACK_DONE":
                if self.tracking and self.track_waiting_ack:
                    self.track_waiting_ack = False
                    self.track_settle_until = time.monotonic() + TRACK_SETTLE_MS / 1000.0
                    self.track_required_packet = self.track_camera_packets + 1
                    self.status.setText("TRACKING: waiting for updated camera frame")
                    self.request_position()

            elif message in ("TRACK_BUSY", "TRACK_INVALID"):
                if self.tracking:
                    self.cancel_tracking(send_stop=False)
                    self.status.setText(f"TRACKING STOPPED: {message}")
                    self.update_controls()

            elif message == "STOP":

                if self.tracking:
                    self.cancel_tracking(send_stop=False)
                self.auto_moving = False

                self.update_controls()

                self.status.setText(
                    "STOPPED"
                )

                self.request_position()

            elif message == "ZERO_SET":

                self.zero_set = True

                self.update_controls()

                self.status.setText(
                    "ZERO POSITION SET"
                )

                self.request_position()

            elif message == "MOVE_STARTED":

                self.auto_moving = True

                self.jog_timer.stop()

                self.jog_command = None

                self.update_controls()

                self.status.setText(
                    "MOVING TO START..."
                )

            elif message == "MOVE_DONE":

                self.auto_moving = False

                self.update_controls()

                self.status.setText(
                    "START POSITION REACHED"
                )

                self.request_position()

    # =========================================================
    # POSITION MESSAGE
    # =========================================================

    def process_position_message(
        self,
        message
    ):

        parts = message.split(
            ","
        )

        if len(parts) != 4:

            print(
                "Invalid position message:",
                message
            )

            return

        try:

            m1 = int(
                parts[1]
            )

            m2 = int(
                parts[2]
            )

            m3 = int(
                parts[3]
            )

        except ValueError:

            print(
                "Invalid position values:",
                message
            )

            return

        self.m1_position_edit.setText(
            str(m1)
        )

        self.m2_position_edit.setText(
            str(m2)
        )

        self.m3_position_edit.setText(
            str(m3)
        )

    # =========================================================
    # MANUAL MODE
    # =========================================================

    def manual_mode_changed(
        self,
        enabled
    ):

        if not self.connected:

            self.manual_button.setChecked(
                False
            )

            return

        if self.auto_moving:

            self.manual_button.setChecked(
                self.manual_mode
            )

            return

        self.jog_timer.stop()

        self.jog_command = None

        if enabled:
            self.cancel_tracking(send_stop=True)

            # Do not clear zero_set here.
            #
            # Entering Manual Mode does not change
            # the coordinate system.
            #
            # The Nucleo releases M2's break and waits
            # 100 ms before acknowledging Manual Mode.

            self.send_command(
                "MANUAL_MODE_ON"
            )

            self.status.setText(
                "ENTERING MANUAL MODE..."
            )

            self.setFocus()

        else:

            self.send_command(
                "MANUAL_MODE_OFF"
            )

            self.position_set.setChecked(
                True
            )

            self.status.setText(
                "LEAVING MANUAL MODE..."
            )

    # =========================================================
    # ZERO SET
    # =========================================================

    def set_zero(self):

        if not self.manual_mode:
            return

        if self.auto_moving:
            return

        self.jog_timer.stop()

        self.jog_command = None

        self.send_command(
            "ZERO_SET"
        )

        self.position_set.setChecked(
            True
        )

        self.status.setText(
            "SETTING ZERO..."
        )

    # =========================================================
    # GO TO START
    # =========================================================

    def go_to_start(self):

        if not self.connected:
            return

        if not self.zero_set:
            return

        if self.auto_moving:
            return

        self.cancel_tracking(send_stop=True)
        self.jog_timer.stop()

        self.jog_command = None

        # Do NOT switch Manual Mode off.
        #
        # If Manual Mode is active, the M2 break is already
        # released and stays released during and after movement.
        #
        # If Manual Mode is not active, the Nucleo releases
        # the break, waits 100 ms, moves, and engages it again
        # when the movement is complete.

        self.send_command(
            "GO_TO_START"
        )

        self.status.setText(
            "STARTING MOVE..."
        )

    # =========================================================
    # STOP
    # =========================================================

    def stop_robot(self):

        self.cancel_tracking(send_stop=False)
        if not self.connected:
            return

        self.jog_timer.stop()

        self.jog_command = None

        self.send_command(
            "STOP"
        )

    # =========================================================
    # UPDATE CONTROLS
    # =========================================================

    def update_controls(self):

        if not self.connected:

            self.disable_robot_controls()

            return

        self.manual_button.setEnabled(
            not self.auto_moving
        )

        self.stop_button.setEnabled(
            True
        )

        if (
            self.manual_mode
            and not self.auto_moving
        ):

            self.motor1.setEnabled(
                True
            )

            self.motor2.setEnabled(
                True
            )

            self.motor3.setEnabled(
                True
            )

            self.position_set.setEnabled(
                True
            )

            self.zero_set_button.setEnabled(
                True
            )

        else:

            self.motor1.setEnabled(
                False
            )

            self.motor2.setEnabled(
                False
            )

            self.motor3.setEnabled(
                False
            )

            self.position_set.setEnabled(
                False
            )

            self.zero_set_button.setEnabled(
                False
            )

        self.go_start_button.setEnabled(
            self.zero_set
            and not self.auto_moving
        )
        self.tracking_button.setEnabled(
            not self.auto_moving and not self.manual_mode
        )
        self.tracking_button.setChecked(self.tracking)

    # =========================================================
    # JOG REFRESH
    # =========================================================

    def refresh_jog(self):

        if not self.connected:

            self.jog_timer.stop()

            self.jog_command = None

            return

        if not self.manual_mode:

            self.jog_timer.stop()

            self.jog_command = None

            return

        if self.auto_moving:

            self.jog_timer.stop()

            self.jog_command = None

            return

        if self.jog_command is None:

            self.jog_timer.stop()

            return

        self.send_command(
            self.jog_command
        )

    # =========================================================
    # START JOG
    # =========================================================

    def start_jog(
        self,
        command
    ):

        if not self.connected:
            return

        if not self.manual_mode:
            return

        if self.auto_moving:
            return

        self.jog_command = command

        self.send_command(
            self.jog_command
        )

        self.jog_timer.start()

    # =========================================================
    # KEY PRESS
    # =========================================================

    def keyPressEvent(
        self,
        event
    ):

        if not self.connected:
            return

        if not self.manual_mode:
            return

        if self.auto_moving:
            return

        if event.isAutoRepeat():
            return

        key = event.key()

        if self.motor1.isChecked():

            if key == Qt.Key.Key_Left:

                self.start_jog(
                    "M1_LEFT"
                )

            elif key == Qt.Key.Key_Right:

                self.start_jog(
                    "M1_RIGHT"
                )

        elif self.motor2.isChecked():

            if key == Qt.Key.Key_Up:

                self.start_jog(
                    "M2_UP"
                )

            elif key == Qt.Key.Key_Down:

                self.start_jog(
                    "M2_DOWN"
                )

        elif self.motor3.isChecked():

            if key == Qt.Key.Key_Up:

                self.start_jog(
                    "M3_UP"
                )

            elif key == Qt.Key.Key_Down:

                self.start_jog(
                    "M3_DOWN"
                )

    # =========================================================
    # KEY RELEASE
    # =========================================================

    def keyReleaseEvent(
        self,
        event
    ):

        if event.isAutoRepeat():
            return

        if (
            event.key() in (
                Qt.Key.Key_Left,
                Qt.Key.Key_Right,
                Qt.Key.Key_Up,
                Qt.Key.Key_Down
            )
            and self.jog_command is not None
        ):

            self.jog_timer.stop()

            self.jog_command = None

            if self.connected:

                # STOP only stops motion.
                #
                # While Manual Mode is active, the Nucleo
                # deliberately leaves the M2 break released.

                self.send_command(
                    "STOP"
                )

    # =========================================================
    # DISABLE MANUAL CONTROLS
    # =========================================================

    def disable_manual_controls(self):

        self.motor1.setEnabled(
            False
        )

        self.motor2.setEnabled(
            False
        )

        self.motor3.setEnabled(
            False
        )

        self.position_set.setEnabled(
            False
        )

        self.zero_set_button.setEnabled(
            False
        )

    # =========================================================
    # DISABLE ROBOT CONTROLS
    # =========================================================

    def disable_robot_controls(self):

        self.manual_button.setEnabled(
            False
        )

        self.stop_button.setEnabled(
            False
        )

        self.go_start_button.setEnabled(
            False
        )
        self.tracking_button.setEnabled(False)
        self.tracking_button.setChecked(False)

        self.disable_manual_controls()

    # =========================================================
    # CAMERA TRACKING
    # =========================================================

    def tracking_changed(self, enabled):
        if not enabled:
            self.cancel_tracking(send_stop=True)
            self.status.setText("TRACKING OFF")
            self.update_controls()
            return
        if not self.connected or self.manual_mode or self.auto_moving:
            self.tracking_button.setChecked(False)
            return
        self.tracking = True
        self.track_waiting_ack = False
        self.track_h = None
        self.track_v = None
        self.track_required_packet = self.track_camera_packets + 1
        self.track_settle_until = 0.0
        self.track_last_camera_time = time.monotonic()
        self.tracking_button.setChecked(True)
        self.status.setText("TRACKING ON: waiting for camera")
        self.update_controls()

    def cancel_tracking(self, send_stop=False):
        was_active = self.tracking or self.track_waiting_ack
        self.tracking = False
        self.track_waiting_ack = False
        self.track_h = None
        self.track_v = None
        self.track_last_command = None
        if hasattr(self, "tracking_button"):
            self.tracking_button.setChecked(False)
        if send_stop and was_active and self.connected:
            self.send_command("STOP")

    def receive_camera(self):
        while True:
            try:
                data, _addr = self.camera_sock.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError:
                break
            message = data.decode("ascii", errors="replace").strip()
            match = CAMERA_PATTERN.fullmatch(message)
            if not match:
                continue
            self.track_camera_packets += 1
            self.track_last_camera_time = time.monotonic()
            self.track_h = float(match.group(1))
            self.track_v = float(match.group(2))
            self.camera_status.setText(
                "Hello"
                #f"Camera: H={self.track_h:+.2f}°, V={self.track_v:+.2f}°"
            )
            self.try_tracking_step()

    def try_tracking_step(self):
        if not self.tracking or not self.connected:
            return
        if self.manual_mode or self.auto_moving or self.track_waiting_ack:
            return
        if self.track_h is None or self.track_v is None:
            return
        if self.track_camera_packets < self.track_required_packet:
            return
        if time.monotonic() < self.track_settle_until:
            return
        # The packet must have arrived after the settle interval.
        if self.track_last_camera_time < self.track_settle_until:
            return
        if time.monotonic() - self.track_last_camera_time > TRACK_DATA_TIMEOUT_MS / 1000.0:
            return

        def correction(error, sign):
            if abs(error) <= TRACK_DEADBAND_DEG:
                return 0
            magnitude = max(1, min(TRACK_MAX_STEPS,
                                   round(abs(error) * TRACK_STEPS_PER_DEG)))
            return (1 if error > 0 else -1) * sign * magnitude

        m1 = correction(self.track_h, TRACK_H_SIGN)
        m3 = correction(self.track_v, TRACK_V_SIGN)
        self.track_required_packet = self.track_camera_packets + 1
        if m1 == 0 and m3 == 0:
            self.status.setText("TRACKING: target centered")
            return

        # New firmware command: signed M1/M3 step counts.
        # Existing firmware does not implement this command yet.
        self.track_last_command = f"TRACK_STEP,{m1},{m3}"
        self.track_waiting_ack = True
        self.track_last_command_time = time.monotonic()
        self.send_command(self.track_last_command)
        self.status.setText(f"TRACKING: M1 {m1:+d}, M3 {m3:+d} steps")

    def track_watchdog(self):
        if not self.tracking:
            return
        now = time.monotonic()
        if self.track_waiting_ack:
            if now - self.track_last_command_time > TRACK_ACK_TIMEOUT_MS / 1000.0:
                self.cancel_tracking(send_stop=True)
                self.status.setText("TRACKING STOPPED: no TRACK_DONE acknowledgment")
                self.update_controls()
            return
        if now - self.track_last_camera_time > TRACK_DATA_TIMEOUT_MS / 1000.0:
            self.status.setText("TRACKING: waiting for camera data")
            return
        self.try_tracking_step()

    # =========================================================
    # WINDOW CLOSE
    # =========================================================

    def closeEvent(
        self,
        event
    ):

        self.cancel_tracking(send_stop=False)
        self.jog_timer.stop()

        self.jog_command = None

        if self.connected:

            self.send_command(
                "STOP"
            )

            # STOP deliberately does not engage M2's break
            # while Manual Mode is active.
            #
            # Therefore explicitly leave Manual Mode before
            # closing the application.

            if self.manual_mode:

                self.send_command(
                    "MANUAL_MODE_OFF"
                )

        self.notifier.setEnabled(
            False
        )
        self.camera_notifier.setEnabled(False)
        self.camera_sock.close()
        self.sock.close()

        event.accept()


# =============================================================
# APPLICATION
# =============================================================

app = QApplication(
    sys.argv
)

window = RobotGUI()

window.show()

sys.exit(
    app.exec()
)
