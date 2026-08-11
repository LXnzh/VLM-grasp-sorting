# Copyright (c) 2019 Anders Prier Lindvig - SDU Robotics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#####################################################
# Modifications 2022 FZI Forschungszentrum Informatik
# Original source as of 2022-12-07:
# https://gitlab.com/sdurobotics/ur_rtde/blob/master/doc/_static/robotiq_gripper.py

"""Module that encapsulates the socket interface to the Robotiq Grippers URCap."""

import socket
import threading
import time
import typing
from collections import OrderedDict
from typing import Union, Tuple
from enum import Enum


class GripperStatus(Enum):
    """Gripper status reported by the gripper."""

    RESET = 0
    ACTIVATING = 1
    # UNUSED = 2  # This value is currently not used by the gripper firmware
    ACTIVE = 3


class ObjectStatus(Enum):
    """Object status reported by the gripper."""

    MOVING = 0
    STOPPED_OUTER_OBJECT = 1
    STOPPED_INNER_OBJECT = 2
    AT_DEST = 3


def clip_val(min_value: int, value: int, max_value: int) -> int:
    """
    Clips a value to the bounds specified by both arguments.

    :param min_value: Lowest value returned
    :param value: Value that should be clipped to the bounds.
    :param max_value: Maximum value returned

    :return Clipped value
    """
    return max(min_value, min(value, max_value))


class Robotiq2fSocketAdapter:
    """Communicates with the gripper directly via socket with string commands."""

    __SET_COMMAND = 'SET'
    __GET_COMMAND = 'GET'
    __ACK_MESSAGE = 'ack'

    ACT = 'ACT'
    GTO = 'GTO'
    ATR = 'ATR'
    ADR = 'ADR'
    FOR = 'FOR'
    SPE = 'SPE'
    POS = 'POS'
    STA = 'STA'
    PRE = 'PRE'
    OBJ = 'OBJ'
    FLT = 'FLT'

    ENCODING = 'UTF-8'

    def __init__(self):
        self.socket: typing.Optional[socket.socket] = None
        self.socket_lock = threading.Lock()
        self._min_position: int = 0
        self._max_position: int = 255
        self._min_speed: int = 0
        self._max_speed: int = 255
        self._min_force: int = 0
        self._max_force: int = 255

        self._host: typing.Optional[str] = None
        self._port: typing.Optional[int] = None
        self._timeout: float = 2.0

    def connect(self, hostname: str, port: int, socket_timeout: float = 2.0) -> None:
        """
        Connect to a Robotiq 2 finger gripper at the given address.
        """
        with self.socket_lock:
            self._host, self._port, self._timeout = hostname, port, socket_timeout

            if self.socket is not None:
                try:
                    self.socket.close()
                finally:
                    self.socket = None

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((hostname, port))
            self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        """Close the connection with the gripper."""
        if self.socket is None:
            return

        self._reset()

        with self.socket_lock:
            try:
                self.socket.close()
            finally:
                self.socket = None

    def _ensure_connected(self) -> None:
        """Reconnect if socket is missing/closed."""
        if self.socket is not None:
            return
        if self._host is None or self._port is None:
            raise RuntimeError("Endpoint unknown. Call connect(host, port) first.")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self._host, self._port))
        s.settimeout(self._timeout)
        self.socket = s

    def _send_recv(self, payload: bytes, expect_bytes: int = 1024, retries: int = 1) -> typing.Optional[bytes]:
        """
        Send and receive with at most `retries`. If still failing, return None.
        """
        attempt = 0
        while attempt < retries:
            attempt += 1
            with self.socket_lock:
                try:
                    self._ensure_connected()
                    assert self.socket is not None
                    self.socket.sendall(payload)
                    data = self.socket.recv(expect_bytes)
                    if not data:
                        raise ConnectionResetError("Peer closed")
                    return data
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError,
                        TimeoutError, ConnectionRefusedError, OSError):
                    if self.socket is not None:
                        try:
                            self.socket.close()
                        finally:
                            self.socket = None
                    # short sleep before retry
                    time.sleep(0.05)
        # After retries exhausted → non-blocking fallback
        return None

    def set_gripper_variables(
        self,
        variable_dict: typing.OrderedDict[str, Union[int, float]]
    ) -> bool:
        """
        Set the variables specified in the dict on the gripper via the socket connection.
        This command waits for the 'ack' response form the gripper.
        """
        cmd = f'{self.__SET_COMMAND}'
        for variable, value in variable_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += '\n'  # new line is required for the command to finish

        data = self._send_recv(cmd.encode(self.ENCODING), expect_bytes=1024, retries=1)
        if data is None:
            return False
        return self._is_ack(data)

    def set_gripper_variable(self, variable: str, value: Union[int, float]):
        return self.set_gripper_variables(OrderedDict([(variable, value)]))

    def get_gripper_variable(self, variable: str) -> int:
        """
        Get the specified variable value from the gripper using the socket connection.

        This command blocks until the response has been received or a timeout occurs.

        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer, or -1 if timeout occurs.
        """
        # atomic commands send/rcv
        cmd = f"{self.__GET_COMMAND} {variable}\n"
        data = self._send_recv(cmd.encode(self.ENCODING), expect_bytes=1024, retries=1)
        if data is None:
            # --- return dummy value when server is dead ---
            return -1   # or 0 if that's safer for your logic
        try:
            var_name, value_str = data.decode(self.ENCODING).split()
            if var_name != variable:
                raise ValueError(
                    f"Unexpected response {data} ({data.decode(self.ENCODING)}): "
                    f"does not match '{variable}'"
                )
            return int(value_str)
        except Exception:
            return -1

    def _is_ack(self, data: bytes) -> bool:
        """
        Check if received data is an acknowledgement message.

        :param data: bytes data received from the socket.

        :return if the data is an acknowledgement message.
        """
        return data.decode(self.ENCODING) == self.__ACK_MESSAGE

    def _reset(self):
        """Reset the gripper."""
        if not self.set_gripper_variable(self.ACT, 0):
            raise RuntimeError("Failed to reset gripper ACT register.")
        if not self.set_gripper_variable(self.ATR, 0):
            raise RuntimeError("Failed to reset gripper ATR register.")
        self._wait_until(
            lambda: (
                self.get_gripper_variable(self.ACT) == 0
                and self.get_gripper_variable(self.STA) == 0
            ),
            timeout_sec=5.0,
            description="gripper reset",
        )
        time.sleep(0.5)

    def _wait_until(self, predicate, timeout_sec: float, description: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        raise RuntimeError(f"Timed out while waiting for {description}.")

    def activate(self, auto_calibrate: bool = True):
        """
        Reset the activation flag in the gripper and clear previous fault flags.

        :param auto_calibrate: Whether to calibrate the
        minimum and maximum positions based on actual motion.
        """
        if not self.is_active:
            self._reset()
            if not self.set_gripper_variable(self.ACT, 1):
                raise RuntimeError("Failed to set gripper ACT register.")
            time.sleep(1.0)
            self._wait_until(
                lambda: (
                    self.get_gripper_variable(self.ACT) == 1
                    and self.get_gripper_variable(self.STA) == 3
                ),
                timeout_sec=5.0,
                description="gripper activation",
            )

        if auto_calibrate:
            self.auto_calibrate()

    def deactivate(self):
        if self.is_active:
            self._reset()

    @property
    def is_active(self):
        """Return whether the gripper is active."""
        status = self.get_gripper_variable(self.STA)
        try:
            return GripperStatus(status) == GripperStatus.ACTIVE
        except ValueError:
            return False

    @property
    def max_force(self):
        """Return the maximum force the gripper can provide."""
        return self._max_force

    @property
    def min_force(self):
        """Return the minimum force the gripper can provide."""
        return self._min_force

    @property
    def max_speed(self):
        """Return the maximum force the gripper can move with."""
        return self._max_speed

    @property
    def min_speed(self):
        """Return the minimum force the gripper can move with."""
        return self._min_speed

    @property
    def min_position(self) -> int:
        """Return the minimum position the gripper can reach (open position)."""
        return self._min_position

    @property
    def max_position(self) -> int:
        """Return the maximum position the gripper can reach (closed position)."""
        return self._max_position

    @property
    def open_position(self) -> int:
        """Return what is considered the open position for gripper (minimum position value)."""
        return self.min_position

    @property
    def closed_position(self) -> int:
        """Return what is considered the closed position for gripper (maximum position value)."""
        return self.max_position

    @property
    def force(self) -> int:
        """Return the current force used by the gripper."""
        return self.get_gripper_variable(self.FOR)

    @property
    def speed(self) -> int:
        """Return the current speed used by the gripper."""
        return self.get_gripper_variable(self.SPE)

    @property
    def position(self) -> int:
        """Return the current position as returned by the physical hardware."""
        return self.get_gripper_variable(self.POS)

    def auto_calibrate(self, log: bool = True) -> None:
        """
        Attempt to calibrate the open and closed positions.

        This slowly closes and opens the gripper.

        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self.move_and_wait_for_pos(self.open_position, 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            print(f"Calibration failed opening to start: {str(status)}")

        # try to close as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.closed_position, 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            print(f"Calibration failed because of an object: {str(status)}")
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.open_position, 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            print(f"Calibration failed because of an object: {str(status)}")
        assert position >= self._min_position
        self._min_position = position

        if log:
            print(f"Gripper auto-calibrated to [{self.min_position}, {self.max_position}]")

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """
        Send commands to start moving towards the given position.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]

        :return: A tuple with a bool indicating whether the action it was successfully sent,
        and an integer with the actual position that was requested,
        after being adjusted to the min/max calibrated range.
        """
        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict(
            [
                (self.POS, clip_pos),
                (self.SPE, clip_spe),
                (self.FOR, clip_for),
                (self.GTO, 1)
            ]
        )

        return self.set_gripper_variables(var_dict), clip_pos

    def move_and_wait_for_pos(
        self,
        position: int,
        speed: int,
        force: int
    ) -> Tuple[int, ObjectStatus]:
        """
        Send commands to start moving towards the given position waiting for the move to complete.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]

        :return: A tuple with an integer representing the last position returned by the gripper
        after it notified that the move had completed, a status indicating how the move ended.
        """
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while self.get_gripper_variable(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self.get_gripper_variable(self.OBJ)
        while ObjectStatus(cur_obj) == ObjectStatus.MOVING:
            cur_obj = self.get_gripper_variable(self.OBJ)

        # report the actual position and the object status
        final_pos = self.get_gripper_variable(self.POS)
        final_obj = cur_obj
        return final_pos, ObjectStatus(final_obj)
