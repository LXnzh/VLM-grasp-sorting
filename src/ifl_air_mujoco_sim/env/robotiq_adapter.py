#!/usr/bin/env python3

"""Mock Robotiq 2F gripper adapter.

Provides a RobotiqAdapter class which runs a simple TCP server emulating a Robotiq register API.
The class exposes get_target_position() and set_current_position() for external control and inspection.

The module can still be run as a script to start the mock server (keeps previous behavior).
"""

import socket
import threading
import time
import sys
from typing import Optional


class RobotiqAdapter:
    """Simple mock adapter for the Robotiq 2F gripper.

    It listens for textual "GET <REG>" and "SET <REG> <VALUE> ..." commands over TCP and
    maintains an internal register dictionary. Thread-safe accessors allow external code to
    read/write the simulated gripper target/current positions.
    """

    def __init__(self, host: str = 'localhost', port: int = 63352):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._server_thread: Optional[threading.Thread] = None
        self._running = threading.Event()
        self._lock = threading.Lock()
        # Registers emulate Robotiq behavior (strings for simplicity)
        self.registers = {
            'ACT': '0', # Activation register
            'GTO': '0', # Go to position register
            'STA': '0', # Status status register
            'OBJ': '0', # Object detection register
            'POS': '0', # Current position register
            'PRE': '0'  # Position request register
        }
        
        self.movement_speed = 1.0  # seconds to close/open gripper fully
        self.last_move_time = time.time()
        self.internal_goal_position = 0  # internal target position (0-255)

    def start(self):
        """Start the adapter's TCP server in a background thread."""
        if self._server_thread and self._server_thread.is_alive():
            return
        self._running.set()
        self._server_thread = threading.Thread(target=self._serve, daemon=True)
        self._server_thread.start()

    def stop(self, timeout: float = 1.0):
        """Stop the server and wait for the thread to exit."""
        self._running.clear()
        # Close listening socket to break accept()
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        if self._server_thread:
            self._server_thread.join(timeout)

    def _serve(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.host, self.port))
            self._sock.listen(5)
        except Exception as e:
            print(f"[ERROR] Failed to start Robotiq server: {e}")
            return

        print(f"[INFO] Robotiq server listening on {self.host}:{self.port}")

        while self._running.is_set():
            try:
                self._sock.settimeout(0.5)
                try:
                    client_socket, client_address = self._sock.accept()
                except socket.timeout:
                    print(".", end='', flush=True)
                    continue
                print(f"Connection from {client_address} established")
                with client_socket:
                    client_socket.settimeout(0.5)
                    while self._running.is_set():
                        # emulate internal gripper state changes
                        with self._lock:
                            if self.registers.get('ACT', '0') == '1':
                                self.registers['STA'] = '3'

                        try:
                            data = client_socket.recv(1024)
                        except socket.timeout:
                            continue
                        except Exception:
                            break
                        if not data:
                            break
                        try:
                            text = data.decode('utf-8').strip()
                        except Exception:
                            break
                        parts = text.split()
                        response = 'Invalid'
                        if len(parts) >= 2:
                            cmd = parts[0].upper()
                            if cmd == 'GET':
                                reg = parts[1]
                                with self._lock:
                                    val = self.registers.get(reg, '0')
                                response = f"{reg} {val}"
                            elif cmd == 'SET':
                                # SET <REG> <VAL> [<REG> <VAL> ...]
                                if len(parts) < 3 or (len(parts) - 1) % 2 != 0:
                                    response = 'Invalid'
                                    print("[ERROR] Invalid SET command received")
                                else:
                                    with self._lock:
                                        for i in range(1, len(parts), 2):
                                            reg = parts[i]
                                            val = parts[i + 1]
                                            self.registers[reg] = val
                                            if reg == 'POS':
                                                self.registers['PRE'] = val
                                                # print(f"[INFO] Set gripper target position to {val}")
                                            if reg == 'GTO':
                                                self.registers['OBJ'] = '0' # moving
                                    response = 'ack'
                        try:
                            client_socket.sendall(response.encode('utf-8'))
                        except Exception:
                            break
                        
                        now = time.time()
                        elapsed = now - self.last_move_time
                        if elapsed > 0.01:
                            steps = int(min(255, (elapsed / self.movement_speed) * 255.0))
                            self.last_move_time = now
                        
                            pre_position = int(self.registers.get('PRE', '0'))    
                            if abs(pre_position - self.internal_goal_position) > 0:
                                #print(f"[DEBUG] elapsed time: {elapsed:.3f}s, steps: {steps}")
                                #print(f"[DEBUG] Moving gripper from {self.internal_goal_position} to {pre_position} by {steps} steps")
                                if pre_position > self.internal_goal_position:
                                    # clamp to target
                                    self.internal_goal_position = min(pre_position, self.internal_goal_position + steps)
                                elif pre_position < self.internal_goal_position:
                                    # clamp to target
                                    self.internal_goal_position = max(pre_position, self.internal_goal_position - steps)
                                self.registers['OBJ'] = '0' # moving
                                self.obj_stall_vote = 0
                            elif abs(int(self.registers.get('POS', '0')) - int(self.registers.get('PRE', '0'))) < 10:
                                self.registers['OBJ'] = '3' # at target
                                self.obj_stall_vote = 0
                            else:
                                self.obj_stall_vote = min(getattr(self, 'obj_stall_vote', 0) + 1, 10)
                                if self.obj_stall_vote >= 3:
                                    self.registers['OBJ'] = '1' # stalled due to object

                            #print(f"[DEBUG] POS: {self.registers.get('POS', '0')}, PRE: {self.registers.get('PRE', '0')} OBJ: {self.registers.get('OBJ', '0')}")

                print(f"Connection with {client_address} closed")
            except Exception:
                # continue accepting while running
                continue

        try:
            self._sock.close()
        except Exception:
            pass

    def __normalized_position_from_radians(self, radians: float) -> int:
        """Convert radians to normalized position (0-255)."""
        # Assuming 0 rad = fully open (0), 0.8 rad = fully closed (255)
        clamped = max(0.0, min(0.8, radians))
        return int((clamped / 0.8) * 255.0)

    def __radians_from_normalized_position(self, normalized: int) -> float:
        """Convert normalized position (0-255) to radians."""
        clamped = max(0, min(255, normalized))
        return (clamped / 255.0) * 0.8

    # Public API for external code / ROS2Interface
    def get_target_position(self) -> float:
        """ Get the current target position for the gripper """
        return self.internal_goal_position


    def set_current_position(self, value: float):
        """ Set the current position of the gripper """
        norm_pos = self.__normalized_position_from_radians(value)
        with self._lock:
            self.registers['POS'] = str(norm_pos)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    port = 63352
    if len(argv) > 0:
        try:
            port = int(argv[0])
        except Exception:
            pass
    adapter = RobotiqAdapter(port=port)
    try:
        adapter.start()
        # Keep main thread alive while server runs
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('[INFO] Shutting down Robotiq server')
    finally:
        adapter.stop()


if __name__ == '__main__':
    main()
