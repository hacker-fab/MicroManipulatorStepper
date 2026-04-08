import re
import threading
import time
from enum import Enum
import warnings

import numpy as np
import serial
from colorama import Fore, Style

# --- SerialInterface --------------------------------------------------------------------------------------------------


class SerialInterface:
    class ReplyStatus(Enum):
        OK = "ok"
        ERROR = "error"
        TIMEOUT = "timeout"
        BUSY = "busy"

    class LogLevel(Enum):
        DEBUG = "debug"
        INFO = "info"
        WARNING = "warning"
        ERROR = "error"

    # Static mapping from prefix to LogLevel
    log_level_prefix_map = {
        "D)": LogLevel.DEBUG,
        "I)": LogLevel.INFO,
        "W)": LogLevel.WARNING,
        "E)": LogLevel.ERROR,
    }

    def __init__(
        self,
        port: str,
        baud_rate: int = 115200,
        command_msg_callback=None,
        log_msg_callback=None,
        unsolicited_msg_callback=None,
        reconnect_timeout: int = 5,
    ):
        """
        Initializes the serial connection and starts background reader.
        :param port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
        :param baud_rate: Serial baud rate.
        :param log_msg_callback: called when a log message is received
        :param unsolicited_msg_callback: Optional function to call with unsolicited messages.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.reconnect_timeout = reconnect_timeout
        self.serial = None  # initialized on connect

        self.command_msg_callback = command_msg_callback
        self.log_message_callback = log_msg_callback
        self.unsolicited_msg_callback = unsolicited_msg_callback

        # Synchronization for blocking send/receive
        self._lock = threading.Lock()
        self._condition = threading.Condition(self._lock)
        self._waiting_for_response = False
        self._response_string = ""
        self._response_status = None
        self._response_error_msg = None

        self.connect(self.reconnect_timeout)

        # Start reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def connect(self, timeout):
        """
        Try to open the serial port. Retry until timeout expires.
        """
        deadline = time.time() + timeout
        print(Fore.MAGENTA, end="")
        print(f"[SerialInterface] Connecting to port '{self.port}'...", end="")
        while time.time() < deadline:
            try:
                self.serial = serial.Serial(self.port, self.baud_rate, timeout=30)
                print(" [OK]")
                print(Style.RESET_ALL, end="")
                return True
            except (serial.SerialException, OSError):
                print(".", end="")
                time.sleep(0.2)

        print(f" [FAILED] Timeout after {timeout} seconds.")
        print("[SerialInterface] Connection is permanently closed")
        print(Style.RESET_ALL, end="")
        self.serial = None
        return False

    def _reader_loop(self):
        """
        Asynchronous reader loop, collecting serial data into a buffer
        """
        buffer = ""
        while True:
            try:
                if self.serial is not None and self.serial.in_waiting:
                    char = self.serial.read(1).decode("ascii", errors="ignore")
                    if char in ["\n", "\r"]:
                        if len(buffer) > 0:
                            self._handle_line(buffer)
                            buffer = ""
                    else:
                        buffer += char
                else:
                    time.sleep(0.001)
            except (serial.SerialException, OSError) as e:
                print(Fore.MAGENTA + f"[SerialInterface] Lost connection: {e}" + Style.RESET_ALL)
                try:
                    if self.serial is not None and self.serial.is_open:
                        self.serial.close()
                except Exception:
                    pass

                self.serial = None
                self.connect(self.reconnect_timeout)

    def _handle_line(self, line: str):
        """
        Handles a single serial line sent by the device
        :param line: string containing a single line
        """
        with self._lock:
            log_level, log_msg = self._check_log_msg(line)
            # print(line)
            # log message
            if log_level is not None:
                if self.log_message_callback:
                    self.log_message_callback(log_level, log_msg)
            # response
            elif self._waiting_for_response:
                line_lower = line.lower()
                if line_lower.startswith("ok"):
                    self._response_status = SerialInterface.ReplyStatus.OK
                elif line_lower.startswith("busy"):
                    self._response_status = SerialInterface.ReplyStatus.BUSY
                elif line_lower.startswith("error"):
                    self._response_status = SerialInterface.ReplyStatus.ERROR
                    parts = line.split(":", 1)
                    self._response_error_msg = parts[1].strip() if len(parts) > 1 else ""

                if self._response_status is not None:
                    self._condition.notify()
                else:
                    self._response_string += line + "\n"

            # unsolicited message
            else:
                if self.unsolicited_msg_callback:
                    self.unsolicited_msg_callback(line)

    def _check_log_msg(self, msg: str):
        if len(msg) < 2:
            return None, ""
        return self.log_level_prefix_map.get(msg[:2]), msg[2:]

    def send_command(self, cmd: str, timeout=2) -> tuple[ReplyStatus, str]:
        """
        Sends a command and blocks until 'ok' or 'error' is received.
        :param cmd: The command to send.
        :param timeout: Maximum time to wait for response.
        :return: Tuple containing Status enum (OK | ERROR | TIMEOUT), and response lines.
        """
        with self._lock:
            if not self.serial or not self.serial.is_open:
                return SerialInterface.ReplyStatus.ERROR, "Serial not open"

            # Reset state
            self._waiting_for_response = True
            self._response_string = ""
            self._response_error_msg = ""
            self._response_status = None

            cmd = cmd.strip() + "\n"
            self.command_msg_callback(cmd, None, "")

            # Send command
            self.serial.write(cmd.encode("ascii"))
            self.serial.flush()

            # Wait for completion
            timeout = 180
            end_time = time.time() + timeout
            while self._response_status is None:
                remaining = end_time - time.time()
                if remaining <= 0:
                    self._waiting_for_response = False
                    print(
                        Fore.MAGENTA
                        + "[SerialInterface] Command timeout, device didn't reply in time"
                        + Style.RESET_ALL
                    )
                    return SerialInterface.ReplyStatus.TIMEOUT, self._response_string
                self._condition.wait(timeout=remaining)

            self._waiting_for_response = False
            self.command_msg_callback(self._response_string, self._response_status, self._response_error_msg)
            return self._response_status, self._response_string

    def close(self):
        """Closes the serial port."""
        if self.serial and self.serial.is_open:
            self.serial.close()


# --- OpenMicroStageInterface ------------------------------------------------------------------------------------------


class OpenMicroStageInterface:
    # Mapping log levels to colors
    LOG_COLORS = {
        SerialInterface.LogLevel.DEBUG: Fore.WHITE + Style.DIM,
        SerialInterface.LogLevel.INFO: Style.RESET_ALL,
        SerialInterface.LogLevel.WARNING: Fore.YELLOW,
        SerialInterface.LogLevel.ERROR: Fore.RED,
    }

    def __init__(self, show_communication: bool =True, show_log_messages: bool =True, exception_on_no_device: bool =True):
        """Initialize the OMM class

        Args:
            show_communication (bool, optional): If communication should be printed to stdout. Defaults to True.
            show_log_messages (bool, optional): If log messages should be printed to stdout. Defaults to True.
        """
        self.serial = None
        self.workspace_transform = np.eye(4)
        self.show_communication = show_communication
        self.show_log_messages = show_log_messages
        self.disable_message_callbacks = False
        self.exception_on_no_device = exception_on_no_device

    def connect(self, port: str, baud_rate: int = 921600):
        """Connects to the OMM

        Args:
            port (str): _description_
            baud_rate (int, optional): _description_. Defaults to 921600.
        """
        def version_to_str(v):
            return f"v{v[0]}.{v[1]}.{v[2]}"

        if self.serial is not None:
            self.disconnect()
        self.serial = SerialInterface(
            port,
            baud_rate,
            log_msg_callback=self.log_msg_callback,
            command_msg_callback=self.command_msg_callback,
            unsolicited_msg_callback=self.unsolicited_msg_callback,
        )

        self.disable_message_callbacks = True
        fw_version = self.read_firmware_version()
        min_fw_version = (1, 0, 1)
        print(Fore.MAGENTA + f"Firmware version: {version_to_str(fw_version)}" + Style.RESET_ALL)
        if fw_version < min_fw_version:
            print(
                Fore.MAGENTA + f"Firmware version {version_to_str(fw_version)} incompatible. "
                f"At least {version_to_str(min_fw_version)} required" + Style.RESET_ALL
            )
            self.serial = None
        print("")
        self.disable_message_callbacks = False

    def disconnect(self):
        """Disconnects from the OMM and stops the reader thread. After calling this, the instance cannot be used anymore
        """
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def check_connection(self)->bool:
        """Check if a connection to the device has been made

        Returns:
            bool: The device status
        """
        if self.serial is None:
            if self.exception_on_no_device:
                raise RuntimeError("No connection to device. Please call connect() first.")
            else:
                print(Fore.MAGENTA + "No connection to device. Please call connect() first." + Style.RESET_ALL)
                warnings.warn("No connection to device. Please call connect() first.")
                return False
        return True


    def log_msg_callback(self, log_level, msg):
        if not self.show_log_messages or self.disable_message_callbacks:
            return

        color = OpenMicroStageInterface.LOG_COLORS.get(log_level, Fore.WHITE)
        if log_level not in [SerialInterface.LogLevel.INFO, SerialInterface.LogLevel.DEBUG]:
            print(f"{color}[{log_level.name}] {msg}{Style.RESET_ALL}")
        else:
            print(f"{color}{msg}{Style.RESET_ALL}")

    def command_msg_callback(self, msg, reply_status: SerialInterface.ReplyStatus, error_msg: str):
        if not self.show_communication or self.disable_message_callbacks:
            return

        if reply_status is not None:
            if msg:
                msg = "\n".join("> " + line for line in msg.splitlines())
                print(f"{msg.rstrip()}")
            if error_msg:
                print(f"{Style.BRIGHT}{str(reply_status.name)}:{Style.RESET_ALL} {error_msg}\n")
            else:
                print(f"{Style.BRIGHT}{str(reply_status.name)} {Style.RESET_ALL}\n")
        else:
            print(f"{Fore.GREEN + Style.BRIGHT}{msg.rstrip()}{Style.RESET_ALL}")

    def unsolicited_msg_callback(self, msg):
        print(Fore.CYAN + msg + Style.RESET_ALL)
        pass

    def set_workspace_transform(self, transform):
        self.workspace_transform = transform

    def get_workspace_transform(self):
        return self.workspace_transform

    def read_firmware_version(self)->tuple[int, int, int]:
        """Reads the firmware version from the device

        Returns:
            tuple[int, int, int]: Returns the Major, Minor, and Patch version as integers. If the command fails, returns (0, 0, 0).
        """
        if not self.check_connection():
            return 0, 0, 0
        
        ok, response = self.serial.send_command("M58")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0, 0, 0

        major, minor, patch = map(int, re.match(r"v(\d+)\.(\d+)\.(\d+)", response).groups())
        return major, minor, patch

    def home(self, axis_list: list[int]=None)->SerialInterface.ReplyStatus:
        """
        Homes one or more axes on the device, one axis at a time

        Args:
            axis_list (list[int], optional): List of axis indices to home. If None, all axes are homed. Defaults to None.
        
        Returns:
            SerialInterface.ReplyStatus: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        if not self.check_connection():
            return self.serial.ReplyStatus.OK
    
        axis_chars = ["A", "B", "C", "D", "E", "F"]
        if axis_list is None:
            # Home in reverse order for safety
            axis_list = [2, 1, 0]

        # Home one axis at a time
        for axis_idx in axis_list:
            if (0 > axis_idx) or (axis_idx >= len(axis_chars)):
                raise ValueError("Axis index out of range")
            cmd = "G28 " + axis_chars[axis_idx]
            res, msg = self.serial.send_command(cmd + "\n", 10)
            if res != self.serial.ReplyStatus.OK:
                return res
        
        return self.serial.ReplyStatus.OK

    def calibrate_joint(self, joint_index: int, save_result: bool)->tuple[SerialInterface.ReplyStatus, list[list[float]]]:
        """
        Calibrates the given joint and returns the measured data as three lists containing:
            data[0]: list of motor angles
            data[1]: list of electric field angles
            data[2]: list of raw encoder counts
        """
        if not self.check_connection():
            return self.serial.ReplyStatus.OK
    
        cmd = f"M56 J{joint_index} P"
        if save_result:
            cmd += " S"
        res, msg = self.serial.send_command(cmd, 90)

        calibration_data = self._parse_table_data(msg, 3)
        return res, calibration_data

    def move_to(self, x: float, y: float, z: float, f: float, move_immediately: bool=False, blocking: bool=True, timeout: float=1)->SerialInterface.ReplyStatus:
        """
        Moves the stage to an absolute position with a specified feed rate.

        Args:
            x (float): Target X position in workspace coordinates.
            y (float): Target Y position in workspace coordinates.
            z (float): Target Z position in workspace coordinates.
            f (float): Feed rate in mm/s.
            move_immediately (bool): If True, the move command is executed immediately without waiting for previous moves
            blocking (bool): If True, the method will wait and retry if the device is busy. If False, it will return immediately on 'BUSY'.
            timeout (float): Timeout in seconds for each command attempt.

        Returns
            SerialInterface.ReplyStatus: The status of the command (e.g. OK, ERROR, TIMEOUT, BUSY).
        """
        if not self.check_connection():
            return self.serial.ReplyStatus.OK
    
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G0 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f} F{f:.3f}"
        if move_immediately:
            cmd += " I"

        # resend messages if queue is full
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res

    def dwell(self, time_s: float, blocking: bool, timeout: float=1)->SerialInterface.ReplyStatus:
        if not self.check_connection():
            return self.serial.ReplyStatus.OK

        cmd = f"G4 S{time_s:.6f}\n"
        # resend messages if queue is full
        while True:
            res, msg = self.serial.send_command(cmd + "\n", timeout=timeout)
            if res != SerialInterface.ReplyStatus.BUSY or not blocking:
                return res
        return SerialInterface.ReplyStatus.OK

    def set_max_acceleration(self, linear_accel, angular_accel):
        if not self.check_connection():
            return self.serial.ReplyStatus.OK

        linear_accel = max(linear_accel, 0.01)
        angular_accel = max(angular_accel, 0.01)
        cmd = f"M204 L{linear_accel:.6f} A{angular_accel:.6f}\n"
        res, msg = self.serial.send_command(cmd)
        return res

    def wait_for_stop(self, disable_callbacks=True)->SerialInterface.ReplyStatus:
        """Wait for the device to stop movement and complete end

        Args:
            disable_callbacks (bool, optional): If True, disables message callbacks during the wait. Defaults to True.

        Returns:
            SerialInterface.ReplyStatus: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        if not self.check_connection():
            return self.serial.ReplyStatus.OK
    
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks:
            self.disable_message_callbacks = True

        while True:
            res, msg = self.serial.send_command("M53\n")
            if res != SerialInterface.ReplyStatus.OK:
                break
            elif msg.strip() == "1":
                res = SerialInterface.ReplyStatus.OK
                break

        self.disable_message_callbacks = disable_message_callbacks_prev
        return res

    def is_stopped(self, disable_callbacks=True)->bool | None:
        """Check if the device is stopped.

        Args:
            disable_callbacks (bool, optional): If True, disables message callbacks during the check. Defaults to True.
        Returns:
            SerialInterface.ReplyStatus: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        if not self.check_connection():
            return self.serial.ReplyStatus.OK
    
        disable_message_callbacks_prev = self.disable_message_callbacks
        if disable_callbacks:
            self.disable_message_callbacks = True

        res, msg = self.serial.send_command("M53\n")
        self.disable_message_callbacks = disable_message_callbacks_prev
        if res == SerialInterface.ReplyStatus.OK:
            return msg.strip() == "1"
        else:
            return None
        
    def read_current_position(self)->tuple[float, float, float] | tuple[None, None, None]:
        """Get the current position of the device

        Returns:
            tuple[float, float, float] | tuple[None, None, None]: x,y,z floats in workspace coordinates, or None,None,None if the command failed
        """
        if not self.check_connection():
            return 0.0, 0.0, 0.0
        
        ok, response = self.serial.send_command("M50")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return None, None, None

        # Match values with NO space between axis letter and number
        match = re.search(r"X([-+]?\d*\.?\d+)\s*Y([-+]?\d*\.?\d+)\s*Z([-+]?\d*\.?\d+)", response)
        if not match:
            raise ValueError(f"Invalid format: {response}")

        x, y, z = match.groups()
        return float(x), float(y), float(z)

    def read_encoder_angles(self):
        if not self.check_connection():
            return []

        ok, response = self.serial.send_command("M51")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return []
        return []

    def read_device_state_info(self):
        if not self.check_connection():
            return SerialInterface.ReplyStatus.OK
        
        res, msg = self.serial.send_command("M57")
        return res

    def set_servo_parameter(self, pos_kp=150, pos_ki=50000, vel_kp=0.2, vel_ki=100, vel_filter_tc=0.0025):
        if not self.check_connection():
            return SerialInterface.ReplyStatus.OK
        cmd = f"M55 A{pos_kp:.6f} B{pos_ki:.6f} C{vel_kp:.6f} D{vel_ki:.6f} F{vel_filter_tc:.6f}"
        res, msg = self.serial.send_command(cmd)
        return res

    def enable_motors(self, enable):
        if not self.check_connection():
            return SerialInterface.ReplyStatus.OK
        cmd = "M17" if enable else "M18"
        res, msg = self.serial.send_command(cmd, timeout=5)
        return res

    def set_pose(self, x: float, y: float, z: float)->SerialInterface.ReplyStatus:
        """Set the pose as fast as possible

        Args:
            x (float): The x coordinate in workspace coordinates
            y (float): The y coordinate in workspace coordinates
            z (float): The z coordinate in workspace coordinates

        Returns:
            SerialInterface.ReplyStatus: The status of the command (e.g. OK, ERROR, TIMEOUT).
        """
        if not self.check_connection():
            return SerialInterface.ReplyStatus.OK
    
        # Convert to homogeneous vector
        transformed = self.workspace_transform @ np.array([x, y, z, 1.0])
        x_t, y_t, z_t = transformed[:3] / transformed[3]

        cmd = f"G24 X{x_t:.6f} Y{y_t:.6f} Z{z_t:.6f}"  # TODO: A, B ,C
        res, msg = self.serial.send_command(cmd)
        return res

    def get_temperature(self)->float:
        """Get the current temperature

        Returns:
            float: The current temperature in celsius.
        """
        if not self.check_connection():
            return 0.0
        
        ok, response = self.serial.send_command("M105")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0.0
        
        # Parse the response which should contain the temperature value
        try:
            temperature = float(response.strip())
            return temperature
        except ValueError:
            return 0.0

    def set_temperature(self, temperature: float):
        """Update the desired temperature

        Args:
            temperature (float): Value in celsius. The device will try to reach and maintain this temperature.
        """
        if not self.check_connection():
            return
        
        cmd = f"M104 S{temperature:.2f}"
        res, msg = self.serial.send_command(cmd)

    def get_vacuum(self)->bool:
        """Get the current vacuum state

        Returns:
            bool: True if the vacuum is on, False if it is off.
        """
        if not self.check_connection():
            return False
        
        ok, response = self.serial.send_command("M801")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return False
        
        # Parse the response which should contain V0 or V1
        try:
            if '1' in response:
                return True
            else:
                return False
        except ValueError:
            return False
        
    def set_vacuum(self, vacuum_on: bool):
        """Turn the vacuum on or off

        Args:
            vacuum_on (bool): If true, the vacuum will be turned on. If false, it will be turned off.
        """
        if not self.check_connection():
            return
        
        cmd = "M10" if vacuum_on else "M11"
        res, msg = self.serial.send_command(cmd)
    
    def set_rotation(self, angle: float):
        """Set the rotation angle of the stage

        Args:
            angle (float): The desired rotation angle in degrees.
        """
        if not self.check_connection():
            return
        
        cmd = f"G92 W{angle:.2f}"
        res, msg = self.serial.send_command(cmd)
    
    def get_rotation(self)->float:
        """Get the current rotation angle of the stage

        Returns:
            float: The current rotation angle in degrees.
        """
        if not self.check_connection():
            return 0.0
        
        ok, response = self.serial.send_command("M114")
        if ok != SerialInterface.ReplyStatus.OK or len(response) == 0:
            return 0.0
        
        # Parse the response which should contain X Y Z W values
        try:
            match = re.search(r"W([-+]?\d*\.?\d+)", response)
            if match:
                return float(match.group(1))
            return 0.0
        except ValueError:
            return 0.0

    def send_command(self, cmd: str, timeout_s: float = 5):
        res, msg = self.serial.send_command(cmd, timeout_s)
        return res, msg

    @staticmethod
    def _parse_table_data(data_string, cols):
        # Parse the data
        data = [[] for _ in range(cols)]

        for line in data_string.strip().splitlines():
            parts = line.strip().split(",")
            if len(parts) != cols:
                continue  # skip malformed lines
            numbers = map(float, parts)
            for i, n in enumerate(numbers):
                data[i].append(n)

        return data
