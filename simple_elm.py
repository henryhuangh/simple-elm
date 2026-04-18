from __future__ import annotations

import queue
import re
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import serial


class SimpleELMError(Exception):
    """Base exception for the simple ELM library."""


class SimpleELMValidationError(SimpleELMError):
    """Raised when a caller provides invalid command data."""


class SimpleELMTimeoutError(SimpleELMError):
    """Raised when the adapter does not respond before timeout."""


class ELMKeyword(str, Enum):
    # Basic adapter control
    RESET = "Z"
    WARM_START = "WS"
    DEFAULTS = "D"
    DESCRIBE_PROTOCOL = "DP"
    DESCRIBE_PROTOCOL_NUMBER = "DPN"
    READ_VOLTAGE = "RV"
    IDENTIFY = "I"

    # Formatting
    ECHO_OFF = "E0"
    ECHO_ON = "E1"
    LINEFEEDS_OFF = "L0"
    LINEFEEDS_ON = "L1"
    SPACES_OFF = "S0"
    SPACES_ON = "S1"
    HEADERS_OFF = "H0"
    HEADERS_ON = "H1"

    # Timing
    ADAPTIVE_TIMING_OFF = "0"
    ADAPTIVE_TIMING_AUTO1 = "1"
    ADAPTIVE_TIMING_AUTO2 = "2"
    SET_TIMEOUT = "ST"

    # Protocol
    SET_PROTOCOL = "SP"
    TRY_PROTOCOL = "TP"

    # CAN / OBD framing
    SET_HEADER = "SH"
    CAN_RECEIVE_ADDRESS = "CRA"
    CAN_AUTO_FORMAT_OFF = "CAF0"
    CAN_AUTO_FORMAT_ON = "CAF1"
    MONITOR_ALL = "MA"
    MONITOR_RECEIVER = "MR"
    MONITOR_TRANSMITTER = "MT"

    # Memory
    MEMORY_OFF = "M0"
    MEMORY_ON = "M1"


class OBDMode(str, Enum):
    CURRENT_DATA = "01"
    FREEZE_FRAME = "02"
    STORED_DTCS = "03"
    CLEAR_DTCS = "04"
    OXYGEN_TEST = "05"
    ONBOARD_MONITOR = "06"
    PENDING_DTCS = "07"
    VEHICLE_INFO = "09"
    PERMANENT_DTCS = "0A"


class OBDPid(str, Enum):
    ENGINE_LOAD = "04"
    COOLANT_TEMP = "05"
    ENGINE_RPM = "0C"
    VEHICLE_SPEED = "0D"
    MAF_AIR_FLOW = "10"
    THROTTLE_POSITION = "11"
    INTAKE_TEMP = "0F"
    FUEL_LEVEL = "2F"


@dataclass(frozen=True)
class CANFrame:
    """One parsed CAN frame seen during monitoring."""

    can_id: str
    data: str
    raw: str
    timestamp: float


class CANBuffer:
    """Thread-safe queue wrapper for monitor output."""

    def __init__(self, maxsize: int = 0) -> None:
        self._queue: queue.Queue[CANFrame] = queue.Queue(maxsize=maxsize)

    def put(self, frame: CANFrame) -> None:
        self._queue.put(frame)

    def get(self, timeout: Optional[float] = None) -> CANFrame:
        return self._queue.get(timeout=timeout)

    def get_nowait(self) -> CANFrame:
        return self._queue.get_nowait()

    def empty(self) -> bool:
        return self._queue.empty()

    def qsize(self) -> int:
        return self._queue.qsize()


class ELM327:
    """
    Small ELM327 helper library.

    Features:
    - PID requests with request/response
    - send raw CAN/diagnostic data
    - set header / receive filter / formatting helpers
    - background CAN monitor thread that pushes parsed frames into a queue buffer
    """

    PROMPT = b">"

    def __init__(
        self,
        port: str,
        baudrate: int = 38400,
        timeout: float = 0.2,
        monitor_buffer_size: int = 0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

        self.current_header: Optional[str] = None
        self.current_receive_filter: Optional[str] = None

        self._io_lock = threading.RLock()
        self._monitor_stop = threading.Event()
        self._monitor_thread: Optional[threading.Thread] = None
        self.monitor_buffer = CANBuffer(maxsize=monitor_buffer_size)

    # ---------------------------------------------------------------------
    # Lifecycle
    # ---------------------------------------------------------------------

    def close(self) -> None:
        self.stop_monitor()
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "ELM327":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ---------------------------------------------------------------------
    # Validation helpers
    # ---------------------------------------------------------------------

    @staticmethod
    def _clean_hex(value: str, field_name: str) -> str:
        if not isinstance(value, str):
            raise TypeError(f"{field_name} must be a string")
        cleaned = value.replace(" ", "").upper()
        if not cleaned:
            raise SimpleELMValidationError(f"{field_name} cannot be empty")
        if not re.fullmatch(r"[0-9A-F]+", cleaned):
            raise SimpleELMValidationError(f"{field_name} must be hex")
        return cleaned

    def _validate_hex(self, value: str, allowed_lengths: set[int], field_name: str) -> str:
        cleaned = self._clean_hex(value, field_name)
        if len(cleaned) not in allowed_lengths:
            allowed = ", ".join(str(x) for x in sorted(allowed_lengths))
            raise SimpleELMValidationError(
                f"{field_name} must be one of these lengths: {allowed} hex chars"
            )
        return cleaned

    def _validate_even_hex(self, value: str, field_name: str, min_len: int = 2) -> str:
        cleaned = self._clean_hex(value, field_name)
        if len(cleaned) < min_len:
            raise SimpleELMValidationError(
                f"{field_name} must be at least {min_len} hex chars"
            )
        if len(cleaned) % 2 != 0:
            raise SimpleELMValidationError(
                f"{field_name} must contain an even number of hex chars"
            )
        return cleaned

    def _validate_protocol(self, protocol: str | int) -> str:
        if isinstance(protocol, int):
            if not 0 <= protocol <= 0x0C:
                raise SimpleELMValidationError(
                    "protocol must be between 0 and 12")
            return str(protocol) if protocol < 10 else chr(ord("A") + protocol - 10)

        if not isinstance(protocol, str):
            raise TypeError("protocol must be str or int")

        cleaned = protocol.strip().upper()
        if not re.fullmatch(r"[0-9A-C]", cleaned):
            raise SimpleELMValidationError(
                "protocol must be one hex digit: 0-9 or A-C")
        return cleaned

    def _validate_timeout_byte(self, value: int) -> str:
        if not isinstance(value, int):
            raise TypeError("timeout value must be int")
        if not 0 <= value <= 0xFF:
            raise SimpleELMValidationError(
                "timeout value must be between 0 and 255")
        return f"{value:02X}"

    # ---------------------------------------------------------------------
    # Low-level I/O
    # ---------------------------------------------------------------------

    def flush_input(self) -> None:
        with self._io_lock:
            self.ser.reset_input_buffer()

    def flush_output(self) -> None:
        with self._io_lock:
            self.ser.reset_output_buffer()

    def _write_line(self, text: str) -> None:
        self.ser.write((text + "\r").encode("ascii"))
        self.ser.flush()

    def _read_until_prompt(self, timeout: Optional[float] = None) -> str:
        deadline = time.monotonic() + (timeout if timeout is not None else self.timeout)
        buf = bytearray()

        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            if waiting:
                chunk = self.ser.read(waiting)
            else:
                chunk = self.ser.read(1)

            if chunk:
                buf.extend(chunk)
                if self.PROMPT in buf:
                    break
            else:
                time.sleep(0.005)

        if self.PROMPT not in buf:
            raise SimpleELMTimeoutError("Timed out waiting for ELM prompt")

        text = buf.decode("ascii", errors="replace")
        text = text.replace(">", "")
        return text.strip()

    def _clean_response(self, sent: str, response: str) -> str:
        lines = [line.strip()
                 for line in response.splitlines() if line.strip()]
        if lines and lines[0].replace(" ", "").upper() == sent.replace(" ", "").upper():
            lines = lines[1:]
        return "\n".join(lines).strip()

    def _query(self, text: str, timeout: Optional[float] = None) -> str:
        with self._io_lock:
            self.flush_input()
            self._write_line(text)
            raw = self._read_until_prompt(timeout=timeout)
            return self._clean_response(text, raw)

    # ---------------------------------------------------------------------
    # Generic send helpers
    # ---------------------------------------------------------------------

    def send_at(self, body_or_command: str) -> str:
        """
        Send an AT command.

        Accepts either:
        - "ATZ"
        - "Z"
        - "SH7E0"
        """
        if not isinstance(body_or_command, str):
            raise TypeError("AT command must be a string")

        cleaned = body_or_command.strip().upper().replace(" ", "")
        full = cleaned if cleaned.startswith("AT") else f"AT{cleaned}"
        return self._query(full)

    def send_data(self, data: str, header: Optional[str] = None) -> str:
        """
        Send raw diagnostic/CAN payload data and return the adapter response.

        If header is provided, it is sent first with ATSH.
        """
        data_clean = self._validate_even_hex(data, "data")
        if header is not None:
            self.set_header(header)
        return self._query(data_clean)

    def send_can(self, header: str, data: str) -> str:
        """Convenience wrapper to send data to a specific CAN header."""
        return self.send_data(data=data, header=header)

    # ---------------------------------------------------------------------
    # PID requests
    # ---------------------------------------------------------------------

    def request_pid(
        self,
        pid: OBDPid | str,
        mode: OBDMode | str = OBDMode.CURRENT_DATA,
        header: Optional[str] = None,
    ) -> str:
        mode_hex = mode.value if isinstance(
            mode, OBDMode) else self._validate_hex(mode, {2}, "mode")
        pid_hex = pid.value if isinstance(
            pid, OBDPid) else self._validate_hex(pid, {2}, "pid")
        return self.send_data(mode_hex + pid_hex, header=header)

    # ---------------------------------------------------------------------
    # Common adapter helpers
    # ---------------------------------------------------------------------

    def reset(self) -> str:
        return self.send_at(ELMKeyword.RESET.value)

    def warm_start(self) -> str:
        return self.send_at(ELMKeyword.WARM_START.value)

    def defaults(self) -> str:
        return self.send_at(ELMKeyword.DEFAULTS.value)

    def identify(self) -> str:
        return self.send_at(ELMKeyword.IDENTIFY.value)

    def describe_protocol(self) -> str:
        return self.send_at(ELMKeyword.DESCRIBE_PROTOCOL.value)

    def describe_protocol_number(self) -> str:
        return self.send_at(ELMKeyword.DESCRIBE_PROTOCOL_NUMBER.value)

    def read_voltage(self) -> str:
        return self.send_at(ELMKeyword.READ_VOLTAGE.value)

    def set_echo(self, enabled: bool) -> str:
        return self.send_at(
            ELMKeyword.ECHO_ON.value if enabled else ELMKeyword.ECHO_OFF.value
        )

    def set_linefeeds(self, enabled: bool) -> str:
        return self.send_at(
            ELMKeyword.LINEFEEDS_ON.value if enabled else ELMKeyword.LINEFEEDS_OFF.value
        )

    def set_spaces(self, enabled: bool) -> str:
        return self.send_at(
            ELMKeyword.SPACES_ON.value if enabled else ELMKeyword.SPACES_OFF.value
        )

    def set_headers(self, enabled: bool) -> str:
        return self.send_at(
            ELMKeyword.HEADERS_ON.value if enabled else ELMKeyword.HEADERS_OFF.value
        )

    def adaptive_timing_off(self) -> str:
        return self.send_at(ELMKeyword.ADAPTIVE_TIMING_OFF.value)

    def adaptive_timing_auto1(self) -> str:
        return self.send_at(ELMKeyword.ADAPTIVE_TIMING_AUTO1.value)

    def adaptive_timing_auto2(self) -> str:
        return self.send_at(ELMKeyword.ADAPTIVE_TIMING_AUTO2.value)

    def set_timeout(self, value: int) -> str:
        timeout_hex = self._validate_timeout_byte(value)
        return self.send_at(f"{ELMKeyword.SET_TIMEOUT.value}{timeout_hex}")

    def set_protocol(self, protocol: str | int) -> str:
        protocol_str = self._validate_protocol(protocol)
        return self.send_at(f"{ELMKeyword.SET_PROTOCOL.value}{protocol_str}")

    def try_protocol(self, protocol: str | int) -> str:
        protocol_str = self._validate_protocol(protocol)
        return self.send_at(f"{ELMKeyword.TRY_PROTOCOL.value}{protocol_str}")

    def set_header(self, header: str) -> str:
        header_clean = self._validate_hex(header, {3, 6, 8}, "header")
        self.current_header = header_clean
        return self.send_at(f"{ELMKeyword.SET_HEADER.value}{header_clean}")

    def set_can_receive_filter(self, can_id: str) -> str:
        can_id_clean = self._validate_hex(can_id, {3, 8}, "CAN receive filter")
        self.current_receive_filter = can_id_clean
        return self.send_at(f"{ELMKeyword.CAN_RECEIVE_ADDRESS.value}{can_id_clean}")

    def clear_can_receive_filter(self) -> str:
        self.current_receive_filter = None
        return self.send_at(ELMKeyword.CAN_RECEIVE_ADDRESS.value)

    def set_can_auto_format(self, enabled: bool) -> str:
        return self.send_at(
            ELMKeyword.CAN_AUTO_FORMAT_ON.value if enabled else ELMKeyword.CAN_AUTO_FORMAT_OFF.value
        )

    def set_memory(self, enabled: bool) -> str:
        return self.send_at(
            ELMKeyword.MEMORY_ON.value if enabled else ELMKeyword.MEMORY_OFF.value
        )

    def initialize(self, protocol: str | int = 0, headers: bool = True) -> list[str]:
        """Common startup sequence."""
        responses = [
            self.reset(),
            self.set_echo(False),
            self.set_linefeeds(False),
            self.set_spaces(False),
            self.set_headers(headers),
            self.set_can_auto_format(True),
            self.adaptive_timing_auto1(),
            self.set_protocol(protocol),
        ]
        return responses

    # ---------------------------------------------------------------------
    # Monitoring / streaming
    # ---------------------------------------------------------------------

    @staticmethod
    def _parse_monitor_line(line: str) -> Optional[CANFrame]:
        cleaned = line.strip()
        if not cleaned:
            return None
        if cleaned.upper() in {"OK", "STOPPED", "SEARCHING...", "NO DATA", "?"}:
            return None

        parts = cleaned.split()
        if len(parts) < 2:
            return None

        can_id = parts[0].upper()
        if not re.fullmatch(r"[0-9A-F]+", can_id):
            return None

        data = "".join(parts[1:]).upper()
        if not re.fullmatch(r"[0-9A-F]+", data):
            return None

        return CANFrame(
            can_id=can_id,
            data=data,
            raw=cleaned,
            timestamp=time.time(),
        )

    def start_monitor_all(self) -> CANBuffer:
        if self._monitor_thread and self._monitor_thread.is_alive():
            raise SimpleELMError("Monitor is already running")

        # Keep monitor output parse-friendly (ID + spaced bytes).
        self.set_headers(True)
        self.set_spaces(True)
        self._monitor_stop.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=("ATMA",),
            daemon=True,
            name="simple-elm-monitor",
        )
        self._monitor_thread.start()
        return self.monitor_buffer

    def start_monitor_can_id(self, can_id: str) -> CANBuffer:
        """
        Monitor frames by CAN ID using CRA filter + MA.
        Good for simple streaming of one responder ID.
        """
        if self._monitor_thread and self._monitor_thread.is_alive():
            raise SimpleELMError("Monitor is already running")

        can_id_clean = self._validate_hex(can_id, {3, 8}, "CAN ID")
        # Match monitor_lights framing so monitor lines parse consistently.
        self.set_headers(True)
        self.set_spaces(True)
        self.set_can_receive_filter(can_id_clean)

        self._monitor_stop.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=("ATMA",),
            daemon=True,
            name="simple-elm-monitor",
        )
        self._monitor_thread.start()
        return self.monitor_buffer

    def _monitor_loop(self, monitor_command: str) -> None:
        with self._io_lock:
            self.flush_input()
            self._write_line(monitor_command)

        buffer = ""

        try:
            while not self._monitor_stop.is_set():
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue

                text = chunk.decode("ascii", errors="replace")
                saw_prompt = ">" in text
                if saw_prompt:
                    text = text.split(">", 1)[0]

                if text:
                    buffer += text

                # ELM monitor output is CR/LF separated; parse one line at a time.
                while "\r" in buffer or "\n" in buffer:
                    m = re.search(r"[\r\n]", buffer)
                    if not m:
                        break
                    idx = m.start()
                    line = buffer[:idx]
                    buffer = buffer[idx + 1:]
                    frame = self._parse_monitor_line(line)
                    if frame is not None:
                        self.monitor_buffer.put(frame)

                if saw_prompt:
                    break
        finally:
            if buffer.strip():
                line = buffer
                frame = self._parse_monitor_line(line)
                if frame is not None:
                    self.monitor_buffer.put(frame)

    def stop_monitor(self) -> None:
        if not (self._monitor_thread and self._monitor_thread.is_alive()):
            return

        self._monitor_stop.set()

        with self._io_lock:
            self.ser.write(b" ")
            self.ser.flush()

            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    if self.PROMPT in data:
                        break
                else:
                    time.sleep(0.01)

        self._monitor_thread.join(timeout=1.0)
        self._monitor_thread = None


if __name__ == "__main__":
    # Minimal example usage.
    with ELM327(port="COM9", baudrate=38400, timeout=1.0) as elm:
        elm.initialize(protocol=6, headers=True)

        # Standard PID request.
        rpm_response = elm.request_pid(OBDPid.ENGINE_RPM, header="7DF")
        print("RPM response:", rpm_response)

        # Send raw CAN/diagnostic data to a specific header.
        response = elm.send_can(header="7E0", data="010C")
        print("Direct response:", response)

        # Start background streaming.
        buf = elm.start_monitor_can_id("60D")
        time.sleep(5.0)
        elm.stop_monitor()

        while not buf.empty():
            frame = buf.get_nowait()
            print(frame)
