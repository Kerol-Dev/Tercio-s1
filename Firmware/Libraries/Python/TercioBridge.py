"""Tercio host library — CAN protocol v2, adapter serial framing v2.

Talks to Tercio S1 drivers through the Tercio FD USB-CAN adapter. On the
serial link every frame is ``[id u16][opcode u8][payload][crc16 u16]``
(little-endian), COBS-encoded and terminated by a 0x00 byte; id 0x800
addresses the adapter itself (bus health, info, firmware-update mode).

Quick start::

    from TercioBridge import Bus, Stepper, Unit

    with Bus() as bus:                       # first serial port, or Bus("COM5")
        for info in bus.discover():
            print(info)
        motor = Stepper(bus, 1, unit=Unit.DEGREES)
        motor.enable()
        motor.move_to(90, wait=True)
        print(motor.position, motor.telemetry.state)

The wire contracts are ``Tercio-s1/Firmware/src/protocol/Protocol.h`` (CAN) and
``Tercio-fdcan/Firmware/src/bridge/{Framing,AdapterProtocol}.h`` (serial); keep
them in sync.
"""
from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass, field
from enum import Enum, IntEnum, IntFlag
from typing import Callable, Dict, List, Optional, Tuple, Union

import serial
from serial.tools import list_ports

PROTOCOL_VERSION = 2

# ---------------------------------------------------------------- identifiers

BROADCAST_ID = 0x000
FN_EVENT = 0x080
FN_COMMAND = 0x100
FN_REPLY = 0x180
FN_TELEMETRY = 0x200
MAX_NODE_ID = 127

NO_REPLY = 0x80
MOVE_DEFERRED = 0x01

FRAME_TELEMETRY = 0x01
FRAME_FAULT = 0x02


class Cmd(IntEnum):
    GET_INFO = 0x00
    SAVE_CONFIG = 0x01
    FACTORY_RESET = 0x02
    REBOOT = 0x03
    CLEAR_FAULTS = 0x04
    ASSIGN_NODE_ID = 0x05
    GET_PARAM = 0x08
    SET_PARAM = 0x09
    ENABLE = 0x10
    STOP = 0x11
    EMERGENCY_STOP = 0x12
    MOVE_TO = 0x13
    MOVE_BY = 0x14
    SET_VELOCITY = 0x15
    SET_ZERO = 0x16
    SYNC = 0x17
    CALIBRATE = 0x20
    HOME = 0x21
    AUTO_TUNE = 0x22


class Status(IntEnum):
    OK = 0
    UNKNOWN_COMMAND = 1
    BAD_LENGTH = 2
    BAD_VALUE = 3
    UNKNOWN_PARAM = 4
    READ_ONLY = 5
    BUSY = 6
    NOT_CALIBRATED = 7
    FAULTED = 8
    DISABLED = 9
    STORAGE_ERROR = 10


class AxisState(IntEnum):
    DISABLED = 0
    HOLDING = 1
    MOVING = 2
    VELOCITY = 3
    STEP_DIR = 4
    CALIBRATING = 5
    HOMING = 6
    TUNING = 7
    FAULT = 8


class Fault(IntFlag):
    OVER_TEMPERATURE = 1 << 0
    DRIVER_OVER_TEMP = 1 << 1
    DRIVER_FAULT = 1 << 2
    DRIVER_COMM = 1 << 3
    ENCODER = 1 << 4
    STALL = 1 << 5
    CALIBRATION_FAILED = 1 << 6
    HOMING_FAILED = 1 << 7
    COMMAND_TIMEOUT = 1 << 8


class Warning_(IntFlag):
    TEMPERATURE_HIGH = 1 << 0
    DRIVER_PRE_WARNING = 1 << 1
    MAGNET_WEAK = 1 << 2
    MAGNET_STRONG = 1 << 3
    SUPPLY_LOW = 1 << 4
    CAN_ERROR_PASSIVE = 1 << 5
    OPEN_LOAD = 1 << 6
    STORAGE = 1 << 7


class Flag(IntFlag):
    ENABLED = 1 << 0
    CALIBRATED = 1 << 1
    HOMED = 1 << 2
    SETTLED = 1 << 3
    LIMIT_MIN = 1 << 4
    LIMIT_MAX = 1 << 5
    MOVE_PENDING = 1 << 6
    EXT_ENABLE = 1 << 7


class Param(IntEnum):
    NODE_ID = 0x01
    TELEMETRY_RATE_HZ = 0x02
    COMMAND_TIMEOUT_MS = 0x03
    FULL_STEPS_PER_REV = 0x10
    MICROSTEPS = 0x11
    RUN_CURRENT_MA = 0x12
    HOLD_CURRENT_PCT = 0x13
    STEALTHCHOP = 0x14
    STEALTHCHOP_MAX_VEL = 0x15
    INVERT_DIRECTION = 0x16
    GEAR_RATIO = 0x17
    MAX_VELOCITY = 0x20
    MAX_ACCELERATION = 0x21
    KP = 0x22
    KI = 0x23
    KD = 0x24
    POSITION_DEADBAND = 0x25
    FOLLOWING_ERROR_LIMIT = 0x26
    STALL_TIMEOUT_MS = 0x27
    SOFT_LIMIT_MIN = 0x28
    SOFT_LIMIT_MAX = 0x29
    ENABLE_ON_BOOT = 0x2A
    ENCODER_TYPE = 0x30
    ENCODER_INVERT = 0x31
    CALIBRATED = 0x32
    LIMIT_SWITCHES_ENABLED = 0x40
    LIMIT_SWITCH_ACTIVE_LOW = 0x41
    STEP_DIR_MODE = 0x42
    STEP_DIR_ENABLE_ACTIVE_LOW = 0x43
    HOMING_MODE = 0x50
    HOMING_VELOCITY = 0x51
    HOMING_CURRENT_MA = 0x52
    HOMING_BACKOFF = 0x53
    HOMING_STALL_ERROR = 0x54
    HOMING_TIMEOUT_S = 0x55
    OVER_TEMPERATURE_C = 0x60


# Value type per parameter: "f" float, "b" bool, "i" unsigned integer.
_FLOAT_PARAMS = {
    Param.STEALTHCHOP_MAX_VEL, Param.GEAR_RATIO, Param.MAX_VELOCITY, Param.MAX_ACCELERATION,
    Param.KP, Param.KI, Param.KD, Param.POSITION_DEADBAND, Param.FOLLOWING_ERROR_LIMIT,
    Param.SOFT_LIMIT_MIN, Param.SOFT_LIMIT_MAX, Param.HOMING_VELOCITY, Param.HOMING_BACKOFF,
    Param.HOMING_STALL_ERROR, Param.OVER_TEMPERATURE_C,
}
_BOOL_PARAMS = {
    Param.STEALTHCHOP, Param.INVERT_DIRECTION, Param.ENABLE_ON_BOOT, Param.ENCODER_INVERT,
    Param.CALIBRATED, Param.LIMIT_SWITCHES_ENABLED, Param.LIMIT_SWITCH_ACTIVE_LOW,
    Param.STEP_DIR_MODE, Param.STEP_DIR_ENABLE_ACTIVE_LOW,
}


class EncoderType(IntEnum):
    AS5600_INTERNAL = 0
    AS5600_EXTERNAL = 1
    AS5048A = 2


class HomingMode(IntEnum):
    SWITCH_MIN = 0
    SWITCH_MAX = 1
    SENSORLESS_NEGATIVE = 2
    SENSORLESS_POSITIVE = 3


class Unit(Enum):
    """Position unit for a Stepper. The wire always carries turns."""

    TURNS = 1.0
    DEGREES = 360.0
    RADIANS = 2.0 * math.pi


class TercioError(RuntimeError):
    def __init__(self, status: Union[Status, str], command: Optional[Cmd] = None, node: Optional[int] = None):
        self.status = status
        where = f" (node {node}, {command.name})" if command is not None else ""
        name = status.name if isinstance(status, Status) else status
        super().__init__(f"{name}{where}")


# ------------------------------------------------------------------ payloads

_TELEMETRY = struct.Struct("<BBHHddffhHBBB")  # 37 bytes, see Protocol.h
_INFO = struct.Struct("<BBBBBBBB12s")         # 20 bytes


@dataclass
class Telemetry:
    """Latest periodic status of one node. Positions/velocities in turns."""

    state: AxisState
    flags: Flag
    faults: Fault
    warnings: Warning_
    position: float
    target: float
    velocity: float
    following_error: float
    temperature_c: float
    supply_v: float
    procedure_step: int
    sequence: int
    control_load_pct: int
    timestamp: float = field(default_factory=time.monotonic)

    @property
    def enabled(self) -> bool:
        return Flag.ENABLED in self.flags

    @property
    def calibrated(self) -> bool:
        return Flag.CALIBRATED in self.flags

    @property
    def settled(self) -> bool:
        return Flag.SETTLED in self.flags


@dataclass
class NodeInfo:
    node_id: int
    protocol: int
    firmware: str
    hardware_revision: int
    encoder: EncoderType
    uid: bytes
    crystal_clock: bool = False

    def __str__(self) -> str:
        clock = "crystal" if self.crystal_clock else "HSI clock"
        return (f"node {self.node_id}: fw {self.firmware}, protocol {self.protocol}, hw rev "
                f"{self.hardware_revision}, {self.encoder.name}, {clock}, uid {self.uid.hex()}")


def _parse_info(data: bytes) -> NodeInfo:
    proto, major, minor, patch, hw, enc, node, flags, uid = _INFO.unpack_from(data)
    encoder = EncoderType(enc) if enc in EncoderType._value2member_map_ else EncoderType.AS5600_INTERNAL
    return NodeInfo(node, proto, f"{major}.{minor}.{patch}", hw, encoder, bytes(uid), bool(flags & 1))


def _parse_telemetry(data: bytes) -> Optional[Telemetry]:
    if len(data) < _TELEMETRY.size:
        return None
    (state, flags, faults, warnings, position, target, velocity, error, temp, supply,
     step, sequence, load) = _TELEMETRY.unpack_from(data)
    return Telemetry(
        state=AxisState(state) if state in AxisState._value2member_map_ else AxisState.FAULT,
        flags=Flag(flags), faults=Fault(faults), warnings=Warning_(warnings),
        position=position, target=target, velocity=velocity, following_error=error,
        temperature_c=temp / 10.0, supply_v=supply / 1000.0,
        procedure_step=step, sequence=sequence, control_load_pct=load,
    )


def _encode_param(param: Param, value: Union[float, int, bool]) -> bytes:
    if param in _FLOAT_PARAMS:
        return struct.pack("<f", float(value))
    return struct.pack("<I", int(bool(value)) if param in _BOOL_PARAMS else int(value))


def _decode_param(param: Param, raw: bytes) -> Union[float, int, bool]:
    if param in _FLOAT_PARAMS:
        return struct.unpack("<f", raw)[0]
    value = struct.unpack("<I", raw)[0]
    return bool(value) if param in _BOOL_PARAMS else value


# ------------------------------------------------------------ serial framing

ADAPTER_ID = 0x800  # serial id of the adapter itself (outside the 11-bit CAN range)
MAX_PAYLOAD = 63


def _crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _cobs_encode(data: bytes) -> bytes:
    out = bytearray(b"\x00")
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index, code = len(out), 1
                out.append(0)
    out[code_index] = code
    return bytes(out)


def _cobs_decode(data: bytes) -> Optional[bytes]:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            return None
        out += data[i:i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def _encode_frame(can_id: int, opcode: int, payload: bytes) -> bytes:
    packet = struct.pack("<HB", can_id, opcode & 0xFF) + payload
    return _cobs_encode(packet + struct.pack("<H", _crc16(packet))) + b"\x00"


def _decode_frame(chunk: bytes) -> Optional[Tuple[int, int, bytes]]:
    packet = _cobs_decode(chunk)
    if packet is None or not 5 <= len(packet) <= 5 + MAX_PAYLOAD:
        return None
    if struct.unpack_from("<H", packet, len(packet) - 2)[0] != _crc16(packet[:-2]):
        return None
    can_id, opcode = struct.unpack_from("<HB", packet)
    if can_id > 0x7FF and can_id != ADAPTER_ID:
        return None
    return can_id, opcode, packet[3:-2]


# ------------------------------------------------------------------- adapter

class AdapterOp(IntEnum):
    GET_INFO = 0x00
    GET_STATUS = 0x01
    RESET_COUNTERS = 0x02
    ENTER_BOOTLOADER = 0x03


class BusState(IntEnum):
    ERROR_ACTIVE = 0
    WARNING = 1
    ERROR_PASSIVE = 2
    BUS_OFF = 3


_ADAPTER_INFO = struct.Struct("<BBBBBBH12s")      # 20 bytes
_ADAPTER_STATUS = struct.Struct("<BBBBIIIIIHH")   # 28 bytes


@dataclass
class AdapterInfo:
    protocol: int
    firmware: str
    hardware_revision: int
    crystal_clock: bool
    boot_options_ok: bool
    uid: bytes


@dataclass
class AdapterStatus:
    """Bus health as seen by the adapter; sent every 250 ms while the port is open."""

    bus_state: BusState
    tx_error_count: int
    rx_error_count: int
    last_error: int
    to_can: int            # frames delivered to the bus (acknowledged)
    from_can: int          # frames received from the bus
    dropped_to_can: int    # queue full, or not deliverable within 50 ms (e.g. no node listening)
    dropped_from_can: int  # this host did not read fast enough
    framing_errors: int    # damaged serial frames from this host
    bus_off_events: int
    timestamp: float = field(default_factory=time.monotonic)


# ----------------------------------------------------------------------- bus

@dataclass
class _Pending:
    event: threading.Event = field(default_factory=threading.Event)
    status: Status = Status.OK
    data: bytes = b""


class Bus:
    """Serial connection to the USB-CAN adapter, shared by all nodes on the bus."""

    def __init__(self, port: Optional[str] = None, baud: int = 115200, timeout: float = 0.2):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional[serial.Serial] = None
        self._reader: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._rx = bytearray()
        self._write_lock = threading.Lock()
        self._lock = threading.Lock()
        self._telemetry: Dict[int, Telemetry] = {}
        self._pending: Dict[Tuple[int, int], _Pending] = {}
        self._discovered: Dict[bytes, NodeInfo] = {}
        self._legacy: Dict[int, "ImuState"] = {}
        self._adapter_status: Optional[AdapterStatus] = None
        self.framing_errors = 0  # damaged frames received from the adapter
        self.on_fault: Optional[Callable[[int, Fault, Warning_, AxisState], None]] = None

    # -- lifecycle
    def open(self) -> "Bus":
        port = self.port or self._first_port()
        self._serial = serial.Serial(port, self.baud, timeout=0.02)
        self.port = port
        self._stop.clear()
        self._reader = threading.Thread(target=self._read_loop, name="tercio-rx", daemon=True)
        self._reader.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._reader:
            self._reader.join(timeout=1.0)
        if self._serial:
            self._serial.close()
        self._serial = None

    def __enter__(self) -> "Bus":
        return self.open() if self._serial is None else self

    def __exit__(self, *exc) -> None:
        self.close()

    @staticmethod
    def _first_port() -> str:
        ports = list(list_ports.comports())
        if not ports:
            raise TercioError("no serial port found")
        return ports[0].device

    # -- transmit
    def send(self, can_id: int, opcode: int, payload: bytes = b"") -> None:
        if self._serial is None:
            raise TercioError("bus not open")
        if not (0 <= can_id <= 0x7FF or can_id == ADAPTER_ID) or len(payload) > MAX_PAYLOAD:
            raise ValueError("invalid CAN id or payload too long")
        frame = _encode_frame(can_id, opcode, payload)
        with self._write_lock:
            self._serial.write(frame)

    def _await(self, key: Tuple[int, int], transmit: Callable[[], None], timeout: Optional[float]) -> _Pending:
        pending = _Pending()
        with self._lock:
            self._pending[key] = pending
        try:
            transmit()
            if not pending.event.wait(self.timeout if timeout is None else timeout):
                raise TercioError("timeout")
        finally:
            with self._lock:
                if self._pending.get(key) is pending:
                    del self._pending[key]
        return pending

    def request(self, node: int, cmd: Cmd, payload: bytes = b"", timeout: Optional[float] = None) -> bytes:
        """Sends a command and waits for its reply. Returns the reply data; raises on error status."""
        try:
            pending = self._await((node, int(cmd)), lambda: self.send(FN_COMMAND + node, int(cmd), payload), timeout)
        except TercioError:
            raise TercioError("timeout", cmd, node) from None
        if pending.status != Status.OK:
            raise TercioError(pending.status, cmd, node)
        return pending.data

    def command(self, node: int, cmd: Cmd, payload: bytes = b"") -> None:
        """Fire-and-forget: the node only answers if the command fails."""
        self.send(FN_COMMAND + node, int(cmd) | NO_REPLY, payload)

    # -- broadcast
    def discover(self, wait: float = 0.15) -> List[NodeInfo]:
        """Asks every node for its info. Nodes answer staggered, so duplicates show up too."""
        with self._lock:
            self._discovered.clear()
        self.send(BROADCAST_ID, Cmd.GET_INFO)
        time.sleep(wait)
        with self._lock:
            return sorted(self._discovered.values(), key=lambda i: (i.node_id, i.uid))

    def assign_node_id(self, uid: bytes, new_id: int) -> None:
        if not 1 <= new_id <= MAX_NODE_ID or len(uid) != 12:
            raise ValueError("uid must be 12 bytes and new_id 1..127")
        self.send(BROADCAST_ID, Cmd.ASSIGN_NODE_ID, bytes(uid) + bytes([new_id]))

    def stop_all(self) -> None:
        self.send(BROADCAST_ID, Cmd.STOP)

    def emergency_stop_all(self) -> None:
        self.send(BROADCAST_ID, Cmd.EMERGENCY_STOP)

    def sync(self) -> None:
        """Starts every move queued with deferred=True, on all nodes at once."""
        self.send(BROADCAST_ID, Cmd.SYNC)

    def telemetry(self, node: int) -> Optional[Telemetry]:
        with self._lock:
            return self._telemetry.get(node)

    # -- the adapter itself
    def _adapter_request(self, op: AdapterOp, timeout: Optional[float] = None) -> bytes:
        return self._await((ADAPTER_ID, int(op)), lambda: self.send(ADAPTER_ID, int(op)), timeout).data

    def adapter_info(self) -> AdapterInfo:
        proto, major, minor, patch, hw, flags, _, uid = _ADAPTER_INFO.unpack_from(
            self._adapter_request(AdapterOp.GET_INFO))
        return AdapterInfo(proto, f"{major}.{minor}.{patch}", hw, bool(flags & 1), bool(flags & 2), bytes(uid))

    @property
    def adapter_status(self) -> Optional[AdapterStatus]:
        """Latest bus-health report from the adapter (refreshed every 250 ms)."""
        with self._lock:
            return self._adapter_status

    def reset_adapter_counters(self) -> None:
        self._adapter_request(AdapterOp.RESET_COUNTERS)

    def enter_adapter_bootloader(self) -> None:
        """Restarts the adapter into ST's USB DFU bootloader (for a firmware update).
        The serial port disappears; flash with STM32CubeProgrammer or dfu-util."""
        self._adapter_request(AdapterOp.ENTER_BOOTLOADER)
        self.close()

    def nodes(self) -> List[int]:
        """Nodes that sent telemetry recently."""
        now = time.monotonic()
        with self._lock:
            return sorted(n for n, t in self._telemetry.items() if now - t.timestamp < 1.0)

    # -- receive
    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._serial.read(512) if self._serial else b""
            except serial.SerialException:
                return
            if chunk:
                self._rx.extend(chunk)
                self._parse()

    def _parse(self) -> None:
        rx = self._rx
        while True:
            end = rx.find(0)  # 0x00 only ever appears as the frame delimiter
            if end < 0:
                if len(rx) > 1024:  # no delimiter in far too long: not our stream
                    rx.clear()
                    self.framing_errors += 1
                return
            chunk = bytes(rx[:end])
            del rx[:end + 1]
            if not chunk:
                continue
            frame = _decode_frame(chunk)
            if frame is None:
                self.framing_errors += 1  # e.g. the tail of a frame from before the port was opened
            else:
                self._dispatch(*frame)

    def _dispatch_adapter(self, opcode: int, payload: bytes) -> None:
        if opcode in (AdapterOp.GET_STATUS, AdapterOp.RESET_COUNTERS) and len(payload) >= _ADAPTER_STATUS.size:
            state, tec, rec, lec, to_can, from_can, dropped_to, dropped_from, framing, bus_off, _ = \
                _ADAPTER_STATUS.unpack_from(payload)
            status = AdapterStatus(BusState(state) if state <= 3 else BusState.BUS_OFF, tec, rec, lec, to_can,
                                   from_can, dropped_to, dropped_from, framing, bus_off)
            with self._lock:
                self._adapter_status = status
        with self._lock:
            pending = self._pending.get((ADAPTER_ID, opcode))
        if pending:
            pending.data = payload
            pending.event.set()

    def _dispatch(self, can_id: int, opcode: int, payload: bytes) -> None:
        if can_id == ADAPTER_ID:
            self._dispatch_adapter(opcode, payload)
            return
        function, node = can_id & 0x780, can_id & 0x7F
        if function == FN_REPLY and len(payload) >= 1:
            status = Status(payload[0]) if payload[0] in Status._value2member_map_ else Status.BAD_VALUE
            data = payload[1:]
            if opcode == Cmd.GET_INFO and status == Status.OK and len(data) >= _INFO.size:
                info = _parse_info(data)
                with self._lock:
                    self._discovered[info.uid] = info
            with self._lock:
                pending = self._pending.get((node, opcode))
            if pending:
                pending.status, pending.data = status, data
                pending.event.set()
        elif function == FN_TELEMETRY and opcode == FRAME_TELEMETRY:
            telemetry = _parse_telemetry(payload)
            if telemetry:
                with self._lock:
                    self._telemetry[node] = telemetry
        elif function == FN_EVENT and opcode == FRAME_FAULT and len(payload) >= 5:
            faults, warnings, state = struct.unpack_from("<HHB", payload)
            if self.on_fault:
                self.on_fault(node, Fault(faults), Warning_(warnings), AxisState(state))
        elif opcode == 0x02 and len(payload) >= 28:  # Tercio IMU (legacy protocol)
            with self._lock:
                self._legacy[can_id] = ImuState(*struct.unpack_from("<fffffff", payload))


# ------------------------------------------------------------------- stepper

class Stepper:
    """One Tercio S1 driver. Positions are in `unit` (degrees by default)."""

    def __init__(self, bus: Bus, node_id: int, unit: Unit = Unit.DEGREES):
        if not 1 <= node_id <= MAX_NODE_ID:
            raise ValueError("node id must be 1..127")
        self.bus = bus
        self.id = node_id
        self.unit = unit

    # -- units
    def _to_turns(self, value: float) -> float:
        return value / self.unit.value

    def _from_turns(self, turns: float) -> float:
        return turns * self.unit.value

    # -- system
    def info(self) -> NodeInfo:
        return _parse_info(self.bus.request(self.id, Cmd.GET_INFO))

    def save(self) -> None:
        """Persists all parameters to flash (refused while moving)."""
        self.bus.request(self.id, Cmd.SAVE_CONFIG, timeout=1.0)

    def factory_reset(self) -> None:
        self.bus.request(self.id, Cmd.FACTORY_RESET)

    def reboot(self) -> None:
        self.bus.request(self.id, Cmd.REBOOT)

    def clear_faults(self) -> None:
        self.bus.request(self.id, Cmd.CLEAR_FAULTS)

    def get_param(self, param: Param) -> Union[float, int, bool]:
        data = self.bus.request(self.id, Cmd.GET_PARAM, bytes([param]))
        return _decode_param(param, data[1:5])

    def set_param(self, param: Param, value: Union[float, int, bool]) -> Union[float, int, bool]:
        """Writes a parameter (RAM; call save() to persist). Returns the value as applied."""
        data = self.bus.request(self.id, Cmd.SET_PARAM, bytes([param]) + _encode_param(param, value))
        applied = _decode_param(param, data[1:5])
        if param == Param.NODE_ID:
            self.id = int(applied)
        return applied

    def set_node_id(self, new_id: int, persist: bool = True) -> None:
        self.set_param(Param.NODE_ID, new_id)
        time.sleep(0.02)
        if persist:
            self.save()

    # -- motion
    def enable(self) -> None:
        self.bus.request(self.id, Cmd.ENABLE, b"\x01")

    def disable(self) -> None:
        self.bus.request(self.id, Cmd.ENABLE, b"\x00")

    def stop(self) -> None:
        self.bus.request(self.id, Cmd.STOP)

    def emergency_stop(self) -> None:
        self.bus.request(self.id, Cmd.EMERGENCY_STOP)

    def _move(self, cmd: Cmd, value: float, velocity: Optional[float], acceleration: Optional[float],
              deferred: bool, wait: bool, timeout: float, ack: bool) -> None:
        payload = struct.pack("<dBff", self._to_turns(value), MOVE_DEFERRED if deferred else 0,
                              self._to_turns(velocity) if velocity else 0.0,
                              self._to_turns(acceleration) if acceleration else 0.0)
        if ack:
            self.bus.request(self.id, cmd, payload)
        else:
            self.bus.command(self.id, cmd, payload)
        if wait and not deferred:
            self.wait_settled(timeout)

    def move_to(self, position: float, velocity: Optional[float] = None, acceleration: Optional[float] = None,
                deferred: bool = False, wait: bool = False, timeout: float = 30.0, ack: bool = True) -> None:
        """Absolute move. `velocity`/`acceleration` override the limits (capped by them).
        `deferred` queues the move until Bus.sync(). `ack=False` streams without waiting for a reply."""
        self._move(Cmd.MOVE_TO, position, velocity, acceleration, deferred, wait, timeout, ack)

    def move_by(self, delta: float, velocity: Optional[float] = None, acceleration: Optional[float] = None,
                deferred: bool = False, wait: bool = False, timeout: float = 30.0, ack: bool = True) -> None:
        self._move(Cmd.MOVE_BY, delta, velocity, acceleration, deferred, wait, timeout, ack)

    def set_velocity(self, velocity: float, acceleration: Optional[float] = None) -> None:
        """Continuous rotation at `velocity` (units/s); 0 decelerates to a stop and holds."""
        self.bus.request(self.id, Cmd.SET_VELOCITY, struct.pack(
            "<ff", self._to_turns(velocity), self._to_turns(acceleration) if acceleration else 0.0))

    def set_zero(self, position: float = 0.0) -> None:
        """Declares the current position to be `position`."""
        self.bus.request(self.id, Cmd.SET_ZERO, struct.pack("<d", self._to_turns(position)))

    # -- procedures
    def calibrate(self, wait: bool = True, timeout: float = 10.0) -> None:
        """Encoder direction/scale calibration. The shaft moves a quarter turn each way."""
        self.bus.request(self.id, Cmd.CALIBRATE)
        if wait:
            self._wait_procedure(AxisState.CALIBRATING, Fault.CALIBRATION_FAILED, timeout)

    def home(self, wait: bool = True, timeout: float = 60.0) -> None:
        """Homing as configured by the HOMING_* parameters."""
        self.bus.request(self.id, Cmd.HOME)
        if wait:
            self._wait_procedure(AxisState.HOMING, Fault.HOMING_FAILED, timeout)

    def auto_tune(self, minimum: float, maximum: float, wait: bool = True, timeout: float = 600.0) -> Tuple[float, float]:
        """Finds the highest reliable velocity/acceleration between two positions.
        Returns (max velocity, max acceleration) in units/s and units/s²."""
        self.bus.request(self.id, Cmd.AUTO_TUNE,
                         struct.pack("<ff", self._to_turns(minimum), self._to_turns(maximum)))
        if wait:
            self._wait_procedure(AxisState.TUNING, Fault.STALL, timeout)
        return (self._from_turns(self.get_param(Param.MAX_VELOCITY)),
                self._from_turns(self.get_param(Param.MAX_ACCELERATION)))

    # -- convenience settings (in this stepper's units where they apply)
    def set_limits(self, velocity: Optional[float] = None, acceleration: Optional[float] = None) -> None:
        if velocity is not None:
            self.set_param(Param.MAX_VELOCITY, self._to_turns(velocity))
        if acceleration is not None:
            self.set_param(Param.MAX_ACCELERATION, self._to_turns(acceleration))

    def set_current(self, run_ma: int, hold_percent: Optional[int] = None) -> None:
        self.set_param(Param.RUN_CURRENT_MA, run_ma)
        if hold_percent is not None:
            self.set_param(Param.HOLD_CURRENT_PCT, hold_percent)

    def set_microsteps(self, microsteps: int) -> None:
        self.set_param(Param.MICROSTEPS, microsteps)

    def set_pid(self, kp: float, ki: float = 0.0, kd: float = 0.0) -> None:
        self.set_param(Param.KP, kp)
        self.set_param(Param.KI, ki)
        self.set_param(Param.KD, kd)

    # -- status
    @property
    def telemetry(self) -> Optional[Telemetry]:
        return self.bus.telemetry(self.id)

    @property
    def position(self) -> Optional[float]:
        t = self.telemetry
        return self._from_turns(t.position) if t else None

    @property
    def velocity(self) -> Optional[float]:
        t = self.telemetry
        return self._from_turns(t.velocity) if t else None

    def wait_settled(self, timeout: float = 30.0) -> None:
        """Blocks until the current move finished and the axis settled on target."""
        deadline = time.monotonic() + timeout
        time.sleep(0.03)  # let at least one telemetry frame reflect the new move
        while time.monotonic() < deadline:
            t = self.telemetry
            if t and t.state == AxisState.FAULT:
                raise TercioError(f"fault {t.faults!r}", node=self.id)
            if t and Fault.STALL in t.faults:
                raise TercioError("stall", node=self.id)
            if t and t.state == AxisState.HOLDING and t.settled:
                return
            time.sleep(0.005)
        raise TercioError("timeout waiting for the move", node=self.id)

    def _wait_procedure(self, running: AxisState, failure: Fault, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        time.sleep(0.05)
        while time.monotonic() < deadline:
            t = self.telemetry
            if t and t.state != running:
                if failure in t.faults or t.state == AxisState.FAULT:
                    raise TercioError(f"{running.name.lower()} failed: {t.faults!r}", node=self.id)
                return
            time.sleep(0.01)
        raise TercioError(f"timeout during {running.name.lower()}", node=self.id)


# ----------------------------------------------------- Tercio IMU (legacy v1)

@dataclass
class ImuState:
    roll: float
    pitch: float
    yaw: float
    ax: float
    ay: float
    az: float
    temp: float
    timestamp: float = field(default_factory=time.monotonic)


class IMU:
    """Tercio IMU module (its own protocol; unchanged)."""

    SET_ID = 0xA1
    RESET_ORIENTATION = 0xA2

    def __init__(self, bus: Bus, control_id: int = 0x003):
        self.bus = bus
        self.id = control_id

    def reset_orientation(self) -> None:
        self.bus.send(self.id, self.RESET_ORIENTATION)

    def set_can_id(self, new_id: int) -> None:
        self.bus.send(self.id, self.SET_ID, struct.pack("<H", new_id & 0x7FF))
        self.id = new_id & 0x7FF

    @property
    def state(self) -> Optional[ImuState]:
        with self.bus._lock:
            return self.bus._legacy.get(self.id)
