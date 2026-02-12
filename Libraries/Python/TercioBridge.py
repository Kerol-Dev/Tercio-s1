from __future__ import annotations
import struct, threading, time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional, Dict, Union
import serial
from serial.tools import list_ports


HDR_FMT = "<HBB"
HDR_SIZE = struct.calcsize(HDR_FMT)
MAX_PAYLOAD = 64


# Motor Constants
TELEMETRY_CMD = 0x01
GET_CONFIG_CMD = 0x20

# IMU Constants
IMU_TELEMETRY_CMD = 0x02


class Cmd(IntEnum):
    TARGET_ANGLE = 0x01
    SET_CURRENT_MA = 0x02
    SET_SPEED_LIMIT = 0x03
    SET_PID = 0x04
    SET_ID = 0x05
    SET_MICROSTEPS = 0x06
    SET_STEALTHCHOP = 0x07
    SET_EXT_MODE = 0x08
    SET_UNITS = 0x09
    SET_ENC_INVERT = 0x0A
    SET_ENABLED = 0x0B
    SET_STEPS_PER_REV = 0x0C
    DO_CALIBRATE = 0x0D
    DO_HOMING = 0x0E
    SET_ENDSTOP = 0x0F
    SET_EXT_ENCODER = 0x10
    SET_ACCEL_LIMIT = 0x11
    SET_DIR_INVERT = 0x12
    SET_EXT_SPI = 0x13
    DO_AUTO_TUNE = 0x14
    SET_LIMITSWITCH_ACTIVELOW = 0x15
    GET_CONFIG = 0x20
    SET_IMU_ID = 0xA1
    RESET_ORIENT = 0xA2


class TuningState(IntEnum):
    IDLE = 0
    PREP = 1
    FWD = 2
    BWD = 3
    INCREASE = 4
    DONE = 5


AXIS_CONFIG_FMT = "<I H H B H H H f f f f f H"
AXIS_CONFIG_SIZE = struct.calcsize(AXIS_CONFIG_FMT)

TELEM_TAIL_FMT = "<f f f f B B B B"
TELEM_TAIL_SIZE = struct.calcsize(TELEM_TAIL_FMT)

# IMU Payload: 7 floats (28 bytes)
IMU_PAYLOAD_FMT = "<fffffff"
IMU_PAYLOAD_SIZE = struct.calcsize(IMU_PAYLOAD_FMT)


@dataclass
class AxisFlags:
    encInvert: bool
    dirInvert: bool
    stealthChop: bool
    externalMode: bool
    enableEndStop: bool
    externalEncoder: bool
    calibratedOnce: bool
    externalSPI: bool
    limitSwitchActiveLow: bool


@dataclass
class AxisConfig:
    crc32: int
    microsteps: int
    stepsPerRev: int
    units: int
    flags: AxisFlags
    encZeroCounts: int
    driver_mA: int
    maxRPS: float
    maxRPS2: float
    Kp: float
    Ki: float
    Kd: float
    canArbId: int


@dataclass
class AxisState:
    config: AxisConfig
    currentSpeed: float
    currentAngle: float
    targetAngle: float
    temperature: float
    stalled: bool
    tuneState: TuningState
    minTriggered: bool
    maxTriggered: bool
    timestamp: float = field(default_factory=time.time)


@dataclass
class ImuState:
    roll: float
    pitch: float
    yaw: float
    ax: float
    ay: float
    az: float
    temp: float
    timestamp: float = field(default_factory=time.time)


@dataclass
class HomingParams:
    useIN1Trigger: bool = True
    sensorlessHoming: bool = False
    homingCurrent: int = 1200
    offset: float = 0.0
    activeLow: bool = True
    speed: float = 1.0
    direction: bool = True  # True => (+)


# ========= pack helpers =========
_u8 = lambda v: struct.pack("<B", v & 0xFF)
_u16 = lambda v: struct.pack("<H", v & 0xFFFF)
_f32 = lambda v: struct.pack("<f", float(v))
_b01 = lambda b: struct.pack("<B", 1 if b else 0)


def _pack_homing(p: HomingParams) -> bytes:
    cur = int(p.homingCurrent)
    if cur < 0:
        cur = 0
    if cur > 0xFFFF:
        cur = 0xFFFF

    # fixed name: useIN1Trigger
    use_in1 = 1 if (p.useIN1Trigger and not p.sensorlessHoming) else 0
    active_low = 1 if (p.activeLow and not p.sensorlessHoming) else 0

    return struct.pack(
        "<BBHfBfB",
        use_in1,
        1 if p.sensorlessHoming else 0,
        cur,
        float(p.offset),
        active_low,
        float(p.speed),
        1 if p.direction else 0,
    )


def _make_frame(can_id: int, cmd: int, payload: bytes = b"") -> bytes:
    if not (0 <= can_id <= 0x7FF):
        raise ValueError("can_id must be 11-bit (0..0x7FF)")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    return struct.pack(HDR_FMT, can_id, cmd & 0xFF, len(payload)) + payload


# ========= parsing =========
def _parse_axis_config(b: bytes) -> AxisConfig:
    (
        crc32,
        microsteps,
        stepsPerRev,
        units,
        flags_u16,      # Changed from flags_u8
        encZeroCounts,
        driver_mA,
        maxRPS,
        maxRPS2,
        Kp,
        Ki,
        Kd,
        canArbId,
    ) = struct.unpack(AXIS_CONFIG_FMT, b[:AXIS_CONFIG_SIZE])

    fl = AxisFlags(
        encInvert=bool(flags_u16 & 0x01),
        dirInvert=bool(flags_u16 & 0x02),
        stealthChop=bool(flags_u16 & 0x04),
        externalMode=bool(flags_u16 & 0x08),
        enableEndStop=bool(flags_u16 & 0x10),
        externalEncoder=bool(flags_u16 & 0x20),
        calibratedOnce=bool(flags_u16 & 0x40),
        externalSPI=bool(flags_u16 & 0x80),
        limitSwitchActiveLow=bool(flags_u16 & 0x100),  # Now valid!
    )

    return AxisConfig(
        crc32=crc32,
        microsteps=microsteps,
        stepsPerRev=stepsPerRev,
        units=units,
        flags=fl,
        encZeroCounts=encZeroCounts,
        driver_mA=driver_mA,
        maxRPS=maxRPS,
        maxRPS2=maxRPS2,
        Kp=Kp,
        Ki=Ki,
        Kd=Kd,
        canArbId=canArbId,
    )


def _parse_telemetry(payload: bytes) -> Optional[AxisState]:
    if len(payload) < AXIS_CONFIG_SIZE + TELEM_TAIL_SIZE:
        return None

    cfg = _parse_axis_config(payload[:AXIS_CONFIG_SIZE])

    (curSpd, curAng, tgtAng, temp, stalled_u8, tuneState_u8, minT_u8, maxT_u8) = (
        struct.unpack(
            TELEM_TAIL_FMT,
            payload[AXIS_CONFIG_SIZE : AXIS_CONFIG_SIZE + TELEM_TAIL_SIZE],
        )
    )

    return AxisState(
        config=cfg,
        currentSpeed=curSpd,
        currentAngle=curAng,
        targetAngle=tgtAng,
        temperature=temp,
        stalled=bool(stalled_u8),
        tuneState=TuningState(tuneState_u8) if tuneState_u8 <= 5 else TuningState.IDLE,
        minTriggered=bool(minT_u8),
        maxTriggered=bool(maxT_u8),
    )


def _parse_imu_telemetry(payload: bytes) -> Optional[ImuState]:
    if len(payload) < IMU_PAYLOAD_SIZE:
        return None
    roll, pitch, yaw, ax, ay, az, temp = struct.unpack(
        IMU_PAYLOAD_FMT, payload[:IMU_PAYLOAD_SIZE]
    )
    return ImuState(roll, pitch, yaw, ax, ay, az, temp)


# ========= Bridge =========
class Bridge:
    def __init__(self, port: Optional[str] = None, baud=115200, timeout=0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._rx_buf = bytearray()
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None

        # State stores both Stepper and IMU states
        # Key: CAN ID (int). Value: AxisState or ImuState
        self._state: Dict[int, Union[AxisState, ImuState]] = {}
        self._lock = threading.Lock()

    @staticmethod
    def find_first_port() -> str:
        ports = list(list_ports.comports())
        if not ports:
            raise RuntimeError("No serial ports found.")
        return ports[0].device

    def open(self):
        if not self.port:
            self.port = self.find_first_port()
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def close(self):
        self._stop.set()
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=1.5)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._reader = None
        self._ser = None

    # ---- writer
    def send(self, can_id: int, cmd: IntEnum | int, payload: bytes = b""):
        if self._ser is None:
            raise RuntimeError("Bridge not open")
        frame = _make_frame(can_id, int(cmd), payload)
        self._ser.write(frame)
        self._ser.flush()

    # ---- reader
    def _reader_loop(self):
        ser = self._ser
        if not ser:
            return
        try:
            while not self._stop.is_set():
                chunk = ser.read(256)
                if not chunk:
                    continue
                self._rx_buf.extend(chunk)
                self._process_buf()
        except Exception:
            return

    def _process_buf(self):
        while True:
            if len(self._rx_buf) < HDR_SIZE:
                return
            can_id, cmd, ln = struct.unpack(HDR_FMT, self._rx_buf[:HDR_SIZE])
            total = HDR_SIZE + ln
            if len(self._rx_buf) < total:
                return
            payload = bytes(self._rx_buf[HDR_SIZE:total])
            del self._rx_buf[:total]

            if cmd == TELEMETRY_CMD:
                pkt = _parse_telemetry(payload)
                if pkt:
                    with self._lock:
                        self._state[pkt.config.canArbId] = pkt

            elif cmd == GET_CONFIG_CMD:
                if len(payload) >= AXIS_CONFIG_SIZE:
                    cfg = _parse_axis_config(payload[:AXIS_CONFIG_SIZE])
                    with self._lock:
                        st = self._state.get(cfg.canArbId)
                        if st and isinstance(st, AxisState):
                            st.config = cfg
                        else:
                            self._state[cfg.canArbId] = AxisState(
                                config=cfg,
                                currentSpeed=0.0,
                                currentAngle=0.0,
                                targetAngle=0.0,
                                temperature=0.0,
                                stalled=False,
                                tuneState=TuningState.IDLE,
                                minTriggered=False,
                                maxTriggered=False,
                            )

            elif cmd == IMU_TELEMETRY_CMD:
                imu_pkt = _parse_imu_telemetry(payload)
                if imu_pkt:
                    with self._lock:
                        self._state[can_id] = imu_pkt

    # ---- state getters
    def get_state(self, can_id: int) -> Union[AxisState, ImuState, None]:
        with self._lock:
            return self._state.get(can_id)

    def request_config(self, can_id: int):
        self.send(can_id, Cmd.GET_CONFIG, b"")

    # ---- high-level command wrappers (id must be provided)
    def set_target_angle(self, can_id: int, angle: float):
        self.send(can_id, Cmd.TARGET_ANGLE, _f32(angle))

    def set_current_ma(self, can_id: int, ma: int):
        self.send(can_id, Cmd.SET_CURRENT_MA, _u16(ma))

    def set_speed_limit_rps(self, can_id: int, rps: float):
        self.send(can_id, Cmd.SET_SPEED_LIMIT, _f32(rps))

    def set_accel_limit_rps2(self, can_id: int, rps2: float):
        self.send(can_id, Cmd.SET_ACCEL_LIMIT, _f32(rps2))

    def set_pid(self, can_id: int, kp: float, ki: float, kd: float):
        self.send(can_id, Cmd.SET_PID, _f32(kp) + _f32(ki) + _f32(kd))

    def set_can_id(self, can_id: int, new_id: int):
        self.send(can_id, Cmd.SET_ID, _u16(new_id & 0x7FF))

    def set_microsteps(self, can_id: int, microsteps: int):
        self.send(can_id, Cmd.SET_MICROSTEPS, _u16(microsteps))

    def set_stealthchop(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_STEALTHCHOP, _b01(enable))

    def set_external_mode(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_EXT_MODE, _b01(enable))

    def set_units_degrees(self, can_id: int, use_degrees: bool):
        self.send(can_id, Cmd.SET_UNITS, _u8(1 if use_degrees else 0))

    def set_encoder_invert(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_ENC_INVERT, _b01(enable))

    def set_direction_invert(self, can_id: int, invert: bool):
        self.send(can_id, Cmd.SET_DIR_INVERT, _b01(invert))

    def enable_motor(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_ENABLED, _b01(enable))

    def set_steps_per_rev(self, can_id: int, steps_per_rev: int):
        self.send(can_id, Cmd.SET_STEPS_PER_REV, _u16(steps_per_rev))

    def set_external_encoder(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_EXT_ENCODER, _b01(enable))

    def set_external_spi(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_EXT_SPI, _b01(enable))

    def set_endstop(self, can_id: int, enable: bool):
        self.send(can_id, Cmd.SET_ENDSTOP, _b01(enable))

    def set_limitswitch_activelow(self, can_id: int, active_low: bool):
        self.send(can_id, Cmd.SET_LIMITSWITCH_ACTIVELOW, _b01(active_low))

    def do_calibrate(self, can_id: int):
        self.send(can_id, Cmd.DO_CALIBRATE, b"")

    def do_homing(self, can_id: int, p: HomingParams):
        self.send(can_id, Cmd.DO_HOMING, _pack_homing(p))

    def do_auto_tune(self, can_id: int, min_angle: float, max_angle: float):
        payload = struct.pack("<ff", float(min_angle), float(max_angle))
        self.send(can_id, Cmd.DO_AUTO_TUNE, payload)

    def set_imu_id(self, current_id: int, new_id: int):
        self.send(current_id, Cmd.SET_IMU_ID, _u16(new_id & 0x7FF))

    def reset_orientation(self, can_id: int):
        self.send(can_id, Cmd.RESET_ORIENT, b"")


# ========= Friendly Stepper wrapper (pins id) =========
class Stepper:
    def __init__(self, bridge: Bridge, can_id: int):
        if not (0 <= can_id <= 0x7FF):
            raise ValueError("11-bit CAN id expected")
        self.bridge = bridge
        self.id = can_id

    # convenience
    def request_config(self):
        self.bridge.request_config(self.id)

    def enable_motor(self, en: bool):
        self.bridge.enable_motor(self.id, en)

    def set_target_angle(self, ang: float):
        self.bridge.set_target_angle(self.id, ang)

    def set_current_ma(self, ma: int):
        self.bridge.set_current_ma(self.id, ma)

    def set_speed_limit_rps(self, rps: float):
        self.bridge.set_speed_limit_rps(self.id, rps)

    def set_accel_limit_rps2(self, rps2: float):
        self.bridge.set_accel_limit_rps2(self.id, rps2)

    def set_pid(self, kp, ki, kd):
        self.bridge.set_pid(self.id, kp, ki, kd)

    def set_can_id(self, new_id: int):
        self.bridge.set_can_id(self.id, new_id)
        self.id = new_id & 0x7FF

    def set_microsteps(self, m: int):
        self.bridge.set_microsteps(self.id, m)

    def set_stealthchop(self, en: bool):
        self.bridge.set_stealthchop(self.id, en)

    def set_external_mode(self, en: bool):
        self.bridge.set_external_mode(self.id, en)

    def set_units_degrees(self, on: bool):
        self.bridge.set_units_degrees(self.id, on)

    def set_encoder_invert(self, on: bool):
        self.bridge.set_encoder_invert(self.id, on)

    def set_direction_invert(self, on: bool):
        self.bridge.set_direction_invert(self.id, on)

    def set_external_encoder(self, on: bool):
        self.bridge.set_external_encoder(self.id, on)

    def set_external_spi(self, on: bool):
        self.bridge.set_external_spi(self.id, on)

    def set_endstop(self, on: bool):
        self.bridge.set_endstop(self.id, on)

    def set_limitswitch_activelow(self, active_low: bool):
        self.bridge.set_limitswitch_activelow(self.id, active_low)

    def do_calibrate(self):
        self.bridge.do_calibrate(self.id)

    def do_homing(self, p: HomingParams):
        self.bridge.do_homing(self.id, p)

    def do_auto_tune(self, min_angle: float, max_angle: float):
        self.bridge.do_auto_tune(self.id, min_angle, max_angle)

    def get_axis_state(self) -> Optional[AxisState]:
        st = self.bridge.get_state(self.id)
        if isinstance(st, AxisState):
            return st
        return None


# ========= Friendly IMU wrapper =========
class IMU:
    def __init__(self, bridge: Bridge, control_id: int = 0x003):
        """
        :param bridge: The shared Bridge instance.
        :param control_id: The CAN ID used to send commands (like Reset) to the IMU. Default 0x003.
        """
        self.bridge = bridge
        self.control_id = control_id

    def reset_orientation(self):
        """Sends the orientation reset command to the IMU."""
        self.bridge.reset_orientation(self.control_id)

    def set_can_id(self, new_id: int):
        """
        Sets the CAN ID of the IMU.
        Note: This changes the ID the IMU listens to for future commands.
        """
        self.bridge.set_imu_id(self.control_id, new_id)
        self.control_id = new_id & 0x7FF

    def get_state(self) -> Optional[ImuState]:
        """Returns the latest IMU telemetry state (Roll, Pitch, Yaw, Accel, Temp)."""
        st = self.bridge.get_state(self.control_id)
        if isinstance(st, ImuState):
            return st
        return None

    def get_roll(self) -> Optional[float]:
        st = self.get_state()
        return st.roll if st else None

    def get_pitch(self) -> Optional[float]:
        st = self.get_state()
        return st.pitch if st else None

    def get_yaw(self) -> Optional[float]:
        st = self.get_state()
        return st.yaw if st else None

    def get_accel_x(self) -> Optional[float]:
        st = self.get_state()
        return st.ax if st else None

    def get_accel_y(self) -> Optional[float]:
        st = self.get_state()
        return st.ay if st else None

    def get_accel_z(self) -> Optional[float]:
        st = self.get_state()
        return st.az if st else None

    def get_temperature(self) -> Optional[float]:
        st = self.get_state()
        return st.temp if st else None