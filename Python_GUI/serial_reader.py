"""STM32 F446RE / ICM-20948 serial transport.

Firmware: DATA,seq,ms,ax,ay,az,gx,gy,gz,mx,my,mz,mag_valid
          13 comma-separated fields, with raw integer sensor readings.
Converted IMUSample units: acceleration m/s^2, angular rate rad/s,
magnetic field microtesla. The firmware applies existing accel offsets.
Legacy 12-field SI-format packets are also accepted.
"""
from dataclasses import dataclass
from typing import Callable, Optional
import math
import threading
import time
from pathlib import Path

DEBUG_LOG = Path(__file__).resolve().parent / "serial_debug.log"

def _debug(message):
    try:
        with DEBUG_LOG.open("a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}\n")
    except OSError:
        pass

ACCEL_MPS2_PER_LSB = 9.80665 / 16384.0  # +/-2g
GYRO_RADPS_PER_LSB = math.pi / (180.0 * 131.0)  # +/-250 deg/s
MAG_UT_PER_LSB = 0.15  # AK09916


@dataclass(frozen=True)
class IMUSample:
    seq: int
    timestamp_ms: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float
    mag_valid: bool = True


@dataclass(frozen=True)
class FilterSample:
    filter_id: str
    roll: float
    pitch: float
    yaw: float
    quaternion: tuple

@dataclass(frozen=True)
class AngleSample:
    filter_id: str
    angle_360: float
    angle_signed: float

@dataclass(frozen=True)
class PerformanceSample:
    filter_id: str
    execution_us: float
    cpu_percent: float
    state_bytes: int

def parse_telemetry(line):
    fields = line.strip().split(',')
    if len(fields) == 4 and fields[0] == 'ANGLE' and fields[1] in 'CMAE':
        try:
            v=float(fields[2]); signed=float(fields[3])
            if all(map(math.isfinite,(v,signed))) and -360<=signed<=360:
                return AngleSample(fields[1],v,signed)
        except ValueError:pass
        return None
    if len(fields) == 9 and fields[0] == 'FILT' and fields[1] in 'CMAE':
        try:
            nums = tuple(float(x) for x in fields[2:])
            if not all(map(math.isfinite, nums)):
                return None
            q = nums[3:7]
            n = math.sqrt(sum(x*x for x in q))
            if not 0.5 < n < 1.5:
                return None
            return FilterSample(fields[1], *nums[:3], tuple(x/n for x in q))
        except (ValueError, OverflowError):
            return None
    if len(fields) == 5 and fields[0] == 'PERF' and fields[1] in 'CMAE':
        try:
            us, cpu, size = float(fields[2]), float(fields[3]), int(fields[4])
            if not all(map(math.isfinite, (us,cpu))) or min(us,cpu,size) < 0:
                return None
            return PerformanceSample(fields[1], us, cpu, size)
        except (ValueError, OverflowError):
            return None
    return None


def parse_sample(line: str) -> Optional[IMUSample]:
    fields = line.strip().split(',')
    if fields[0] != 'DATA' or len(fields) not in (12, 13):
        return None
    try:
        seq, timestamp_ms = int(fields[1]), int(fields[2])
        values = [float(v) for v in fields[3:12]]
        if not all(math.isfinite(v) for v in values):
            return None
        if len(fields) == 13:
            flag = int(fields[12])
            if flag not in (0, 1):
                return None
            ax, ay, az, gx, gy, gz, mx, my, mz = values
            values = [
                ax * ACCEL_MPS2_PER_LSB,
                ay * ACCEL_MPS2_PER_LSB,
                az * ACCEL_MPS2_PER_LSB,
                gx * GYRO_RADPS_PER_LSB,
                gy * GYRO_RADPS_PER_LSB,
                gz * GYRO_RADPS_PER_LSB,
                mx * MAG_UT_PER_LSB,
                my * MAG_UT_PER_LSB,
                mz * MAG_UT_PER_LSB,
            ]
            return IMUSample(seq, timestamp_ms, *values, mag_valid=bool(flag))
        return IMUSample(seq, timestamp_ms, *values)
    except (ValueError, OverflowError):
        return None


class STM32Serial:
    """Background reader; callbacks run on reader thread, not Qt GUI thread."""

    def __init__(self, port: str, on_sample: Callable[[IMUSample], None],
                 on_error: Optional[Callable[[Exception], None]] = None,
                 baudrate: int = 115200,
                 on_telemetry: Optional[Callable[[object], None]] = None,
                 on_command: Optional[Callable[[str], None]] = None):
        self.port, self.baudrate = port, baudrate
        self.on_sample, self.on_error = on_sample, on_error
        self.on_telemetry = on_telemetry
        self.on_command = on_command
        self._serial = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._raw_count = 0
        self._valid_count = 0
        self._invalid_count = 0
        self._telemetry_count = {k: 0 for k in "CMAE"}

    def start(self):
        import serial
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._raw_count = self._valid_count = self._invalid_count = 0
        _debug(f"OPEN port={self.port} baud={self.baudrate}")
        self._serial = serial.Serial(self.port, self.baudrate,
                                     timeout=0.2, write_timeout=1)
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                self._raw_count += 1
                line = raw.decode('ascii', errors='replace').strip()
                sample = parse_sample(line)
                if sample is not None:
                    self._valid_count += 1
                    if self._valid_count <= 3 or self._valid_count % 250 == 0:
                        _debug(f"VALID #{self._valid_count} {line[:180]!r}")
                    self.on_sample(sample)
                elif (telemetry := parse_telemetry(line)) is not None:
                    if isinstance(telemetry, FilterSample):
                        fid = telemetry.filter_id
                        self._telemetry_count[fid] += 1
                        n = self._telemetry_count[fid]
                        if n in (1, 50, 200) or n % 1000 == 0:
                            _debug(f"FILT {fid} #{n} yaw={telemetry.yaw:.2f} roll={telemetry.roll:.2f} pitch={telemetry.pitch:.2f}")
                    if self.on_telemetry:
                        self.on_telemetry(telemetry)
                elif line.startswith(('STATUS,', 'ACK,', 'ERR,', 'PONG')):
                    _debug(f'RESPONSE {line[:240]}')
                    if self.on_command:
                        self.on_command(line)
                else:
                    self._invalid_count += 1
                    if self._invalid_count <= 20 or self._invalid_count % 250 == 0:
                        _debug(f"REJECT #{self._invalid_count} fields={len(line.split(','))} raw={line[:240]!r}")
            except Exception as exc:
                _debug(f"ERROR {type(exc).__name__}: {exc}")
                if not self._stop.is_set() and self.on_error:
                    self.on_error(exc)
                break

    def send(self, command: str):
        if not self._serial or not self._serial.is_open:
            raise RuntimeError('STM32 serial port is not open')
        if '\n' in command or '\r' in command:
            raise ValueError('Invalid command')
        with self._lock:
            _debug(f'SEND {command}')
            self._serial.write((command + '\n').encode('ascii'))

    def vibrate(self, enabled: bool):
        self.send('VIBRATE,ON' if enabled else 'VIBRATE,OFF')

    def rotate(self, degrees: float):
        if not math.isfinite(degrees) or not -360 <= degrees <= 360:
            raise ValueError('Rotation command outside allowed range')
        # Firmware uses strtol, so send integer degrees only.
        self.send(f'ROTATE,{round(degrees)}')

    def zero_home(self):
        self.send('ZERO')

    def stop_motor(self):
        self.send('STOP')

    def close(self):
        _debug(f"CLOSE raw={self._raw_count} valid={self._valid_count} rejected={self._invalid_count}")
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1)
        if self._serial:
            self._serial.close()
            self._serial = None


def list_ports():
    from serial.tools import list_ports as ports
    return [(p.device, p.description) for p in ports.comports()]
