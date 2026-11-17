import argparse
import bisect
import struct
import sys
import time
import serial

MODELS = {
    101: ("RPM-20GS", "ARW28340", 202, 21, 10.0, 20000.0, -40.0, 10.0),
    102: ("RPM-3GS",  "AD8362",    33, 25, 0.00005, 3000.0, -50.0, 10.0),
    103: ("RPM-9G",   "ARW22347",  92, 21, 10.0, 9000.0, -40.0, 10.0),
    104: ("RPM-20GS", "ARW28340", 218, 21, 10.0, 20000.0, -40.0, 10.0),
    105: ("RPM-3GS",  "AD8362",    58, 25, 0.00005, 3000.0, -50.0, 10.0),
    106: ("RPM-9G",   "ARW22347",  99, 21, 10.0, 9000.0, -40.0, 10.0),
    107: ("RPM-6GH",  "ARW22283",  69, 41, 10.0, 6000.0, -80.0, 20.0),
}

BAUD = 460800
ADC_FULL_SCALE = 8388607.0  # 2^23 - 1, 24-bit ADC
ADC_RANGE_MV = 1500.0
SETUP_TIMEOUT = 3.0
CAL_STALL_TIMEOUT = 5.0
CAL_RETRY = 0.25
# A terminal cannot print 640/1280 lines per second without stalling the reader.
TTY_FAST_HZ = 20.0

STATUS_TEXT = {
    "under": "below calibrated range (noise floor)",
    "over": "above calibrated range (sensor overload)",
    "freq": "frequency outside calibration grid",
    "out": "out of calibration range",
}

def fail(message):
    print(message, file=sys.stderr)
    sys.exit(1)

def build_frame(cmd: int, data: bytes = b"") -> bytes:
    length = len(data)
    inv_len = (~length) & 0xFF
    checksum = (cmd + length + inv_len + sum(data)) & 0xFF
    return bytes([0x55, 0xAA, cmd, length, inv_len]) + data + bytes([checksum])

def read_frame(buf: bytearray):
    while len(buf) >= 6:
        if buf[0] != 0x55 or buf[1] != 0xAA or buf[3] != ((~buf[4]) & 0xFF):
            idx = buf.find(b"\x55", 1)
            if idx == -1:
                buf.clear()
                return None
            del buf[:idx]
            continue
        length = buf[3]
        total = 6 + length
        if len(buf) < total:
            return None
        checksum = sum(buf[2 : 5 + length]) & 0xFF
        if checksum != buf[5 + length]:
            del buf[:1]
            continue
        cmd = buf[2]
        payload = bytes(buf[5 : 5 + length])
        del buf[:total]
        return cmd, payload
    return None

def build_freq_grid(device_id: int, f_num: int, f_min: float, f_max: float):
    # 10 MHz, 50 MHz, then a 100 MHz step. RPM-20GS ID 104 is not in this set:
    # its factory spacing is not confirmed, so it keeps the linear grid below.
    if device_id in (101, 103, 107) and f_num > 2:
        grid = [10.0, 50.0] + [100.0 * i for i in range(1, f_num - 1)]
        if len(grid) == f_num:
            return grid
    step = (f_max - f_min) / max(1, f_num - 1)
    return [f_min + i * step for i in range(f_num)]

def range_status(voltage_mv, row, powers):
    """Which end of the calibration the voltage fell off.

    The extreme sample owns the power level that this voltage is past.
    A rising detector (voltage grows with power) and a falling one both work:
    past the high-power end is overload, past the low-power end is the noise floor.
    """
    if voltage_mv > max(row):
        index = max(range(len(row)), key=row.__getitem__)
    elif voltage_mv < min(row):
        index = min(range(len(row)), key=row.__getitem__)
    else:
        return "out"
    span = powers[-1] - powers[0]
    if span == 0:
        return "out"
    toward_high_power = (powers[index] - powers[0]) / span
    return "over" if toward_high_power >= 0.5 else "under"

def interp_power(voltage_mv, freq_mhz, freq_grid, cal_table, p_min, p_max):
    """Return (dBm, None) inside the table, or (None, status) outside it."""
    p_num = len(cal_table[0])
    p_step = (p_max - p_min) / (p_num - 1)
    powers = [p_min + j * p_step for j in range(p_num)]

    if freq_mhz < freq_grid[0] or freq_mhz > freq_grid[-1]:
        return None, "freq"

    def eval_knot(k_idx):
        row = cal_table[k_idx]
        for j in range(p_num - 1):
            v0, v1 = row[j], row[j + 1]
            if v0 <= voltage_mv <= v1 or v1 <= voltage_mv <= v0:
                if v1 == v0:
                    return powers[j]
                return powers[j] + (voltage_mv - v0) / (v1 - v0) * (powers[j + 1] - powers[j])
        return range_status(voltage_mv, row, powers)

    pos = bisect.bisect_right(freq_grid, freq_mhz) - 1
    pos = max(0, min(pos, len(freq_grid) - 1))
    if freq_grid[pos] == freq_mhz or pos + 1 >= len(freq_grid):
        return _reading(eval_knot(pos))

    p1 = eval_knot(pos)
    p2 = eval_knot(pos + 1)
    if not isinstance(p1, str) and not isinstance(p2, str):
        span = freq_grid[pos + 1] - freq_grid[pos]
        prop = 0.0 if span == 0 else (freq_mhz - freq_grid[pos]) / span
        prop = min(1.0, max(0.0, prop))
        return p1 * (1.0 - prop) + p2 * prop, None
    return None, _combine_status(p1, p2)

def _reading(value):
    if isinstance(value, str):
        return None, value
    return value, None

def _combine_status(p1, p2):
    statuses = {p for p in (p1, p2) if isinstance(p, str)}
    if not statuses:
        return "out"
    if len(statuses) == 1:
        return statuses.pop()
    if "over" in statuses:
        return "over"
    if "under" in statuses:
        return "under"
    return "out"

def wait_packet(ser, buf, accept, timeout, request, retry_interval=0.05):
    """Resend request until accept(cmd, payload) returns a value, or time out.

    A returned 0 is a valid result (sampling-rate ack), so only None means "keep waiting".
    """
    deadline = time.monotonic() + timeout
    next_send = 0.0
    while True:
        now = time.monotonic()
        if now >= deadline:
            return None
        if now >= next_send:
            ser.write(request)
            next_send = now + retry_interval
        buf.extend(ser.read(256))
        while (pkt := read_frame(buf)) is not None:
            result = accept(*pkt)
            if result is not None:
                return result

def accept_ident(cmd, payload):
    if cmd == 0x00 and len(payload) == 8:
        return struct.unpack(">HHI", payload)
    return None

def accept_rate(cmd, payload):
    if cmd == 0x03 and len(payload) == 1:
        return payload[0]
    return None

def download_calibration(ser, buf, f_num, p_num):
    cal_table = [None] * f_num
    filled = 0
    reported = 0
    row_bytes = p_num * 4 + 2
    ser.write(build_frame(0x86, struct.pack(">H", 0xFFFF)))
    now = time.monotonic()
    last_new = now
    last_retry = now

    while filled < f_num:
        buf.extend(ser.read(1024))
        while (pkt := read_frame(buf)) is not None:
            cmd, payload = pkt
            if cmd != 0x05 or len(payload) != row_bytes:
                continue
            f_idx = struct.unpack(">H", payload[:2])[0]
            if not 0 <= f_idx < f_num or cal_table[f_idx] is not None:
                continue
            cal_table[f_idx] = struct.unpack(f"<{p_num}f", payload[2:])
            filled += 1
            last_new = time.monotonic()

        if filled != reported and (filled == f_num or filled - reported >= 25):
            print(f"  {filled}/{f_num}", flush=True)
            reported = filled

        now = time.monotonic()
        if now - last_new > CAL_STALL_TIMEOUT:
            missing = [i for i, row in enumerate(cal_table) if row is None]
            shown = ", ".join(str(i) for i in missing[:8])
            if len(missing) > 8:
                shown += f", ... ({len(missing)} missing)"
            fail(f"Calibration stalled at {filled}/{f_num}. Missing rows: {shown}")
        if filled < f_num and now - last_new > CAL_RETRY and now - last_retry > CAL_RETRY:
            missing_idx = next(i for i, row in enumerate(cal_table) if row is None)
            ser.write(build_frame(0x86, struct.pack(">H", missing_idx)))
            last_retry = now

    return cal_table

def format_reading(freq_mhz, voltage_mv, dbm, status, atten):
    """One measurement. dBm is the sensor reading plus external attenuation."""
    head = f"{freq_mhz:8.2f} MHz"
    if status is not None:
        text = STATUS_TEXT.get(status, STATUS_TEXT["out"])
        return f"{head} | {text} | {voltage_mv:7.2f} mV"
    total_dbm = dbm + atten
    mw = 10.0 ** (total_dbm / 10.0)
    line = f"{head} | {total_dbm:+7.2f} dBm | {mw:10.4f} mW | {voltage_mv:7.2f} mV"
    if atten != 0.0:
        line += f" | sensor {dbm:+7.2f} dBm | atten {atten:+.1f} dB"
    return line

def emit_reading(line, live):
    if live:
        print(f"\r{line}\033[K", end="", flush=True)
    else:
        print(line, flush=True)

def main():
    parser = argparse.ArgumentParser(
        description="Read power from a ConceptRF RPM-series meter.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  python rfmeter.py /dev/cu.usbserial-14120
  python rfmeter.py /dev/cu.usbserial-14120 -f 5800 -a 30 -r 1
  python rfmeter.py /dev/cu.usbserial-14120 --freq 2400 --atten 30 --rate 1 --verbose
""",
    )
    parser.add_argument("port", help="Serial port (e.g. /dev/cu.usbserial-14120)")
    parser.add_argument("-f", "--freq", type=float, default=2400.0, help="Target frequency in MHz (default: 2400)")
    parser.add_argument("-a", "--atten", type=float, default=0.0, help="External attenuation offset in dB (default: 0.0)")
    parser.add_argument("-r", "--rate", type=int, choices=[0, 1, 2, 3], default=0,
                        help="Sampling rate index: 0=10Hz, 1=40Hz, 2=640Hz, 3=1280Hz (default: 0)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Print each sample on its own line (default: refresh one line)")
    args = parser.parse_args()

    try:
        run(args)
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    except BrokenPipeError:
        sys.stdout.close()
        sys.exit(0)
    except serial.SerialException as exc:
        fail(f"Serial port error: {exc}")

def run(args):
    with serial.Serial(args.port, BAUD, timeout=0.05) as ser:
        ser.reset_input_buffer()
        buf = bytearray()

        ident = wait_packet(ser, buf, accept_ident, SETUP_TIMEOUT, build_frame(0x80))
        if ident is None:
            fail(f"No identification response on {args.port}. Check the port and that the meter is connected.")
        device_id, fw_ver, serial_num = ident

        if device_id not in MODELS:
            fail(f"Unknown device_id={device_id}. Exiting.")

        model_name, chip, f_num, p_num, f_min, f_max, p_min, p_max = MODELS[device_id]
        print(
            f"Connected: {model_name} ({chip}) | ID={device_id} | FW={fw_ver / 100:.2f} | SN={serial_num}",
            flush=True,
        )

        rate_ack = wait_packet(
            ser, buf, accept_rate, SETUP_TIMEOUT, build_frame(0x83, bytes([args.rate]))
        )
        if rate_ack is None:
            fail("Meter did not acknowledge the sampling rate.")
        if rate_ack != args.rate:
            fail(f"Meter rejected sampling rate {args.rate} (acknowledged {rate_ack}).")
        print(f"Sampling rate index set to: {rate_ack}", flush=True)

        print(f"Downloading {f_num} calibration rows...", flush=True)
        cal_table = download_calibration(ser, buf, f_num, p_num)
        freq_grid = build_freq_grid(device_id, f_num, f_min, f_max)
        if not freq_grid[0] <= args.freq <= freq_grid[-1]:
            print(
                f"Warning: {args.freq:g} MHz is outside the calibration grid "
                f"({freq_grid[0]:.5g} .. {freq_grid[-1]:.5g} MHz).",
                flush=True,
            )

        # A pipe has no cursor to refresh, so each sample is a line there too.
        live = sys.stdout.isatty() and not args.verbose
        display_interval = (1.0 / TTY_FAST_HZ) if sys.stdout.isatty() and args.rate >= 2 else 0.0
        if display_interval and not live:
            print(
                f"Terminal output limited to {TTY_FAST_HZ:.0f} lines/s. Redirect stdout to log every sample.",
                flush=True,
            )
        print(
            f"Calibration loaded. Measuring at {args.freq:g} MHz | "
            f"Atten: {args.atten:+.1f} dB (Ctrl+C to stop)\n",
            flush=True,
        )

        next_display = 0.0
        pending = None
        while True:
            buf.extend(ser.read(512))
            while (pkt := read_frame(buf)) is not None:
                cmd, payload = pkt
                if cmd != 0x06 or len(payload) != 4:
                    continue
                raw_32 = struct.unpack(">i", payload)[0]
                voltage_mv = raw_32 * ADC_RANGE_MV / ADC_FULL_SCALE
                dbm, status = interp_power(
                    voltage_mv, args.freq, freq_grid, cal_table, p_min, p_max
                )
                line = format_reading(args.freq, voltage_mv, dbm, status, args.atten)
                if live:
                    pending = line
                    continue
                now = time.monotonic()
                if display_interval and now < next_display:
                    continue
                if display_interval:
                    next_display = now + display_interval
                emit_reading(line, live=False)
            if pending is not None:
                now = time.monotonic()
                if display_interval == 0.0 or now >= next_display:
                    emit_reading(pending, live=True)
                    pending = None
                    if display_interval:
                        next_display = now + display_interval

if __name__ == "__main__":
    main()
