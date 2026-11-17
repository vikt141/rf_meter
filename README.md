# RF Meter

This script is a way to use the Concept-rf power meter RPM-20GS on macOS. It talks to the meter over USB serial, loads the factory calibration, and prints live power at the frequency you choose.

The meter identifies itself. RPM-20GS answers as device ID 101 or 104. Both use the same power range; the frequency grid differs, as described below.

## What this script assumes for the RPM-20GS

| Parameter | Value |
| --- | --- |
| Frequency range | 10 MHz … 20 GHz |
| Sensor power range | −40 … +10 dBm |
| Detector | ARW28340 |
| Power calibration points | 21 (2.5 dB step) |
| Serial rate | 460800 baud |

| Device ID | Calibration rows | Frequency grid |
| --- | --- | --- |
| 101 | 202 | 10 MHz, 50 MHz, then 100 MHz steps to 20 GHz |
| 104 | 218 | evenly spaced from 10 MHz to 20 GHz |

Set the measurement frequency with `-f` / `--freq`. It has to fall inside that grid. Power between the nearest calibration points is interpolated.

The model table also recognizes the RPM-3GS, RPM-9G, and RPM-6GH. This script is written and used for the RPM-20GS.

## What a measurement does

1. Opens the serial port at 460800 baud.
2. Asks the meter to identify itself (`0x80`). The reply carries the model ID, firmware version, and serial number.
3. Sets the sampling rate (`0x83`).
4. Downloads the factory calibration (`0x86`): one row of detector voltages per frequency point. Progress is printed. If the stream stalls, the missing row is requested again.
5. Reads live samples (`0x06`). The raw 24-bit code is scaled to millivolts (full scale ±1500 mV), then converted to dBm at the chosen frequency.
6. Adds the external attenuation from `-a` / `--atten`, and prints that power in mW as well.

Outside the calibration table the script prints a reason instead of a dBm value:

- below the lowest calibrated point — `below calibrated range (noise floor)`
- above the highest calibrated point — `above calibrated range (sensor overload)`
- frequency outside the grid — `frequency outside calibration grid`

Identification and the sampling-rate command wait up to 3 seconds. Calibration fails if no new row arrives for 5 seconds. A wrong port or a silent meter exits with an error instead of hanging. The acknowledged rate must match `-r` / `--rate`.

Stop with `Ctrl+C`.

## Install

Python 3.8 or newer, with `pip`. The only dependency is `pyserial`.

From the project directory:

```bash
python3 -m venv venv
source venv/bin/activate
python -m pip install --upgrade pip
pip install pyserial
```

The shell prompt gains a `(venv)` prefix, and `python` / `pip` point at the interpreter inside `venv/`.

Check the install:

```bash
python -c "import serial; print(serial.__version__)"
```

Leave the environment with `deactivate`. Next time, activate it again and run the script. The dependency is already installed:

```bash
source venv/bin/activate
python rfmeter.py /dev/cu.usbserial-XXXX
```

`venv/` is local. Do not commit it.

## Port

On macOS the meter shows up as `/dev/cu.usbserial-…`. Use the `cu.` device, not `tty.`. The USB adapter is a CH340 (`USB VID:PID=1A86:7523`).

After activating the virtual environment:

```bash
python -m serial.tools.list_ports
```

or:

```bash
ls /dev/cu.usb*
```

No RF source is required to identify the meter or to load calibration. An open input, or a 50 ohm load, is enough.

## Run

Help, with examples and the argument list:

```bash
python rfmeter.py -h
```

```bash
python rfmeter.py PORT [-f MHz] [-a dB] [-r 0|1|2|3] [-v]
```

Examples for the RPM-20GS:

```bash
# 2.4 GHz, no external attenuator, 10 samples/s
python rfmeter.py /dev/cu.usbserial-14120

# 5.8 GHz, 30 dB attenuator in front of the sensor, 40 samples/s
python rfmeter.py /dev/cu.usbserial-14120 -f 5800 -a 30 -r 1

# one line per sample
python rfmeter.py /dev/cu.usbserial-14120 -v
```

`-a` / `--atten` is a dB offset **added** to the sensor reading. With a 30 dB attenuator in front of the meter, pass `-a 30`. The first dBm number is then the power before the attenuator.

### Arguments

Short and long forms set the same value.

| Argument | Default | Meaning |
| --- | --- | --- |
| `port` | — | Serial port, required |
| `-f`, `--freq` | `2400` | Frequency, MHz. For the RPM-20GS, from 10 to 20000 |
| `-a`, `--atten` | `0` | External attenuation, dB. Added to the sensor dBm |
| `-r`, `--rate` | `0` | Sampling-rate index, see the table below |
| `-v`, `--verbose` | off | One line per sample. Without this flag the terminal refreshes a single line |

| `-r` / `--rate` | Samples per second |
| --- | --- |
| 0 | 10 |
| 1 | 40 |
| 2 | 640 |
| 3 | 1280 |

Without `-v`, the terminal keeps one live reading and rewrites it in place. `-v` prints every sample on its own line. If stdout is redirected to a file or a pipe, samples are also one per line, so the log stays usable.

At `-r 2` and `-r 3` the terminal refreshes at most 20 times per second. Faster printing falls behind and the serial buffer drops frames. A file or a pipe still receives every sample. For normal checks, `0` or `1` is enough.

### Sample output

```text
Connected: RPM-20GS (ARW28340) | ID=104 | FW=1.10 | SN=12345678
Sampling rate index set to: 0
Downloading 218 calibration rows...
  25/218
  218/218
Calibration loaded. Measuring at 2400 MHz | Atten: +30.0 dB (Ctrl+C to stop)

 2400.00 MHz |  +17.60 dBm |    57.5440 mW |  412.18 mV | sensor  -12.40 dBm | atten +30.0 dB
 2400.00 MHz | above calibrated range (sensor overload) |  820.00 mV
```

Without `-v` that reading is a single line, updated in place. With `-v` each reading stays in the scrollback.

The first dBm value is the power you are measuring: the sensor reading plus `-a` / `--atten`, with the same power in mW beside it. Millivolts are the detector level. Sensor and attenuation are printed on their own only when attenuation is not zero. With no attenuator the line is shorter: frequency, dBm, mW, mV.

An unknown model ID prints `Unknown device_id=…` and exits with status 1.
