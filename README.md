# CanSat 2025

Source files for the software side of CanSat 2025 - 2026 for our school.

See more information at https://www.stem.org.uk/esero/secondary/competitions-and-challenges/cansat!

## Hub (Windows/Linux)

Run the laptop hub from `laptop/cansat-hub`:

```sh
go run .
```

The hub auto-tries common serial ports:
- Windows: `COM4`, `COM5`, `COM6`, `COM7`
- Linux: `/dev/ttyUSB0-3`, `/dev/ttyACM0-3`

To force a specific port, set `CANSAT_PORT`:

```sh
# Linux/macOS
export CANSAT_PORT=/dev/ttyACM0

# Windows PowerShell
$env:CANSAT_PORT = "COM4"
```

## Offline Log Parsing

Use `parse_flight_log.py` to parse datalogger files and generate graphs.

```sh
python parse_flight_log.py --input path/to/log.txt --output-dir plots
```

Install dependency:

```sh
pip install matplotlib
```
