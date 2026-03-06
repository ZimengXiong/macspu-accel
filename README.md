# macspu-accel

Small Python wrapper for the private macOS SPU accelerometer and gyroscope.

## Requirements

- macOS (Darwin)
- Python 3.11+
- `sudo` on most machines

## Install

```bash
pip install macspu-accel
```

## From Source

```bash
pip install -e .
```

## Build

```bash
make build
make check
```

## Usage

```python
from macspu_accel import get_spu_provider

Provider = get_spu_provider()
if Provider is None:
    raise RuntimeError("SPU provider unavailable")

sensor = Provider()
try:
    accel = sensor.sample()
    gyro = sensor.sample_gyro()
    print("accel", accel)
    print("gyro", gyro)
finally:
    sensor.close()
```

`sample()` remains the accelerometer API for compatibility.
Use `sample_gyro()` for angular-rate data and `sample_all()` to fetch both streams together.

## Notes

This uses private Apple interfaces and may break across macOS updates.
