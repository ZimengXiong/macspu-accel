# macspu-accel

Small Python wrapper for the private macOS SPU accelerometer.

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
    x, y, z = sensor.sample()
    print(x, y, z)
finally:
    sensor.close()
```

## Notes

This uses private Apple interfaces and may break across macOS updates.
