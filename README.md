# macspu-accel

`macspu-accel` provides private macOS accelerometer access through the
`AppleSPUHIDDevice` path exposed by IOKit.

## Warning

This uses private Apple interfaces and is not App Store safe. It may break
across macOS updates.

## Requirements

- macOS (Darwin)
- Python 3.11+
- Root privileges (`sudo`) on most machines

## Install

```bash
pip install macspu-accel
```

For local development from source:

```bash
pip install -e .
```

## Build & Publish

```bash
make build
make check
# make publish-test
# make publish
```

Build artifacts are written to:

- `dist/macspu_accel-<version>.tar.gz`
- `dist/macspu_accel-<version>-py3-none-any.whl`

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

## API

- `is_root() -> bool`
- `get_spu_provider() -> type | None`

Provider instances support:

- `.sample() -> tuple[float, float, float]`
- `.reset_reference() -> None`
- `.is_alive(timeout_s: float = 1.0) -> bool`
- `.close() -> None`
