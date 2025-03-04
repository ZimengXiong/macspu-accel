from __future__ import annotations

import ctypes
import os
import struct
from typing import Optional, Tuple

IMU_REPORT_LEN = 22
IMU_DATA_OFFSET = 6
IMU_DECIMATION = 8
IMU_SCALE = 65536.0
REPORT_BUF_SIZE = 4096
REPORT_INTERVAL_US = 1000

CF_STRING_ENCODING_UTF8 = 0x08000100
CF_NUMBER_SINT32 = 3
CF_NUMBER_SINT64 = 4

PAGE_VENDOR = 0xFF00
USAGE_ACCEL = 3

_cf_string_cache: dict[str, int] = {}
_cf_buffer_cache: list[ctypes.Array[ctypes.c_char]] = []


def is_root() -> bool:
    return hasattr(os, "geteuid") and os.geteuid() == 0


def cf_deref_symbol(cf_module: ctypes.CDLL, name: str) -> int:
    try:
        raw = ctypes.c_void_p.in_dll(cf_module, name)
        if not raw.value:
            return 0
    except Exception:
        return 0

    try:
        ptr = ctypes.cast(raw.value, ctypes.POINTER(ctypes.c_void_p)).contents.value
        return ptr or raw.value
    except Exception:
        return raw.value


def as_cf_string(cf_module: ctypes.CDLL, value: str) -> int:
    if value in _cf_string_cache:
        return _cf_string_cache[value]

    raw = ctypes.create_string_buffer(value.encode("utf-8") + b"\x00")
    ref = cf_module.CFStringCreateWithCString(0, raw, CF_STRING_ENCODING_UTF8)
    if not ref:
        return 0

    _cf_buffer_cache.append(raw)
    _cf_string_cache[value] = int(ref)
    return int(ref)


def as_cf_number(cf_module: ctypes.CDLL, value: int) -> int:
    packed = ctypes.c_int32(value)
    return cf_module.CFNumberCreate(0, CF_NUMBER_SINT32, ctypes.byref(packed))


def cf_number_to_int(cf_module: ctypes.CDLL, value_ref: int) -> Optional[int]:
    if value_ref == 0:
        return None
    out = ctypes.c_int64()
    ok = cf_module.CFNumberGetValue(value_ref, CF_NUMBER_SINT64, ctypes.byref(out))
    if not ok:
        return None
    return int(out.value)


def parse_imu_report(raw: bytes) -> Tuple[float, float, float]:
    if len(raw) < IMU_DATA_OFFSET + 12:
        return 0.0, 0.0, 1.0
    gx, gy, gz = struct.unpack_from("<3i", raw, IMU_DATA_OFFSET)
    return (
        float(gx) / IMU_SCALE,
        float(gy) / IMU_SCALE,
        float(gz) / IMU_SCALE,
    )
