from __future__ import annotations

import ctypes
import platform
import threading
import time
from collections import deque
from typing import Optional, Tuple

from .lowlevel import (
    IMU_DECIMATION,
    IMU_REPORT_LEN,
    PAGE_VENDOR,
    REPORT_BUF_SIZE,
    REPORT_INTERVAL_US,
    USAGE_ACCEL,
    USAGE_GYRO,
    as_cf_number,
    as_cf_string,
    cf_deref_symbol,
    cf_number_to_int,
    is_root,
    parse_imu_report,
)


def get_spu_provider():
    if platform.system() != "Darwin":
        return None
    if not is_root():
        return None
    try:
        iokit = ctypes.cdll.LoadLibrary("/System/Library/Frameworks/IOKit.framework/IOKit")
        cf = ctypes.cdll.LoadLibrary("/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation")
    except Exception:
        return None

    io_service_matching = iokit.IOServiceMatching
    io_service_matching.argtypes = [ctypes.c_char_p]
    io_service_matching.restype = ctypes.c_void_p

    io_service_get_matching_services = iokit.IOServiceGetMatchingServices
    io_service_get_matching_services.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint32)]
    io_service_get_matching_services.restype = ctypes.c_int32

    io_iterator_next = iokit.IOIteratorNext
    io_iterator_next.argtypes = [ctypes.c_uint32]
    io_iterator_next.restype = ctypes.c_uint32

    io_object_release = iokit.IOObjectRelease
    io_object_release.argtypes = [ctypes.c_uint32]
    io_object_release.restype = ctypes.c_int32

    io_registry_entry_create_cf_property = iokit.IORegistryEntryCreateCFProperty
    io_registry_entry_create_cf_property.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32]
    io_registry_entry_create_cf_property.restype = ctypes.c_void_p

    io_registry_entry_set_cf_prop = iokit.IORegistryEntrySetCFProperty
    io_registry_entry_set_cf_prop.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_void_p]
    io_registry_entry_set_cf_prop.restype = ctypes.c_int32

    io_hid_device_create = iokit.IOHIDDeviceCreate
    io_hid_device_create.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    io_hid_device_create.restype = ctypes.c_void_p

    io_hid_device_open = iokit.IOHIDDeviceOpen
    io_hid_device_open.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    io_hid_device_open.restype = ctypes.c_int32

    io_hid_device_register_input_report = iokit.IOHIDDeviceRegisterInputReportCallback
    io_hid_device_register_input_report.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_long,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]

    io_hid_device_schedule = iokit.IOHIDDeviceScheduleWithRunLoop
    io_hid_device_schedule.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]

    cf_run_loop_get_current = cf.CFRunLoopGetCurrent
    cf_run_loop_get_current.argtypes = []
    cf_run_loop_get_current.restype = ctypes.c_void_p

    cf.CFStringCreateWithCString.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32]
    cf.CFStringCreateWithCString.restype = ctypes.c_void_p

    cf.CFNumberCreate.argtypes = [ctypes.c_void_p, ctypes.c_int32, ctypes.c_void_p]
    cf.CFNumberCreate.restype = ctypes.c_void_p

    cf.CFNumberGetValue.argtypes = [ctypes.c_void_p, ctypes.c_int32, ctypes.c_void_p]
    cf.CFNumberGetValue.restype = ctypes.c_bool

    cf_run_loop_run = cf.CFRunLoopRunInMode
    cf_run_loop_run.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_bool]
    cf_run_loop_run.restype = ctypes.c_int32

    k_cf_allocator_default = cf_deref_symbol(cf, "kCFAllocatorDefault")

    def _service_property_int(service: int, key: str) -> Optional[int]:
        key_ref = as_cf_string(cf, key)
        ref = io_registry_entry_create_cf_property(service, key_ref, None, 0)
        if ref == 0:
            return None
        return cf_number_to_int(cf, ref)

    class SPUHIDProvider:
        name = "spu-hid"

        def __init__(self) -> None:
            self._queue: deque[Tuple[float, float, float]] = deque(maxlen=256)
            self._gyro_queue: deque[Tuple[float, float, float]] = deque(maxlen=256)
            self._lock = threading.Lock()
            self._running = True
            self._running_event = threading.Event()
            self._thread: Optional[threading.Thread] = None
            self._stopped = threading.Event()
            self._ready_error: Optional[str] = None
            self._first_report_event = threading.Event()
            self._first_gyro_report_event = threading.Event()
            self._decimate = 0
            self._last_report_time = time.monotonic()
            self._last_gyro_report_time = time.monotonic()
            self._calibrated = False
            self._x_offset = 0.0
            self._y_offset = 0.0
            self._calibrate_requested_at = time.time()
            self._x = 0.0
            self._y = 0.0
            self._z = 1.0
            self._gx = 0.0
            self._gy = 0.0
            self._gz = 0.0

            self._c_callbacks = {
                USAGE_ACCEL: self._create_callback(USAGE_ACCEL),
                USAGE_GYRO: self._create_callback(USAGE_GYRO),
            }
            self._run_loop_mode = cf_deref_symbol(cf, "kCFRunLoopDefaultMode")
            if not self._run_loop_mode:
                self._run_loop_mode = as_cf_string(cf, "kCFRunLoopDefaultMode")
            if not self._run_loop_mode:
                raise RuntimeError("CF symbol kCFRunLoopDefaultMode unavailable")

            self._report_buffer_refs: list[ctypes.Array[ctypes.c_ubyte]] = []
            self._thread = threading.Thread(target=self._run, name="tiltball-spu", daemon=True)
            self._thread.start()
            if not self._running_event.wait(timeout=1.0):
                raise RuntimeError("spu hid provider did not start in time")
            if self._ready_error:
                raise RuntimeError(self._ready_error)
            if not self._first_report_event.wait(timeout=1.5):
                raise RuntimeError("spu hid provider started but produced no accelerometer reports")

        def _create_callback(self, usage: int):
            callback_type = ctypes.CFUNCTYPE(
                None,
                ctypes.c_void_p,
                ctypes.c_int32,
                ctypes.c_void_p,
                ctypes.c_int32,
                ctypes.c_uint32,
                ctypes.c_void_p,
                ctypes.c_long,
            )

            def _cb(context: ctypes.c_void_p, result: int, sender: int, report_type: int, report_id: int, report: ctypes.c_void_p, length: int) -> None:
                _ = (context, result, sender, report_type, report_id)
                if not self._running or length != IMU_REPORT_LEN:
                    return

                sample = parse_imu_report(ctypes.string_at(report, length))
                with self._lock:
                    if usage == USAGE_ACCEL:
                        self._decimate += 1
                        if self._decimate < IMU_DECIMATION:
                            return
                        self._decimate = 0
                        self._queue.append(sample)
                        self._x, self._y, self._z = sample
                        self._last_report_time = time.monotonic()
                        self._first_report_event.set()
                    elif usage == USAGE_GYRO:
                        self._gyro_queue.append(sample)
                        self._gx, self._gy, self._gz = sample
                        self._last_gyro_report_time = time.monotonic()
                        self._first_gyro_report_event.set()

            return callback_type(_cb)

        def _set_property(self, service: int, key: str, value: int) -> bool:
            key_ref = as_cf_string(cf, key)
            value_ref = as_cf_number(cf, value)
            if key_ref == 0 or value_ref == 0:
                return False
            return io_registry_entry_set_cf_prop(service, key_ref, value_ref) == 0

        def _setup_driver(self, run_loop: int) -> None:
            matching = io_service_matching(b"AppleSPUHIDDriver")
            if matching is None:
                raise RuntimeError("IOServiceMatching(AppleSPUHIDDriver) failed")

            iterator = ctypes.c_uint32()
            if io_service_get_matching_services(0, matching, ctypes.byref(iterator)) != 0:
                raise RuntimeError("IOServiceGetMatchingServices for AppleSPUHIDDriver failed")

            found_driver = False
            while True:
                service = io_iterator_next(iterator.value)
                if service == 0:
                    break
                found_driver = True
                self._set_property(service, "SensorPropertyReportingState", 1)
                self._set_property(service, "SensorPropertyPowerState", 1)
                self._set_property(service, "ReportInterval", REPORT_INTERVAL_US)
                io_object_release(service)

            if not found_driver:
                io_object_release(iterator.value)
                raise RuntimeError("No AppleSPUHIDDriver service found")

            io_object_release(iterator.value)
            self._wake_hid_devices(run_loop)

        def _wake_hid_devices(self, run_loop: int) -> None:
            matching = io_service_matching(b"AppleSPUHIDDevice")
            if matching is None:
                raise RuntimeError("IOServiceMatching(AppleSPUHIDDevice) failed")

            iterator = ctypes.c_uint32()
            if io_service_get_matching_services(0, matching, ctypes.byref(iterator)) != 0:
                raise RuntimeError("IOServiceGetMatchingServices for AppleSPUHIDDevice failed")

            devices_found = 0
            while True:
                service = io_iterator_next(iterator.value)
                if service == 0:
                    break
                up = _service_property_int(service, "PrimaryUsagePage")
                usage = _service_property_int(service, "PrimaryUsage")
                if up == PAGE_VENDOR and usage in {USAGE_ACCEL, USAGE_GYRO}:
                    hid = io_hid_device_create(k_cf_allocator_default, service)
                    if hid and io_hid_device_open(hid, 0) == 0:
                        report_buf = (ctypes.c_ubyte * REPORT_BUF_SIZE)()
                        self._report_buffer_refs.append(report_buf)
                        buf_ptr = ctypes.cast(report_buf, ctypes.c_void_p)
                        callback = self._c_callbacks.get(usage)
                        if callback is not None:
                            io_hid_device_register_input_report(hid, buf_ptr, REPORT_BUF_SIZE, callback, ctypes.cast(0, ctypes.c_void_p))
                        io_hid_device_schedule(hid, run_loop, self._run_loop_mode)
                        devices_found += 1
                io_object_release(service)

            io_object_release(iterator.value)
            if devices_found == 0:
                raise RuntimeError("No AppleSPUHIDDevice accelerometer endpoint found")

            self._running_event.set()

        def close(self) -> None:
            self._running = False
            self._stopped.set()
            if self._thread is not None:
                self._thread.join(timeout=1.5)

        def reset_reference(self) -> None:
            _, _, _ = self.sample()
            with self._lock:
                self._x_offset = self._x
                self._y_offset = self._y
                self._calibrated = True
                self._calibrate_requested_at = time.time()

        def sample(self) -> Tuple[float, float, float]:
            with self._lock:
                if self._queue:
                    self._x, self._y, self._z = self._queue.pop()
                x, y, z = self._x, self._y, self._z

            if not self._calibrated and (time.time() - self._calibrate_requested_at) >= 1.0:
                self._calibrated = True
                self._x_offset = x
                self._y_offset = y

            if not self._calibrated:
                return 0.0, 0.0, z

            x -= self._x_offset
            y -= self._y_offset

            if abs(x) < 0.02:
                x = 0.0
            if abs(y) < 0.02:
                y = 0.0

            return x, y, z

        def sample_gyro(self) -> Tuple[float, float, float]:
            with self._lock:
                if self._gyro_queue:
                    self._gx, self._gy, self._gz = self._gyro_queue.pop()
                return self._gx, self._gy, self._gz

        def sample_all(self) -> dict[str, Tuple[float, float, float]]:
            accel = self.sample()
            gyro = self.sample_gyro()
            return {"accel": accel, "gyro": gyro}

        @property
        def gyro_available(self) -> bool:
            return self._first_gyro_report_event.is_set()

        def is_alive(self, timeout_s: float = 1.0) -> bool:
            with self._lock:
                return (time.monotonic() - self._last_report_time) <= timeout_s

        def _run(self) -> None:
            try:
                run_loop = cf_run_loop_get_current()
                self._setup_driver(int(run_loop))
            except Exception as exc:
                self._ready_error = str(exc)
                self._running_event.set()
                return

            self._running_event.set()
            while not self._stopped.is_set():
                cf_run_loop_run(self._run_loop_mode, 0.05, False)

    return SPUHIDProvider
