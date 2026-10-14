import logging
import queue
import threading
from typing import Any, Callable, Optional

import zmkx
from zmkx import comm_pb2 as comm

from .device_manager import (
    DeviceInfo,
    get_knob_config,
    set_knob_config,
    update_knob_pref,
    rgb_control,
    rgb_get_state,
    rgb_set_state,
    rgb_get_indicator,
    rgb_set_indicator,
    lamp_get_mode,
    lamp_set_mode,
)

logger = logging.getLogger(__name__)


def _find_device(serial: str) -> Optional[zmkx.Device]:
    """Re-enumerate to find a device matching the given serial."""
    for device in zmkx.find_devices():
        try:
            with device.open() as dev:
                if dev.serial == serial:
                    return device
        except Exception:
            continue
    return None


class CommandExecutor:
    """Single-threaded command queue that processes zmkx SDK calls."""

    def __init__(self):
        self._queue: queue.Queue = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._queue.put(None)
        if self._thread:
            self._thread.join(timeout=5)

    def submit(self, serial: str, fn: Callable, callback: Optional[Callable] = None):
        """Queue a command. fn(device) will be called with an opened zmkx.Device."""
        self._queue.put((serial, fn, callback))

    def _run(self):
        while not self._stop.is_set():
            item = self._queue.get()
            if item is None:
                break
            serial, fn, callback = item
            try:
                device = _find_device(serial)
                if device is None:
                    logger.warning("Device %s not found for command", serial)
                    continue
                result = fn(device)
                if callback:
                    callback(result)
            except Exception:
                logger.exception("Command failed for device %s", serial)

    # -- High-level command helpers --

    def knob_set_mode(self, info: DeviceInfo, mode: int, callback: Optional[Callable] = None):
        def cmd(device):
            config = get_knob_config(device)
            config.mode = mode
            result = set_knob_config(device, config)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)

    def knob_set_pref_mode(self, info: DeviceInfo, pref_index: int, mode: int,
                           callback: Optional[Callable] = None):
        def cmd(device):
            config = get_knob_config(device)
            if pref_index < len(config.prefs):
                pref = config.prefs[pref_index]
                pref.mode = mode
                result = update_knob_pref(device, pref)
                if callback:
                    callback(result)
                return result
        self.submit(info.serial, cmd)

    def rgb_toggle(self, info: DeviceInfo, callback: Optional[Callable] = None):
        def cmd(device):
            state = rgb_get_state(device)
            if state.on:
                rgb_control(device, comm.RgbControl.Command.RGB_OFF)
                new_state = False
            else:
                rgb_control(device, comm.RgbControl.Command.RGB_ON)
                new_state = True
            if callback:
                callback(new_state)
            return new_state
        self.submit(info.serial, cmd)

    def rgb_set_effect(self, info: DeviceInfo, effect: int, callback: Optional[Callable] = None):
        def cmd(device):
            state = rgb_get_state(device)
            new_state = comm.RgbState()
            new_state.on = state.on
            new_state.color.CopyFrom(state.color)
            new_state.effect = effect
            new_state.speed = state.speed
            result = rgb_set_state(device, new_state)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)

    def rgb_send_command(self, info: DeviceInfo, command: int, callback: Optional[Callable] = None):
        def cmd(device):
            rgb_control(device, command)
            state = rgb_get_state(device)
            if callback:
                callback(state)
            return state
        self.submit(info.serial, cmd)

    def indicator_set_enable(self, info: DeviceInfo, enable: bool,
                             callback: Optional[Callable] = None):
        def cmd(device):
            ind = comm.RgbIndicator()
            ind.enable = enable
            result = rgb_set_indicator(device, ind)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)

    def indicator_set_brightness(self, info: DeviceInfo, active: Optional[int] = None,
                                 inactive: Optional[int] = None,
                                 callback: Optional[Callable] = None):
        def cmd(device):
            ind = comm.RgbIndicator()
            if active is not None:
                ind.brightness_active = active
            if inactive is not None:
                ind.brightness_inactive = inactive
            result = rgb_set_indicator(device, ind)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)

    def eink_upload(self, info: DeviceInfo, image_bytes: bytes,
                    x: int, y: int, width: int, height: int, partial: bool = False,
                    callback: Optional[Callable] = None):
        def cmd(device):
            with device.open() as dev:
                result = dev.eink_set_image(image_bytes, x, y, width, height, partial)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)

    def lamp_query_mode(self, info: DeviceInfo,
                        callback: Optional[Callable] = None):
        def cmd(device):
            result = lamp_get_mode(device)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)

    def lamp_toggle_mode(self, info: DeviceInfo,
                         callback: Optional[Callable] = None):
        def cmd(device):
            current = lamp_get_mode(device)
            next_mode = comm.LampMode()
            next_mode.mode = 0 if current.mode else 1
            result = lamp_set_mode(device, next_mode)
            if callback:
                callback(result)
            return result
        self.submit(info.serial, cmd)
