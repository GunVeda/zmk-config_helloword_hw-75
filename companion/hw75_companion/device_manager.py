import logging
import threading
from typing import Callable, Optional

import zmkx
from zmkx import comm_pb2 as comm

from .constants import POLL_INTERVAL

logger = logging.getLogger(__name__)


class DeviceInfo:
    """Cached info about a connected device."""

    def __init__(self, device: zmkx.Device):
        self.path = device.path
        self.serial: Optional[str] = None
        self.manufacturer: Optional[str] = None
        self.product: Optional[str] = None
        self.has_rgb = False
        self.has_rgb_full_control = False
        self.has_rgb_indicator = False
        self.has_eink = False
        self.has_knob = False
        self.has_knob_prefs = False
        self.has_lamparray = False

    def refresh(self, device: zmkx.Device):
        with device.open() as dev:
            self.serial = dev.serial
            self.manufacturer = dev.manufacturer
            self.product = dev.product
            ver = dev.version()
            feat = ver.features
            self.has_rgb = feat.rgb
            self.has_rgb_full_control = feat.rgb_full_control
            self.has_rgb_indicator = feat.rgb_indicator
            self.has_eink = feat.eink
            self.has_knob = feat.knob
            self.has_knob_prefs = feat.knob_prefs
            self.has_lamparray = feat.lamparray
        logger.info(
            "Device %s: rgb=%s knob=%s eink=%s indicator=%s lamparray=%s",
            self.serial, self.has_rgb, self.has_knob,
            self.has_eink, self.has_rgb_indicator, self.has_lamparray,
        )


def _call_raw(device: zmkx.Device, h2d: comm.MessageH2D) -> comm.MessageD2H:
    """Send a raw protobuf command via the device's internal _call."""
    with device.open() as dev:
        return dev._call(h2d)


def get_knob_config(device: zmkx.Device) -> comm.KnobConfig:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.KNOB_GET_CONFIG
    d2h = _call_raw(device, h2d)
    return d2h.knob_config


def set_knob_config(device: zmkx.Device, config: comm.KnobConfig) -> comm.KnobConfig:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.KNOB_SET_CONFIG
    h2d.knob_config.CopyFrom(config)
    d2h = _call_raw(device, h2d)
    return d2h.knob_config


def update_knob_pref(device: zmkx.Device, pref: comm.KnobConfig.Pref) -> comm.KnobConfig.Pref:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.KNOB_UPDATE_PREF
    h2d.knob_pref.CopyFrom(pref)
    d2h = _call_raw(device, h2d)
    return d2h.knob_pref


def rgb_control(device: zmkx.Device, command: comm.RgbControl.Command) -> None:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.RGB_CONTROL
    h2d.rgb_control.command = command
    _call_raw(device, h2d)


def rgb_get_state(device: zmkx.Device) -> comm.RgbState:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.RGB_GET_STATE
    d2h = _call_raw(device, h2d)
    return d2h.rgb_state


def rgb_set_state(device: zmkx.Device, state: comm.RgbState) -> comm.RgbState:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.RGB_SET_STATE
    h2d.rgb_state.CopyFrom(state)
    d2h = _call_raw(device, h2d)
    return d2h.rgb_state


def rgb_get_indicator(device: zmkx.Device) -> comm.RgbIndicator:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.RGB_GET_INDICATOR
    d2h = _call_raw(device, h2d)
    return d2h.rgb_indicator


def rgb_set_indicator(device: zmkx.Device, indicator: comm.RgbIndicator) -> comm.RgbIndicator:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.RGB_SET_INDICATOR
    h2d.rgb_indicator.CopyFrom(indicator)
    d2h = _call_raw(device, h2d)
    return d2h.rgb_indicator


def lamp_get_mode(device: zmkx.Device) -> comm.LampMode:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.LAMP_GET_MODE
    d2h = _call_raw(device, h2d)
    return d2h.lamp_mode


def lamp_set_mode(device: zmkx.Device, mode: comm.LampMode) -> comm.LampMode:
    h2d = comm.MessageH2D()
    h2d.action = comm.Action.LAMP_SET_MODE
    h2d.lamp_mode.CopyFrom(mode)
    d2h = _call_raw(device, h2d)
    return d2h.lamp_mode


class DeviceManager:
    """Background thread that polls for USB device changes."""

    def __init__(
        self,
        on_connect: Callable[[DeviceInfo], None],
        on_disconnect: Callable[[DeviceInfo], None],
    ):
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._devices: dict[str, DeviceInfo] = {}  # serial -> DeviceInfo
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=POLL_INTERVAL + 2)

    @property
    def devices(self) -> dict[str, DeviceInfo]:
        with self._lock:
            return dict(self._devices)

    def _poll_loop(self):
        while not self._stop.is_set():
            try:
                self._poll()
            except Exception:
                logger.exception("Error during device poll")
            self._stop.wait(POLL_INTERVAL)

    def _poll(self):
        try:
            found = zmkx.find_devices()
        except Exception:
            logger.debug("find_devices failed", exc_info=True)
            return

        seen_serials: set[str] = set()

        for device in found:
            try:
                info = DeviceInfo(device)
                info.refresh(device)
            except Exception:
                logger.warning("Failed to query device", exc_info=True)
                continue

            serial = info.serial or ""
            if not serial:
                continue
            seen_serials.add(serial)

            with self._lock:
                if serial not in self._devices:
                    self._devices[serial] = info
                    logger.info("Device connected: %s", serial)
                    self._on_connect(info)

        with self._lock:
            gone = [s for s in self._devices if s not in seen_serials]
            for serial in gone:
                info = self._devices.pop(serial)
                logger.info("Device disconnected: %s", serial)
                self._on_disconnect(info)
