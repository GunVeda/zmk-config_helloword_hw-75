import logging
import tkinter as tk
from tkinter import filedialog
from typing import Callable, Optional

import pystray
from PIL import Image, ImageDraw

from zmkx import comm_pb2 as comm

from .commands import CommandExecutor
from .constants import (
    INDICATOR_BRIGHTNESS_LEVELS,
    KNOB_MODES,
    RGB_EFFECTS,
)
from .device_manager import DeviceInfo

logger = logging.getLogger(__name__)

ICON_SIZE = 64


def _make_icon(color: str) -> Image.Image:
    img = Image.new("RGBA", (ICON_SIZE, ICON_SIZE), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    margin = 4
    draw.rounded_rectangle(
        [margin, margin, ICON_SIZE - margin, ICON_SIZE - margin],
        radius=12,
        fill=color,
    )
    cx, cy = ICON_SIZE // 2, ICON_SIZE // 2
    r = 14
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill="white")
    draw.ellipse([cx - 4, cy - 4, cx + 4, cy + 4], fill=color)
    return img


ICON_CONNECTED = _make_icon("#4CAF50")
ICON_DISCONNECTED = _make_icon("#9E9E9E")


class TrayIcon:
    def __init__(self, executor: CommandExecutor, on_exit: Callable):
        self._executor = executor
        self._on_exit = on_exit
        self._devices: dict[str, DeviceInfo] = {}
        self._icon: Optional[pystray.Icon] = None
        self._rgb_on: dict[str, bool] = {}
        self._indicator_on: dict[str, bool] = {}
        self._lamp_slave_mode: dict[str, bool] = {}

    def start(self):
        self._icon = pystray.Icon(
            "hw75",
            ICON_DISCONNECTED,
            "HW-75 Companion",
            menu=self._build_menu(),
        )
        self._icon.run_detached()

    def stop(self):
        if self._icon:
            self._icon.stop()

    def on_device_connected(self, info: DeviceInfo):
        self._devices[info.serial] = info
        self._rgb_on[info.serial] = True
        self._indicator_on[info.serial] = False
        self._lamp_slave_mode[info.serial] = False
        if info.has_lamparray:
            # Query the actual mode from firmware so the menu reflects the
            # state Windows may have already changed.
            self._executor.lamp_query_mode(
                info, callback=lambda m: self._on_lamp_mode(info.serial, m)
            )
        self._refresh()

    def on_device_disconnected(self, info: DeviceInfo):
        self._devices.pop(info.serial, None)
        self._rgb_on.pop(info.serial, None)
        self._indicator_on.pop(info.serial, None)
        self._lamp_slave_mode.pop(info.serial, None)
        self._refresh()

    def _on_lamp_mode(self, serial: str, mode):
        self._lamp_slave_mode[serial] = bool(mode.mode)
        self._refresh()

    def _refresh(self):
        if not self._icon:
            return
        if self._devices:
            self._icon.icon = ICON_CONNECTED
            self._icon.title = f"HW-75 Companion ({len(self._devices)} device(s))"
        else:
            self._icon.icon = ICON_DISCONNECTED
            self._icon.title = "HW-75 Companion (disconnected)"
        try:
            self._icon.menu = self._build_menu()
            self._icon.update_menu()
        except Exception:
            logger.debug("Menu update skipped", exc_info=True)

    def _build_menu(self) -> pystray.Menu:
        items = []
        if not self._devices:
            items.append(pystray.MenuItem("No device connected", None, enabled=False))
        else:
            for info in self._devices.values():
                items.extend(self._device_menu(info))
                items.append(pystray.Menu.SEPARATOR)

        items.append(pystray.MenuItem("Exit", self._on_exit))
        return pystray.Menu(*items)

    def _device_menu(self, info: DeviceInfo) -> list:
        label = info.product or info.serial
        items = [pystray.MenuItem(f"🔗 {label}", None, enabled=False)]

        if info.has_knob:
            items.append(pystray.MenuItem("Knob Mode", pystray.Menu(
                *[
                    pystray.MenuItem(
                        name,
                        lambda _, m=mode, i=info: self._set_knob_mode(i, m),
                    )
                    for name, mode in KNOB_MODES.items()
                ]
            )))

        if info.has_rgb:
            items.append(pystray.MenuItem(
                lambda: f"RGB: {'ON' if self._rgb_on.get(info.serial, True) else 'OFF'}",
                lambda: self._toggle_rgb(info),
            ))
            if info.has_rgb_full_control:
                items.append(pystray.MenuItem("RGB Effect", pystray.Menu(
                    *[
                        pystray.MenuItem(
                            name,
                            lambda _, e=eff, i=info: self._set_rgb_effect(i, e),
                        )
                        for name, eff in RGB_EFFECTS.items()
                    ]
                )))

        if info.has_eink:
            items.append(pystray.MenuItem(
                "Upload E-Ink Image...",
                lambda: self._upload_eink(info),
            ))

        if info.has_rgb_indicator:
            items.append(pystray.MenuItem(
                lambda: f"Indicator: {'ON' if self._indicator_on.get(info.serial, False) else 'OFF'}",
                lambda: self._toggle_indicator(info),
            ))
            items.append(pystray.MenuItem("Indicator Brightness (Active)", pystray.Menu(
                *[
                    pystray.MenuItem(
                        f"{lvl}%",
                        lambda _, v=lvl, i=info: self._set_indicator_brightness(i, active=v),
                    )
                    for lvl in INDICATOR_BRIGHTNESS_LEVELS
                ]
            )))
            items.append(pystray.MenuItem("Indicator Brightness (Inactive)", pystray.Menu(
                *[
                    pystray.MenuItem(
                        f"{lvl}%",
                        lambda _, v=lvl, i=info: self._set_indicator_brightness(i, inactive=v),
                    )
                    for lvl in INDICATOR_BRIGHTNESS_LEVELS
                ]
            )))

        if info.has_lamparray:
            items.append(pystray.MenuItem(
                lambda: (
                    "Lighting: SLAVE (Windows Dynamic Lighting)"
                    if self._lamp_slave_mode.get(info.serial, False)
                    else "Lighting: AUTONOMOUS (firmware effects)"
                ),
                lambda: self._toggle_lamp_mode(info),
            ))

        return items

    def _set_knob_mode(self, info: DeviceInfo, mode: int):
        logger.info("Set knob mode %s for %s", mode, info.serial)
        self._executor.knob_set_mode(info, mode, callback=lambda _: self._refresh())

    def _toggle_rgb(self, info: DeviceInfo):
        def on_result(new_state):
            self._rgb_on[info.serial] = new_state
            self._refresh()

        self._executor.rgb_toggle(info, callback=on_result)

    def _set_rgb_effect(self, info: DeviceInfo, effect: int):
        self._executor.rgb_set_effect(info, effect, callback=lambda _: self._refresh())

    def _toggle_indicator(self, info: DeviceInfo):
        current = self._indicator_on.get(info.serial, False)

        def on_result(_):
            self._indicator_on[info.serial] = not current
            self._refresh()

        self._executor.indicator_set_enable(info, not current, callback=on_result)

    def _set_indicator_brightness(self, info: DeviceInfo, active=None, inactive=None):
        self._executor.indicator_set_brightness(info, active=active, inactive=inactive)

    def _toggle_lamp_mode(self, info: DeviceInfo):
        def on_result(new_state):
            self._lamp_slave_mode[info.serial] = bool(new_state.mode)
            self._refresh()

        self._executor.lamp_toggle_mode(info, callback=on_result)

    def _upload_eink(self, info: DeviceInfo):
        root = tk.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(
            title="Select E-Ink Image",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp *.gif"), ("All files", "*.*")],
        )
        root.destroy()
        if not path:
            return

        try:
            from PIL import Image as PILImage
            img = PILImage.open(path)
            canvas = PILImage.new("1", (128, 296), 255)
            img = img.convert("1")
            img.thumbnail((128, 296))
            x = (128 - img.width) // 2
            y = (296 - img.height) // 2
            canvas.paste(img, (x, y))
            self._executor.eink_upload(
                info, canvas.tobytes(), 0, 0, 128, 296,
                callback=lambda _: logger.info("E-Ink image uploaded"),
            )
        except Exception:
            logger.exception("Failed to upload e-ink image")
