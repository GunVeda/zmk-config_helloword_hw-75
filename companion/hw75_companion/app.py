import logging
import threading

from .commands import CommandExecutor
from .device_manager import DeviceManager, DeviceInfo
from .tray import TrayIcon

logger = logging.getLogger(__name__)


class HW75CompanionApp:
    def __init__(self):
        self._executor = CommandExecutor()
        self._tray = TrayIcon(self._executor, on_exit=self.shutdown)
        self._monitor = DeviceManager(
            on_connect=self._on_connect,
            on_disconnect=self._on_disconnect,
        )
        self._shutdown_event = threading.Event()

    def run(self):
        logger.info("Starting HW-75 Companion")
        self._executor.start()
        self._monitor.start()
        self._tray.start()
        logger.info("Companion running in system tray")
        self._shutdown_event.wait()

    def shutdown(self):
        logger.info("Shutting down")
        self._monitor.stop()
        self._tray.stop()
        self._executor.stop()
        self._shutdown_event.set()

    def _on_connect(self, info: DeviceInfo):
        self._tray.on_device_connected(info)

    def _on_disconnect(self, info: DeviceInfo):
        self._tray.on_device_disconnected(info)
