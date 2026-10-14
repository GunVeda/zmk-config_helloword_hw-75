from zmkx import comm_pb2 as comm

POLL_INTERVAL = 5.0

KNOB_MODES = {
    "Disable": comm.KnobConfig.Mode.DISABLE,
    "Inertia": comm.KnobConfig.Mode.INERTIA,
    "Encoder": comm.KnobConfig.Mode.ENCODER,
    "Spring": comm.KnobConfig.Mode.SPRING,
    "Damped": comm.KnobConfig.Mode.DAMPED,
    "Spin": comm.KnobConfig.Mode.SPIN,
    "Ratchet": comm.KnobConfig.Mode.RATCHET,
    "Switch": comm.KnobConfig.Mode.SWITCH,
}

RGB_EFFECTS = {
    "Solid": comm.RgbState.Effect.SOLID,
    "Breathe": comm.RgbState.Effect.BREATHE,
    "Spectrum": comm.RgbState.Effect.SPECTRUM,
    "Swirl": comm.RgbState.Effect.SWIRL,
}

RGB_COMMANDS = {
    "on": comm.RgbControl.Command.RGB_ON,
    "off": comm.RgbControl.Command.RGB_OFF,
    "hui": comm.RgbControl.Command.RGB_HUI,
    "hud": comm.RgbControl.Command.RGB_HUD,
    "sai": comm.RgbControl.Command.RGB_SAI,
    "sad": comm.RgbControl.Command.RGB_SAD,
    "bri": comm.RgbControl.Command.RGB_BRI,
    "brd": comm.RgbControl.Command.RGB_BRD,
    "spi": comm.RgbControl.Command.RGB_SPI,
    "spd": comm.RgbControl.Command.RGB_SPD,
    "eff": comm.RgbControl.Command.RGB_EFF,
    "efr": comm.RgbControl.Command.RGB_EFR,
}

INDICATOR_BRIGHTNESS_LEVELS = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
