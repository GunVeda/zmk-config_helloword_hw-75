/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 *
 * Lamp position table for the HW-75 keyboard board (101 WS2812 LEDs).
 *
 * Chassis: 330 mm × 140 mm × 20 mm.
 * Key matrix: 16 columns × 6 rows, 19.05 mm (3/4") pitch.
 * Bezel: ~12.6 mm on each side; top bezel ~12.85 mm.
 *
 * Positions are in micrometers with the chassis top-left corner as (0, 0).
 * The strip order follows the remap in
 * config/boards/arm/hw75_keyboard/hw75_keyboard.dts (led-strip-remap.map):
 *
 *   / * hub  * /      99  98  97  96  95  94  93  92  91  90  89  88  87  86  85 100
 *   / * keys * /      13      12  11  10 ... 0   (F-row, LED 13 leftmost)
 *                     14  15  16  17  18 ... 28  (number row)
 *                     ...
 *                     72  73  74          75 ... 81  (mod row, Space + arrows)
 *   / * status * /    84  83  82  (right-bottom corner, "top/mid/bottom")
 *
 * Per-key LEDs sit directly below the switch center.
 * Hub LEDs run along the back edge in the strip order: 99, 98, ..., 85, 100
 *   (i.e. LED 99 leftmost, LED 100 rightmost).
 * Status LEDs 82/83/84 are stacked vertically at the right-bottom corner
 *   of the chassis (3 indicators spaced ~5 mm apart, top→mid→bottom).
 */

#ifndef HW75_LAMPARRAY_CONFIG_KEYBOARD_H_
#define HW75_LAMPARRAY_CONFIG_KEYBOARD_H_

#define HW75_LAMPARRAY_LAMP_COUNT 101

#define HW75_LAMPARRAY_LAMP_KIND 0x01 /* LampArrayKindKeyboard */

#define HW75_LAMPARRAY_BB_W_MICRONS 330000000UL /* 33 cm */
#define HW75_LAMPARRAY_BB_H_MICRONS 140000000UL /* 14 cm */
#define HW75_LAMPARRAY_BB_D_MICRONS 20000000UL  /* 2 cm */

#define HW75_LAMPARRAY_MIN_UPDATE_US 10000UL /* 10 ms ≈ 100 Hz */

/*
 * Key matrix grid. Each (col c, row r) center sits at:
 *   x = (12.6 + 9.525 + c * 19.05) mm = (22.125 + c * 19.05) mm
 *   y = (12.85 + 9.525 + r * 19.05) mm = (22.375 + r * 19.05) mm
 *
 * The 12.6/12.85 mm is the chassis bezel and 9.525 mm (0.5 u) centers
 * each LED on its 1u key. Wide keys (Backspace 2u, Enter 2.25u,
 * LShift 2.25u, Space 6.25u) are approximated on the 1u grid center —
 * LampArray visualization only needs approximate positions.
 */
#define HW75_KB_X(c) ((22125UL + (c) * 19050UL))
#define HW75_KB_Y(r) ((22375UL + (r) * 19050UL))

/*
 * Per-light purpose defaults. Each lamp defaults to Illumination;
 * status LEDs override to Status, the spare LED overrides to Accent.
 */
#define LAMP_PURPOSE_0  0x10
#define LAMP_PURPOSE_1  0x10
#define LAMP_PURPOSE_2  0x10
#define LAMP_PURPOSE_3  0x10
#define LAMP_PURPOSE_4  0x10
#define LAMP_PURPOSE_5  0x10
#define LAMP_PURPOSE_6  0x10
#define LAMP_PURPOSE_7  0x10
#define LAMP_PURPOSE_8  0x10
#define LAMP_PURPOSE_9  0x10
#define LAMP_PURPOSE_10 0x10
#define LAMP_PURPOSE_11 0x10
#define LAMP_PURPOSE_12 0x10
#define LAMP_PURPOSE_13 0x10
#define LAMP_PURPOSE_14 0x10
#define LAMP_PURPOSE_15 0x10
#define LAMP_PURPOSE_16 0x10
#define LAMP_PURPOSE_17 0x10
#define LAMP_PURPOSE_18 0x10
#define LAMP_PURPOSE_19 0x10
#define LAMP_PURPOSE_20 0x10
#define LAMP_PURPOSE_21 0x10
#define LAMP_PURPOSE_22 0x10
#define LAMP_PURPOSE_23 0x10
#define LAMP_PURPOSE_24 0x10
#define LAMP_PURPOSE_25 0x10
#define LAMP_PURPOSE_26 0x10
#define LAMP_PURPOSE_27 0x10
#define LAMP_PURPOSE_28 0x10
#define LAMP_PURPOSE_29 0x10
#define LAMP_PURPOSE_30 0x10
#define LAMP_PURPOSE_31 0x10
#define LAMP_PURPOSE_32 0x10
#define LAMP_PURPOSE_33 0x10
#define LAMP_PURPOSE_34 0x10
#define LAMP_PURPOSE_35 0x10
#define LAMP_PURPOSE_36 0x10
#define LAMP_PURPOSE_37 0x10
#define LAMP_PURPOSE_38 0x10
#define LAMP_PURPOSE_39 0x10
#define LAMP_PURPOSE_40 0x10
#define LAMP_PURPOSE_41 0x10
#define LAMP_PURPOSE_42 0x10
#define LAMP_PURPOSE_43 0x10
#define LAMP_PURPOSE_44 0x10
#define LAMP_PURPOSE_45 0x10
#define LAMP_PURPOSE_46 0x10
#define LAMP_PURPOSE_47 0x10
#define LAMP_PURPOSE_48 0x10
#define LAMP_PURPOSE_49 0x10
#define LAMP_PURPOSE_50 0x10
#define LAMP_PURPOSE_51 0x10
#define LAMP_PURPOSE_52 0x10
#define LAMP_PURPOSE_53 0x10
#define LAMP_PURPOSE_54 0x10
#define LAMP_PURPOSE_55 0x10
#define LAMP_PURPOSE_56 0x10
#define LAMP_PURPOSE_57 0x10
#define LAMP_PURPOSE_58 0x10
#define LAMP_PURPOSE_59 0x10
#define LAMP_PURPOSE_60 0x10
#define LAMP_PURPOSE_61 0x10
#define LAMP_PURPOSE_62 0x10
#define LAMP_PURPOSE_63 0x10
#define LAMP_PURPOSE_64 0x10
#define LAMP_PURPOSE_65 0x10
#define LAMP_PURPOSE_66 0x10
#define LAMP_PURPOSE_67 0x10
#define LAMP_PURPOSE_68 0x10
#define LAMP_PURPOSE_69 0x10
#define LAMP_PURPOSE_70 0x10
#define LAMP_PURPOSE_71 0x10
#define LAMP_PURPOSE_72 0x10
#define LAMP_PURPOSE_73 0x10
#define LAMP_PURPOSE_74 0x10
#define LAMP_PURPOSE_75 0x10
#define LAMP_PURPOSE_76 0x10
#define LAMP_PURPOSE_77 0x10
#define LAMP_PURPOSE_78 0x10
#define LAMP_PURPOSE_79 0x10
#define LAMP_PURPOSE_80 0x10
#define LAMP_PURPOSE_81 0x10
#define LAMP_PURPOSE_82 0x08 /* Status (top) */
#define LAMP_PURPOSE_83 0x08 /* Status (middle) */
#define LAMP_PURPOSE_84 0x08 /* Status (bottom) */
#define LAMP_PURPOSE_85 0x10
#define LAMP_PURPOSE_86 0x10
#define LAMP_PURPOSE_87 0x10
#define LAMP_PURPOSE_88 0x10
#define LAMP_PURPOSE_89 0x10
#define LAMP_PURPOSE_90 0x10
#define LAMP_PURPOSE_91 0x10
#define LAMP_PURPOSE_92 0x10
#define LAMP_PURPOSE_93 0x10
#define LAMP_PURPOSE_94 0x10
#define LAMP_PURPOSE_95 0x10
#define LAMP_PURPOSE_96 0x10
#define LAMP_PURPOSE_97 0x10
#define LAMP_PURPOSE_98 0x10
#define LAMP_PURPOSE_99 0x10
#define LAMP_PURPOSE_100 0x02 /* Spare — Accent */

/* ---- Per-key LED positions (LEDs 0..81). ---- */

/* Function row (row 0): 14 keys. DTS map reads "13 _ 12 ... 1 0" — LED 13
 * is the leftmost (ESC), LED 0 is the rightmost (Pause/Break).
 */
#define LAMP_X_0   HW75_KB_X(15) /* rightmost (Pause/Break) */
#define LAMP_Y_0   HW75_KB_Y(0)
#define LAMP_X_1   HW75_KB_X(14)
#define LAMP_Y_1   HW75_KB_Y(0)
#define LAMP_X_2   HW75_KB_X(13)
#define LAMP_Y_2   HW75_KB_Y(0)
#define LAMP_X_3   HW75_KB_X(12)
#define LAMP_Y_3   HW75_KB_Y(0)
#define LAMP_X_4   HW75_KB_X(11)
#define LAMP_Y_4   HW75_KB_Y(0)
#define LAMP_X_5   HW75_KB_X(10)
#define LAMP_Y_5   HW75_KB_Y(0)
#define LAMP_X_6   HW75_KB_X(9)
#define LAMP_Y_6   HW75_KB_Y(0)
#define LAMP_X_7   HW75_KB_X(8)
#define LAMP_Y_7   HW75_KB_Y(0)
#define LAMP_X_8   HW75_KB_X(7)
#define LAMP_Y_8   HW75_KB_Y(0)
#define LAMP_X_9   HW75_KB_X(6)
#define LAMP_Y_9   HW75_KB_Y(0)
#define LAMP_X_10  HW75_KB_X(5)
#define LAMP_Y_10  HW75_KB_Y(0)
#define LAMP_X_11  HW75_KB_X(4)
#define LAMP_Y_11  HW75_KB_Y(0)
#define LAMP_X_12  HW75_KB_X(3)
#define LAMP_Y_12  HW75_KB_Y(0)
#define LAMP_X_13  HW75_KB_X(0) /* leftmost (ESC) */
#define LAMP_Y_13  HW75_KB_Y(0)

/* Number row (row 1): 15 keys (` ... =, Backspace 2u). */
#define LAMP_X_14  HW75_KB_X(0)
#define LAMP_Y_14  HW75_KB_Y(1)
#define LAMP_X_15  HW75_KB_X(1)
#define LAMP_Y_15  HW75_KB_Y(1)
#define LAMP_X_16  HW75_KB_X(2)
#define LAMP_Y_16  HW75_KB_Y(1)
#define LAMP_X_17  HW75_KB_X(3)
#define LAMP_Y_17  HW75_KB_Y(1)
#define LAMP_X_18  HW75_KB_X(4)
#define LAMP_Y_18  HW75_KB_Y(1)
#define LAMP_X_19  HW75_KB_X(5)
#define LAMP_Y_19  HW75_KB_Y(1)
#define LAMP_X_20  HW75_KB_X(6)
#define LAMP_Y_20  HW75_KB_Y(1)
#define LAMP_X_21  HW75_KB_X(7)
#define LAMP_Y_21  HW75_KB_Y(1)
#define LAMP_X_22  HW75_KB_X(8)
#define LAMP_Y_22  HW75_KB_Y(1)
#define LAMP_X_23  HW75_KB_X(9)
#define LAMP_Y_23  HW75_KB_Y(1)
#define LAMP_X_24  HW75_KB_X(10)
#define LAMP_Y_24  HW75_KB_Y(1)
#define LAMP_X_25  HW75_KB_X(11)
#define LAMP_Y_25  HW75_KB_Y(1)
#define LAMP_X_26  HW75_KB_X(12)
#define LAMP_Y_26  HW75_KB_Y(1)
#define LAMP_X_27  HW75_KB_X(13)
#define LAMP_Y_27  HW75_KB_Y(1)
#define LAMP_X_28  HW75_KB_X(14) /* Backspace center (2u wide) */
#define LAMP_Y_28  HW75_KB_Y(1)

/* Top alpha (row 2): 15 keys (Tab 1.5u, ..., ], \ 1.5u). */
#define LAMP_X_29  HW75_KB_X(0)  /* Tab center (1.5u wide) */
#define LAMP_Y_29  HW75_KB_Y(2)
#define LAMP_X_30  HW75_KB_X(1)
#define LAMP_Y_30  HW75_KB_Y(2)
#define LAMP_X_31  HW75_KB_X(2)
#define LAMP_Y_31  HW75_KB_Y(2)
#define LAMP_X_32  HW75_KB_X(3)
#define LAMP_Y_32  HW75_KB_Y(2)
#define LAMP_X_33  HW75_KB_X(4)
#define LAMP_Y_33  HW75_KB_Y(2)
#define LAMP_X_34  HW75_KB_X(5)
#define LAMP_Y_34  HW75_KB_Y(2)
#define LAMP_X_35  HW75_KB_X(6)
#define LAMP_Y_35  HW75_KB_Y(2)
#define LAMP_X_36  HW75_KB_X(7)
#define LAMP_Y_36  HW75_KB_Y(2)
#define LAMP_X_37  HW75_KB_X(8)
#define LAMP_Y_37  HW75_KB_Y(2)
#define LAMP_X_38  HW75_KB_X(9)
#define LAMP_Y_38  HW75_KB_Y(2)
#define LAMP_X_39  HW75_KB_X(10)
#define LAMP_Y_39  HW75_KB_Y(2)
#define LAMP_X_40  HW75_KB_X(11)
#define LAMP_Y_40  HW75_KB_Y(2)
#define LAMP_X_41  HW75_KB_X(12)
#define LAMP_Y_41  HW75_KB_Y(2)
#define LAMP_X_42  HW75_KB_X(13)
#define LAMP_Y_42  HW75_KB_Y(2)
#define LAMP_X_43  HW75_KB_X(14) /* \ center (1.5u wide) */
#define LAMP_Y_43  HW75_KB_Y(2)

/* Home alpha (row 3): 14 keys (Caps 1.75u, ..., ', Enter 2.25u). */
#define LAMP_X_44  HW75_KB_X(0)  /* Caps center (1.75u wide) */
#define LAMP_Y_44  HW75_KB_Y(3)
#define LAMP_X_45  HW75_KB_X(1)
#define LAMP_Y_45  HW75_KB_Y(3)
#define LAMP_X_46  HW75_KB_X(2)
#define LAMP_Y_46  HW75_KB_Y(3)
#define LAMP_X_47  HW75_KB_X(3)
#define LAMP_Y_47  HW75_KB_Y(3)
#define LAMP_X_48  HW75_KB_X(4)
#define LAMP_Y_48  HW75_KB_Y(3)
#define LAMP_X_49  HW75_KB_X(5)
#define LAMP_Y_49  HW75_KB_Y(3)
#define LAMP_X_50  HW75_KB_X(6)
#define LAMP_Y_50  HW75_KB_Y(3)
#define LAMP_X_51  HW75_KB_X(7)
#define LAMP_Y_51  HW75_KB_Y(3)
#define LAMP_X_52  HW75_KB_X(8)
#define LAMP_Y_52  HW75_KB_Y(3)
#define LAMP_X_53  HW75_KB_X(9)
#define LAMP_Y_53  HW75_KB_Y(3)
#define LAMP_X_54  HW75_KB_X(10)
#define LAMP_Y_54  HW75_KB_Y(3)
#define LAMP_X_55  HW75_KB_X(11)
#define LAMP_Y_55  HW75_KB_Y(3)
#define LAMP_X_56  HW75_KB_X(13) /* Enter center (2.25u wide) */
#define LAMP_Y_56  HW75_KB_Y(3)
#define LAMP_X_57  HW75_KB_X(14) /* nav column (PgUp cluster) */
#define LAMP_Y_57  HW75_KB_Y(3)

/* Bottom alpha (row 4): 14 keys (LShift 2.25u, ..., /, RShift). */
#define LAMP_X_58  HW75_KB_X(1)  /* LShift center (2.25u wide) */
#define LAMP_Y_58  HW75_KB_Y(4)
#define LAMP_X_59  HW75_KB_X(3)
#define LAMP_Y_59  HW75_KB_Y(4)
#define LAMP_X_60  HW75_KB_X(4)
#define LAMP_Y_60  HW75_KB_Y(4)
#define LAMP_X_61  HW75_KB_X(5)
#define LAMP_Y_61  HW75_KB_Y(4)
#define LAMP_X_62  HW75_KB_X(6)
#define LAMP_Y_62  HW75_KB_Y(4)
#define LAMP_X_63  HW75_KB_X(7)
#define LAMP_Y_63  HW75_KB_Y(4)
#define LAMP_X_64  HW75_KB_X(8)
#define LAMP_Y_64  HW75_KB_Y(4)
#define LAMP_X_65  HW75_KB_X(9)
#define LAMP_Y_65  HW75_KB_Y(4)
#define LAMP_X_66  HW75_KB_X(10)
#define LAMP_Y_66  HW75_KB_Y(4)
#define LAMP_X_67  HW75_KB_X(11)
#define LAMP_Y_67  HW75_KB_Y(4)
#define LAMP_X_68  HW75_KB_X(12)
#define LAMP_Y_68  HW75_KB_Y(4)
#define LAMP_X_69  HW75_KB_X(13)
#define LAMP_Y_69  HW75_KB_Y(4)
#define LAMP_X_70  HW75_KB_X(14) /* Up arrow center */
#define LAMP_Y_70  HW75_KB_Y(4)
#define LAMP_X_71  HW75_KB_X(15) /* nav column */
#define LAMP_Y_71  HW75_KB_Y(4)

/* Mod row (row 5): 10 keys (Ctrl 1.25u, Win, Alt, Space 6.25u, ..., arrows). */
#define LAMP_X_72  HW75_KB_X(0)  /* Ctrl center (1.25u wide) */
#define LAMP_Y_72  HW75_KB_Y(5)
#define LAMP_X_73  HW75_KB_X(1)  /* Win */
#define LAMP_Y_73  HW75_KB_Y(5)
#define LAMP_X_74  HW75_KB_X(2)  /* Alt */
#define LAMP_Y_74  HW75_KB_Y(5)
#define LAMP_X_75  HW75_KB_X(7)  /* Space center (6.25u wide) */
#define LAMP_Y_75  HW75_KB_Y(5)
#define LAMP_X_76  HW75_KB_X(11) /* AltGr / Fn */
#define LAMP_Y_76  HW75_KB_Y(5)
#define LAMP_X_77  HW75_KB_X(13) /* Fn / Ctrl */
#define LAMP_Y_77  HW75_KB_Y(5)
#define LAMP_X_78  HW75_KB_X(14) /* Left arrow */
#define LAMP_Y_78  HW75_KB_Y(5)
#define LAMP_X_79  HW75_KB_X(15) /* Down arrow */
#define LAMP_Y_79  HW75_KB_Y(5)
#define LAMP_X_80  HW75_KB_X(15) /* Right arrow (under down) */
#define LAMP_Y_80  HW75_KB_Y(5)
#define LAMP_X_81  HW75_KB_X(13) /* extra nav */
#define LAMP_Y_81  HW75_KB_Y(5)

/* ---- Status LEDs (82/83/84) at the right-bottom corner. ----
 *
 * Per the DTS comment "status (top/mid/bottom)" these 3 are stacked
 * vertically. They sit at the right edge near the bottom of the
 * chassis, just inside the bezel.
 */
#define LAMP_X_82  317500UL /* 317.5 mm — top indicator */
#define LAMP_Y_82  130000UL
#define LAMP_X_83  317500UL /* middle */
#define LAMP_Y_83  135000UL
#define LAMP_X_84  317500UL /* bottom */
#define LAMP_Y_84  140000UL

/* ---- Hub underglow LEDs (85..99) along the entire back edge. ----
 *
 * DTS remap order from the strip's DIN (left) to DOUT (right):
 *   99 98 97 96 95 94 93 92 91 90 89 88 87 86 85 100
 * 16 LEDs at 19.05 mm pitch (1u) span ≈304.8 mm centered in the 330 mm
 * chassis. LED 99 sits at the leftmost hub position; LED 100 is the
 * rightmost.
 */
#define LAMP_HUB_Y 5000UL /* ~5 mm below the back edge */

#define LAMP_X_99  22125UL    /* leftmost hub LED */
#define LAMP_Y_99  LAMP_HUB_Y
#define LAMP_X_98  41175UL
#define LAMP_Y_98  LAMP_HUB_Y
#define LAMP_X_97  60225UL
#define LAMP_Y_97  LAMP_HUB_Y
#define LAMP_X_96  79275UL
#define LAMP_Y_96  LAMP_HUB_Y
#define LAMP_X_95  98325UL
#define LAMP_Y_95  LAMP_HUB_Y
#define LAMP_X_94  117375UL
#define LAMP_Y_94  LAMP_HUB_Y
#define LAMP_X_93  136425UL
#define LAMP_Y_93  LAMP_HUB_Y
#define LAMP_X_92  155475UL
#define LAMP_Y_92  LAMP_HUB_Y
#define LAMP_X_91  174525UL
#define LAMP_Y_91  LAMP_HUB_Y
#define LAMP_X_90  193575UL
#define LAMP_Y_90  LAMP_HUB_Y
#define LAMP_X_89  212625UL
#define LAMP_Y_89  LAMP_HUB_Y
#define LAMP_X_88  231675UL
#define LAMP_Y_88  LAMP_HUB_Y
#define LAMP_X_87  250725UL
#define LAMP_Y_87  LAMP_HUB_Y
#define LAMP_X_86  269775UL
#define LAMP_Y_86  LAMP_HUB_Y
#define LAMP_X_85  288825UL
#define LAMP_Y_85  LAMP_HUB_Y
#define LAMP_X_100 307875UL   /* rightmost — spare */
#define LAMP_Y_100 LAMP_HUB_Y

#define LAMP_Z_ALL 0UL /* Single-layer strip */

#define HW75_LAMPARRAY_PURPOSE_FOR(i) LAMP_PURPOSE_##i
#define HW75_LAMPARRAY_X_FOR(i)      LAMP_X_##i
#define HW75_LAMPARRAY_Y_FOR(i)      LAMP_Y_##i
#define HW75_LAMPARRAY_Z            LAMP_Z_ALL

#endif /* HW75_LAMPARRAY_CONFIG_KEYBOARD_H_ */