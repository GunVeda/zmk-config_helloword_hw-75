/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 *
 * Lamp position table for the HW-75 Dynamic module (4 WS2812 LEDs).
 *
 * Module chassis: 40 mm × 135 mm × 20 mm. PCB thickness: 1.0 mm.
 *
 * Physical layout (from annotated PCB/photo):
 *
 *   ┌────────────────────────────────┐ y = 0  (top)
 *   │  knob     OLED display   ╔══╗  │
 *   │   ┌─┐  ┌─┐                ║▣║   │ y ≈ 15 mm
 *   │   │B│  │B│  (2 buttons)   ╚══╝   │
 *   │   └─┘  └─┘      LED 0     ←1 LED between buttons
 *   │            ▣ (between them) │ y ≈ 20 mm
 *   │                                │
 *   │  ▣ ← LED 3                      │ y ≈ 35 mm
 *   │  ▣ ← LED 2   (3 LEDs in        │ y ≈ 35 mm
 *   │  ▣ ← LED 1    a row at base    │
 *   │           of the knob)          │
 *   │                                │
 *   │      (e-ink display)            │
 *   │                                │
 *   └────────────────────────────────┘ y = 135 (bottom)
 *   x = 0                          x = 40
 *
 * Per the annotations on the photo:
 *   - "这里是左右两个按钮中间夹一个灯" → LED 0 sits between the 2
 *     buttons at the top of the module.
 *   - "这里是三个灯排在一起" → LEDs 1/2/3 form a horizontal row at
 *     the knob's base (visible as a "red track" reflecting the LEDs
 *     through the case).
 *
 * DTS remap map = <0 3 2 1> means the WS2812 chain order is:
 *   LED 0 (between buttons) → LED 3 → LED 2 → LED 1 (knob row).
 */

#ifndef HW75_LAMPARRAY_CONFIG_DYNAMIC_H_
#define HW75_LAMPARRAY_CONFIG_DYNAMIC_H_

#define HW75_LAMPARRAY_LAMP_COUNT 4

#define HW75_LAMPARRAY_LAMP_KIND 0x02 /* LampArrayKindPeripheral */

#define HW75_LAMPARRAY_BB_W_MICRONS 40000000UL  /* 40 mm */
#define HW75_LAMPARRAY_BB_H_MICRONS 135000000UL /* 135 mm */
#define HW75_LAMPARRAY_BB_D_MICRONS 20000000UL  /* 20 mm */

#define HW75_LAMPARRAY_MIN_UPDATE_US 10000UL /* 10 ms ≈ 100 Hz */

/*
 * Per-lamp purposes.
 * LED 0 sits between the two buttons (spare / accent feedback).
 * LEDs 1/2/3 are status indicators driven by the indicator app.
 */
#define LAMP_PURPOSE_0 0x02 /* Accent (between buttons) */
#define LAMP_PURPOSE_1 0x08 /* Status (rightmost in knob row) */
#define LAMP_PURPOSE_2 0x08 /* Status (middle in knob row) */
#define LAMP_PURPOSE_3 0x08 /* Status (leftmost in knob row) */

/*
 * Lamp positions, in micrometers, chassis top-left as origin.
 *
 *   LED 0 (spare, between buttons) ≈ (15 mm, 18 mm)
 *   LED 3 (status, knob row left)  ≈ ( 8 mm, 32 mm)
 *   LED 2 (status, knob row mid)   ≈ (18 mm, 32 mm)
 *   LED 1 (status, knob row right) ≈ (28 mm, 32 mm)
 */
#define LAMP_X_0 15000000UL  /* 15 mm */
#define LAMP_Y_0 18000000UL  /* 18 mm */
#define LAMP_X_3 8000000UL
#define LAMP_Y_3 32000000UL  /* 32 mm */
#define LAMP_X_2 18000000UL
#define LAMP_Y_2 32000000UL  /* 32 mm */
#define LAMP_X_1 28000000UL
#define LAMP_Y_1 32000000UL  /* 32 mm */

#define LAMP_Z_ALL 0UL

#define HW75_LAMPARRAY_PURPOSE_FOR(i) LAMP_PURPOSE_##i
#define HW75_LAMPARRAY_X_FOR(i)      LAMP_X_##i
#define HW75_LAMPARRAY_Y_FOR(i)      LAMP_Y_##i
#define HW75_LAMPARRAY_Z            LAMP_Z_ALL

#endif /* HW75_LAMPARRAY_CONFIG_DYNAMIC_H_ */