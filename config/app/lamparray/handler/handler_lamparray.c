/*
 * Copyright (c) 2022-2023 XiNGRZ
 * SPDX-License-Identifier: MIT
 */

#include "handler.h"
#include "usb_comm.pb.h"

#include "lamparray.h"

static bool handle_lamp_get_mode(const usb_comm_MessageH2D *h2d, usb_comm_MessageD2H *d2h,
				 const void *bytes, uint32_t bytes_len)
{
	d2h->payload.lamp_mode.mode = lamparray_is_slave() ? 1 : 0;
	return true;
}

USB_COMM_HANDLER_DEFINE(usb_comm_Action_LAMP_GET_MODE, usb_comm_MessageD2H_lamp_mode_tag,
			handle_lamp_get_mode);

static bool handle_lamp_set_mode(const usb_comm_MessageH2D *h2d, usb_comm_MessageD2H *d2h,
				 const void *bytes, uint32_t bytes_len)
{
	const usb_comm_LampMode *req = &h2d->payload.lamp_mode;

	if (req->mode) {
		lamparray_enter_slave();
	} else {
		lamparray_enter_autonomous();
	}

	return handle_lamp_get_mode(h2d, d2h, NULL, 0);
}

USB_COMM_HANDLER_DEFINE(usb_comm_Action_LAMP_SET_MODE, usb_comm_MessageD2H_lamp_mode_tag,
			handle_lamp_set_mode);