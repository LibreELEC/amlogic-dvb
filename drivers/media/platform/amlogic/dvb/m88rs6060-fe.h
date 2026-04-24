/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Montage M88RS6060 DVB-S/S2/S2X frontend driver
 * Glue layer for Amlogic DVB demux (amlogic-dvb taki-branch)
 *
 * The M88RS6060 is an all-in-one demodulator + tuner chip.
 * It is already present in mainline Linux as drivers/media/dvb-frontends/m88rs6060.c
 * This file provides only the platform glue: I2C probe/remove,
 * GPIO power/reset sequence, and registration with the Amlogic demux.
 *
 * Copyright (C) 2026
 */
#ifndef __M88RS6060_FE_H__
#define __M88RS6060_FE_H__

#include <linux/i2c.h>
#include <media/dvb_frontend.h>

/**
 * struct m88rs6060_fe_state - per-device private state
 * @client:        I2C client
 * @fe:            DVB frontend (allocated by m88rs6060_attach)
 * @lnb_gpio:      LNB 13V/18V select GPIO (optional)
 * @registered:    true once aml_dvb_register_frontend() succeeded
 */
struct m88rs6060_fe_state {
	struct i2c_client       *client;
	struct dvb_frontend     *fe;
	struct gpio_desc        *lnb_gpio;
	bool                     registered;
};

#endif /* __M88RS6060_FE_H__ */
