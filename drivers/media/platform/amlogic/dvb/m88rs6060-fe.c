// SPDX-License-Identifier: GPL-2.0-only
/*
 * Montage M88RS6060 DVB-S/S2/S2X — Amlogic platform glue driver
 *
 * This driver is the glue between the mainline M88RS6060 demodulator driver
 * (drivers/media/dvb-frontends/m88rs6060.c) and the Amlogic DVB demux
 * (drivers/media/platform/amlogic/dvb/, taki-branch V2.68+).
 *
 * Architecture:
 *
 *   [DTS: montage,m88rs6060-fe @ i2c1/0x69]
 *          |
 *          v
 *   m88rs6060_fe_probe()
 *     1. aml_demod_power_reset()   -- GPIO power/reset sequence
 *     2. m88rs6060_attach()        -- mainline demod init (I2C probe)
 *     3. set_voltage hook install  -- LNB 13V/18V via GPIO
 *     4. aml_dvb_register_frontend(node, fe, client)
 *           |
 *           v
 *        amlogic-dvb-frontend.c:
 *          dvb_register_frontend()
 *          aml_dmx_set_source()       (DMX0 ← tsin_a)
 *          aml_asyncfifo_set_source() (FIFO0 ← DMX0)
 *
 * Hardware (Mecool M8S Plus DVB-S2X, confirmed on live device):
 *   I2C:   bus 1 (i2c_B, 0xc11087c0), addr 0x69
 *   TS:    parallel, GPIODV_0..10 (tsin_a)
 *   Reset: GPIODV_13 (gpio-217), active-low
 *   Power: GPIODV_14 (gpio-218), active-high
 *   LNB:   GPIODV_15 (gpio-219), low=13V, high=18V
 *
 * Note: The mainline m88rs6060 driver was originally added for the
 * TBS 5930 USB stick. Using it for the internal Amlogic TS path
 * requires only this glue — no modifications to m88rs6060.c itself.
 *
 * Copyright (C) 2026
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <media/dvb_frontend.h>

/*
 * m88rs6060_attach() is in drivers/media/dvb-frontends/m88rs6060.c
 * (McMCC driver — m88rs6060_config defined in m88rs6060.h)
 */
#include "m88rs6060.h"

/*
 * amlogic-dvb platform API:
 *   aml_demod_power_reset() — GPIO power/reset sequence
 *   aml_dvb_register_frontend() — register with Amlogic demux adapter
 */
#include "amlogic_dvb.h"

#include "m88rs6060-fe.h"

/* ======================================================================
 * LNB voltage control via GPIO
 *
 * M88RS6060 has an internal LNB controller (EN5359), but on the
 * Mecool M8S Plus DVB-S2X the LNB voltage select is done via an
 * external GPIO (GPIODV_15):
 *   low  = 13V  (vertical polarisation / right-circular)
 *   high = 18V  (horizontal polarisation / left-circular)
 *
 * The 22kHz tone is handled internally by the M88RS6060.
 * DiSEqC commands go through the demod chip (already in mainline driver).
 * ====================================================================== */

/* Forward declaration — needed for cfg.set_voltage callback */
static struct m88rs6060_fe_state *g_fe_state; /* single-instance glue */

static int m88rs6060_fe_set_voltage(struct dvb_frontend *fe,
				    enum fe_sec_voltage voltage)
{
	struct m88rs6060_fe_state *state = g_fe_state;

	if (!state || !state->lnb_gpio)
		return 0; /* no external GPIO — let demod handle it */

	switch (voltage) {
	case SEC_VOLTAGE_13:
		gpiod_set_value_cansleep(state->lnb_gpio, 0);
		dev_dbg(&state->client->dev, "LNB: 13V\n");
		break;
	case SEC_VOLTAGE_18:
		gpiod_set_value_cansleep(state->lnb_gpio, 1);
		dev_dbg(&state->client->dev, "LNB: 18V\n");
		break;
	case SEC_VOLTAGE_OFF:
		gpiod_set_value_cansleep(state->lnb_gpio, 0);
		dev_dbg(&state->client->dev, "LNB: OFF\n");
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* ======================================================================
 * I2C probe / remove
 * ====================================================================== */

static int m88rs6060_fe_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct m88rs6060_fe_state *state;
	struct m88rs6060_config cfg = {};
	struct dvb_frontend *fe;
	int ret;

	dev_info(dev, "m88rs6060-fe: probe start (addr=0x%02x bus=%d)\n",
		 client->addr, client->adapter->nr);

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->client = client;
	i2c_set_clientdata(client, state);

	/* ── Step 1: Power and reset sequence ───────────────────────────
	 * aml_demod_power_reset() reads from this I2C node's DTS:
	 *   power-gpios, reset-gpios
	 *   power-off-delay-ms, power-on-delay-ms
	 *   reset-assert-ms, reset-release-ms
	 * ──────────────────────────────────────────────────────────────── */
	ret = aml_demod_power_reset(dev);
	if (ret) {
		dev_err(dev, "m88rs6060-fe: power/reset sequence failed: %d\n",
			ret);
		return ret;
	}

	/* ── Step 2: Optional LNB voltage GPIO ──────────────────────────
	 * GPIODV_15 on M8S Plus: low=13V, high=18V.
	 * Not fatal if absent (some boards use internal LNB controller).
	 * ──────────────────────────────────────────────────────────────── */
	state->lnb_gpio = devm_gpiod_get_optional(dev, "lnb-voltage",
						  GPIOD_OUT_LOW);
	if (IS_ERR(state->lnb_gpio)) {
		if (PTR_ERR(state->lnb_gpio) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "m88rs6060-fe: lnb-voltage GPIO unavailable: %ld\n",
			 PTR_ERR(state->lnb_gpio));
		state->lnb_gpio = NULL;
	}
	if (state->lnb_gpio) {
		gpiod_set_consumer_name(state->lnb_gpio,
					"amlogic-dvb-lnb-voltage");
		dev_info(dev, "m88rs6060-fe: LNB voltage GPIO acquired\n");
	}

	/* ── Step 3: M88RS6060 demodulator init ─────────────────────────
	 *
	 * McMCC m88rs6060_config fields (from m88rs6060.h):
	 *   demod_address  — I2C address of the demod chip
	 *   ts_mode        — 0: Parallel (8-bit, tsin_a), 1: Serial
	 *   pin_ctrl       — 0: LNB voltage via cfg.set_voltage callback
	 *   ci_mode        — 0: no CI
	 *   tuner_readstops — 0: default
	 *   set_voltage    — callback for LNB 13V/18V (our GPIO function)
	 *
	 * ts_mode=0 (Parallel) is required for tsin_a on GPIODV_0..10.
	 * ──────────────────────────────────────────────────────────────── */
	cfg.demod_address  = client->addr;
	cfg.ts_mode        = 0; /* 0 = Parallel TS */
	cfg.pin_ctrl       = 0; /* LNB via set_voltage callback, not chip pin */
	cfg.ci_mode        = 0; /* no CI */
	cfg.tuner_readstops = 0;
	memset(cfg.name_ext_fw, 0, sizeof(cfg.name_ext_fw));

	/*
	 * LNB voltage: McMCC driver calls cfg.set_voltage() directly.
	 * We register our GPIO function here; g_fe_state lets the callback
	 * reach our state (single-instance driver — one chip per board).
	 */
	g_fe_state = state;
	if (state->lnb_gpio)
		cfg.set_voltage = m88rs6060_fe_set_voltage;

	fe = m88rs6060_attach(&cfg, client->adapter);
	if (!fe) {
		dev_err(dev, "m88rs6060-fe: m88rs6060_attach() failed\n");
		g_fe_state = NULL;
		return -ENODEV;
	}

	state->fe = fe;

	/* ── Step 4: LNB voltage hook already installed via cfg.set_voltage */
	dev_info(dev, "m88rs6060-fe: attached — %s\n", fe->ops.info.name);

	/* ── Step 5: Register with Amlogic DVB demux adapter ────────────
	 *
	 * aml_dvb_register_frontend() in amlogic-dvb-frontend.c:
	 *   1. Finds the dvb_adapter by matching dev->of_node against
	 *      dvb-frontends phandle list in platform DTS node.
	 *   2. Calls dvb_register_frontend()
	 *   3. Reads ts-source / ts-port from our DTS node
	 *   4. Calls aml_dmx_set_source(dmx, AML_TS_SRC_FRONTEND_TS0)
	 *   5. Calls aml_asyncfifo_set_source(afifo, demux_id)
	 *
	 * Returns -EPROBE_DEFER if platform device is not yet ready.
	 * The kernel will retry this probe automatically.
	 * ──────────────────────────────────────────────────────────────── */
	ret = aml_dvb_register_frontend(dev->of_node, fe, client);
	if (ret == -EPROBE_DEFER) {
		dev_info(dev, "m88rs6060-fe: platform not ready, deferring\n");
		/* dvb_frontend_detach releases the fe allocated by attach */
		dvb_frontend_detach(fe);
		return -EPROBE_DEFER;
	}
	if (ret < 0) {
		dev_err(dev, "m88rs6060-fe: aml_dvb_register_frontend: %d\n",
			ret);
		dvb_frontend_detach(fe);
		return ret;
	}

	state->registered = true;
	dev_info(dev, "m88rs6060-fe: registered successfully\n");
	return 0;
}

static void m88rs6060_fe_remove(struct i2c_client *client)
{
	struct m88rs6060_fe_state *state = i2c_get_clientdata(client);

	dev_info(&client->dev, "m88rs6060-fe: remove\n");

	if (!state)
		return;

	if (state->registered) {
		aml_dvb_unregister_frontend(client->dev.of_node);
		state->registered = false;
	}

	g_fe_state = NULL;

	if (state->fe) {
		dvb_frontend_detach(state->fe);
		state->fe = NULL;
	}

	/* devm resources (GPIO, state struct) freed automatically */
}

/* ======================================================================
 * Module boilerplate
 * ====================================================================== */

static const struct of_device_id m88rs6060_fe_of_match[] = {
	{ .compatible = "montage,m88rs6060-fe" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, m88rs6060_fe_of_match);

static const struct i2c_device_id m88rs6060_fe_id[] = {
	{ "m88rs6060-fe", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, m88rs6060_fe_id);

static struct i2c_driver m88rs6060_fe_driver = {
	.driver = {
		.name           = "m88rs6060-fe",
		.of_match_table = m88rs6060_fe_of_match,
	},
	.probe    = m88rs6060_fe_probe,
	.remove   = m88rs6060_fe_remove,
	.id_table = m88rs6060_fe_id,
};

module_i2c_driver(m88rs6060_fe_driver);

MODULE_AUTHOR("port for amlogic-dvb taki-branch");
MODULE_DESCRIPTION("Montage M88RS6060 frontend glue for Amlogic DVB");
MODULE_LICENSE("GPL v2");
