// SPDX-License-Identifier: GPL-2.0-only
#include <linux/regmap.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"

/*
 * aml_dvb_volatile_reg — hangi registerlar cache bypass olacak
 *
 * KRİTİK: FM_WR_ADDR (woff 0x07) her demux için volatile OLMAK ZORUNDA.
 *   CPU 0x8000|addr yazar → donanım BIT(15)'i temizler.
 *   REGCACHE_RBTREE varken read, cached 0x8000 değerini döndürür
 *   → fm_wait_ready() sonsuza kadar bekler (FM busy timeout).
 *
 * Aynı sorun STB_INT_STATUS (woff 0x23) için de geçerli:
 *   Donanım bitleri set eder, CPU temizler — cache stale kalır.
 *
 * Çözüm: per-demux stride (0x140) içinde bu offset'leri volatile yap.
 */
static bool aml_dvb_volatile_reg(struct device *dev, unsigned int reg)
{
	u32 offset;

	/* STB_VERSION_O(0) — read-only */
	if (reg == 0x00)
		return true;

	/* WR_PTR registerları — donanım yazar */
	if (reg >= DEMUX_WR_PTR_START && reg < DEMUX_WR_PTR_END)
		return true;

	/*
	 * Per-demux volatile registerlar (stride = 0x140 = 320 byte):
	 *   woff 0x06 = FM_WR_DATA   (0x18) — yazma sonrası oku gerekmez
	 *   woff 0x07 = FM_WR_ADDR   (0x1C) — BIT(15) hw tarafından temizlenir
	 *   woff 0x0b = OM_CMD_STATUS (0x2C) — durum register
	 *   woff 0x11 = SEC_BUFF_BUSY (0x44) — hw yazar
	 *   woff 0x12 = SEC_BUFF_READY(0x48) — hw yazar
	 *   woff 0x13 = SEC_BUFF_NUMBER(0x4C) — hw yazar
	 *   woff 0x23 = STB_INT_STATUS(0x8C) — hw yazar, cpu temizler
	 */
	offset = reg % 0x140;
	switch (offset) {
	case 0x18: /* FM_WR_DATA */
	case 0x1C: /* FM_WR_ADDR   ← EN KRİTİK */
	case 0x2C: /* OM_CMD_STATUS */
	case 0x44: /* SEC_BUFF_BUSY */
	case 0x48: /* SEC_BUFF_READY */
	case 0x4C: /* SEC_BUFF_NUMBER */
	case 0x8C: /* STB_INT_STATUS */
		return true;
	}

	return false;
}

/* Demux region regmap config */
const struct regmap_config aml_demux_regmap_config = {
	.name = "demux",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x1000,
	.writeable_reg = NULL,
	.volatile_reg = aml_dvb_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
	.fast_io = true,
};

/*
 * Async FIFO region regmap config (registers are purely volatile)
 *
 * S905X3 fiziksel layout (dev/mem doğrulandı):
 *   base = 0xFFD09000 (async_fifo2), size = 0x2000
 *   FIFO[0] offset = 0x1000 (+0x00..+0x14) → 0xFFD0A000 (async_fifo)
 *   FIFO[1] offset = 0x0000 (+0x00..+0x14) → 0xFFD09000 (async_fifo2)
 *
 * max_register: son register = FIFO[0]+0x10 = 0x1010
 */
const struct regmap_config aml_async_regmap_config = {
	.name        = "asyncfifo",
	.reg_bits    = 32,
	.val_bits    = 32,
	.reg_stride  = 4,
	.max_register = 0x1FFC,	/* 0x2000 bölge — FIFO[0] 0x1000..0x1014 dahil */
	.cache_type  = REGCACHE_NONE,
	.fast_io     = true,
};
