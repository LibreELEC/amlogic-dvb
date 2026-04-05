// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB — GPIO ve Pinctrl Yönetimi
 *
 * Bu modül iki seviyede GPIO yönetir:
 *
 * 1. Platform seviyesi (platform DTS node):
 *    Sürücü probe'unda tüm GPIO'lar devm ile alınır, consumer name atanır
 *    ve struct aml_dvb_gpio tablosuna eklenir.
 *    Tablo /sys/kernel/debug/<dvb>/gpio ve /sys/kernel/debug/gpio'da görünür.
 *
 * 2. Frontend seviyesi (I2C client DTS node):
 *    Her frontend probe'unda power/reset sekansı uygulanır.
 *
 * ── /sys/kernel/debug/gpio çıktısı ──────────────────────────────────────
 *  gpio-45 (demod-reset  | amlogic-dvb-demod-reset ) out lo ACTIVE LOW
 *  gpio-46 (demod-power  | amlogic-dvb-demod-power ) out hi
 *  gpio-47 (ant-power    | amlogic-dvb-ant-power   ) out lo
 *  gpio-51 (ts0-data0    | amlogic-dvb-ts0-data0   ) in   -- [PINCTRL]
 *
 * ── /sys/kernel/debug/<dvb>/gpio çıktısı ─────────────────────────────────
 *  # Amlogic DVB GPIO Tablosu
 *  # idx  name                  direction  logical  raw  polarity  consumer
 *    [0]  demod-reset           out        1        0    ACTIVE_LOW amlogic-dvb-demod-reset
 *    [1]  demod-power           out        1        1    ACTIVE_HI  amlogic-dvb-demod-power
 *    [2]  ant-power             out        0        0    ACTIVE_HI  amlogic-dvb-ant-power
 *
 * ── /sys/kernel/debug/<dvb>/pinctrl çıktısı ──────────────────────────────
 *  pinctrl device : ff634000.periphs-pinctrl
 *  current state  : default
 *  available states: default parallel s_ts0 s_ts1 p_ts0 p_ts1
 *
 *  TS input mux (STB_TOP_CONFIG):
 *    ts0: parallel (GPIOX_0..7 + CLK + SYNC)
 *    ts1: serial   (GPIOZ_2 data, GPIOZ_3 clk)
 *
 * Copyright (C) 2025 Neil Armstrong <neil.armstrong@linaro.org>
 */

#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "amlogic_dvb.h"

/* ── Timing sabitleri ─────────────────────────────────────────────────── */
#define AML_GPIO_PWR_OFF_MS_DEFAULT	200
#define AML_GPIO_PWR_ON_MS_DEFAULT	200
#define AML_GPIO_RST_ASSERT_MS_DEFAULT	20
#define AML_GPIO_RST_RELEASE_MS_DEFAULT	50

/* ── Maksimum GPIO tablo boyutu ────────────────────────────────────────── */
#define AML_MAX_GPIOS  16

/* ── Consumer name prefix ──────────────────────────────────────────────── */
#define AML_GPIO_PREFIX "amlogic-dvb-"

/* ======================================================================
 * Platform GPIO tanım tablosu
 *
 * Her giriş: DTS property adı, GPIOD yön/başlangıç değeri, insan okunabilir
 * açıklama ve consumer etiket soneki.
 * ====================================================================== */
static const struct {
	const char *prop;		/* DTS property kökü (xxx-gpios) */
	enum gpiod_flags flags;		/* başlangıç yönü + değeri */
	const char *label;		/* kısa insan okunabilir isim */
	const char *consumer_suffix;	/* /sys/kernel/debug/gpio consumer adı */
	bool optional;			/* yoksa hata verme */
} aml_platform_gpios[] = {
	/*
	 * Anten LNA güç anahtarı
	 * DTS: ant-power-gpios = <&gpio GPIOH_6 GPIO_ACTIVE_HIGH>;
	 */
	{ "ant-power",    GPIOD_OUT_LOW,  "ant-power",   "ant-power",   true  },

	/*
	 * Demodülatör güç (LDO enable)
	 * DTS: demod-power-gpios = <&gpio GPIOH_3 GPIO_ACTIVE_HIGH>;
	 */
	{ "demod-power",  GPIOD_OUT_LOW,  "demod-power", "demod-power", true  },

	/*
	 * Demodülatör donanım reset
	 * DTS: demod-reset-gpios = <&gpio GPIOH_4 GPIO_ACTIVE_LOW>;
	 * ACTIVE_LOW: logic-1 = reset çıkılmış (serbest), logic-0 = reset'te
	 */
	{ "demod-reset",  GPIOD_OUT_HIGH, "demod-reset", "demod-reset", true  },

	/*
	 * Tuner güç
	 * DTS: tuner-power-gpios = <&gpio GPIOZ_6 GPIO_ACTIVE_HIGH>;
	 */
	{ "tuner-power",  GPIOD_OUT_LOW,  "tuner-power", "tuner-power", true  },

	/*
	 * Tuner reset
	 * DTS: tuner-reset-gpios = <&gpio GPIOZ_7 GPIO_ACTIVE_LOW>;
	 */
	{ "tuner-reset",  GPIOD_OUT_HIGH, "tuner-reset", "tuner-reset", true  },

	/*
	 * TS çıkış enable (bazı boardlarda mux/buffer enable)
	 * DTS: ts-out-en-gpios = <&gpio GPIOH_7 GPIO_ACTIVE_HIGH>;
	 */
	{ "ts-out-en",    GPIOD_OUT_LOW,  "ts-out-en",   "ts-out-en",   true  },

	/*
	 * LNB güç enable (DiSEqC uygulamalarında)
	 * DTS: lnb-power-gpios = <&gpio GPIOH_8 GPIO_ACTIVE_HIGH>;
	 */
	{ "lnb-power",    GPIOD_OUT_LOW,  "lnb-power",   "lnb-power",   true  },

	/*
	 * LNB 13V/18V seçim pini
	 * DTS: lnb-voltage-gpios = <&gpio GPIOH_9 GPIO_ACTIVE_HIGH>;
	 */
	{ "lnb-voltage",  GPIOD_OUT_LOW,  "lnb-voltage", "lnb-voltage", true  },
};

/* ======================================================================
 * aml_gpio_register_one() — Tek GPIO'yu kaydet ve tabloya ekle
 *
 * 1. devm_gpiod_get_optional() ile GPIO'yu al
 * 2. gpiod_set_consumer_name() ile /sys/kernel/debug/gpio consumer ata
 * 3. aml_dvb_gpio tablosuna ekle (debugfs için)
 * ====================================================================== */
static int aml_gpio_register_one(struct aml_dvb *dvb,
				 const char *prop,
				 enum gpiod_flags flags,
				 const char *label,
				 const char *consumer_suffix,
				 bool optional)
{
	struct device *dev = dvb->dev;
	struct gpio_desc *gd;
	char consumer[64];
	int idx;

	gd = optional
		? devm_gpiod_get_optional(dev, prop, flags)
		: devm_gpiod_get(dev, prop, flags);

	if (IS_ERR(gd)) {
		if (PTR_ERR(gd) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		if (!optional)
			return dev_err_probe(dev, PTR_ERR(gd),
					     "GPIO '%s' alınamadı\n", prop);
		/* optional ve hata: uyar, devam et */
		dev_dbg(dev, "GPIO '%s' bulunamadı (optional, atlandı)\n", prop);
		return 0;
	}

	if (!gd)
		return 0; /* optional, DTS'te yok */

	/*
	 * Consumer name: /sys/kernel/debug/gpio'da görünen "consumer" sütunu.
	 * Format: "amlogic-dvb-<suffix>"
	 * Örnek: "amlogic-dvb-demod-reset"
	 *
	 * kernel/gpio.c debugfs formatı:
	 *   gpio-N (PIN-NAME | CONSUMER-NAME) DIR VALUE [FLAG]
	 */
	snprintf(consumer, sizeof(consumer), "%s%s", AML_GPIO_PREFIX,
		 consumer_suffix);
	gpiod_set_consumer_name(gd, consumer);

	/* Tabloya ekle */
	idx = dvb->gpio_count;
	if (idx < AML_MAX_GPIOS) {
		dvb->gpios[idx].gd       = gd;
		dvb->gpios[idx].label    = label;
		dvb->gpios[idx].consumer = consumer_suffix;
		dvb->gpio_count++;
	}

	dev_info(dev, "GPIO %-16s → %-35s dir=%s val=%d\n",
		 label, consumer,
		 (flags & GPIOD_OUT_LOW || flags & GPIOD_OUT_HIGH) ? "out" : "in",
		 gpiod_get_value_cansleep(gd));

	return 0;
}

/* ======================================================================
 * aml_gpio_init() — Tüm platform GPIO'larını probe'da kaydet
 *
 * Probe sıralaması:
 *   1. Platform GPIO'ları kaydet (bu fonksiyon)
 *   2. Pinctrl state uygula (aml_pinctrl_init)
 *   3. Frontend probe (aml_dvb_probe_frontends)
 *
 * Bu sıra kritiktir: pinctrl TS pin'lerini configure eder, GPIO'lar
 * demod/tuner güç kontrolü yapar, frontend probe her ikisine ihtiyaç duyar.
 * ====================================================================== */
int aml_gpio_init(struct aml_dvb *dvb)
{
	int i, ret;

	dvb->gpio_count = 0;
	memset(dvb->gpios, 0, sizeof(dvb->gpios));

	dev_dbg(dvb->dev, "GPIO: platform GPIO'ları kaydediliyor...\n");

	for (i = 0; i < ARRAY_SIZE(aml_platform_gpios); i++) {
		ret = aml_gpio_register_one(dvb,
					    aml_platform_gpios[i].prop,
					    aml_platform_gpios[i].flags,
					    aml_platform_gpios[i].label,
					    aml_platform_gpios[i].consumer_suffix,
					    aml_platform_gpios[i].optional);
		if (ret)
			return ret;
	}

	/* Geriye uyumluluk: eski struct alanları doldur */
	for (i = 0; i < dvb->gpio_count; i++) {
		if (!strcmp(dvb->gpios[i].label, "demod-power"))
			dvb->tuner_power_gpio = dvb->gpios[i].gd;
		if (!strcmp(dvb->gpios[i].label, "demod-reset"))
			dvb->tuner_reset_gpio = dvb->gpios[i].gd;
	}

	dev_info(dvb->dev,
		 "GPIO: %d GPIO kaydedildi, /sys/kernel/debug/gpio'da görünür\n",
		 dvb->gpio_count);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_gpio_init);

/* ======================================================================
 * aml_gpio_release() — Pinctrl serbest bırak (devm GPIO otomatik)
 * ====================================================================== */
void aml_gpio_release(struct aml_dvb *dvb)
{
	if (!IS_ERR_OR_NULL(dvb->pinctrl)) {
		devm_pinctrl_put(dvb->pinctrl);
		dvb->pinctrl = NULL;
	}
}
EXPORT_SYMBOL_GPL(aml_gpio_release);

/* ======================================================================
 * aml_pinctrl_init() — TS pin multiplexer durumunu yapılandır
 *
 * Pinctrl state sırası (ilk bulunan kullanılır):
 *   1. "default"  — genel amaçlı, DTS'de pinctrl-0 ile tanımlanır
 *   2. "parallel" — paralel TS modu (8-bit data + CLK + SYNC)
 *
 * /sys/kernel/debug/pinctrl/<device>/pingroups dosyasında hangi pinlerin
 * hangi gruba atandığı görülür. pinctrl_select_state() çağrısından sonra
 * /sys/kernel/debug/gpio'da "PINCTRL" etiketiyle işaretlenir.
 *
 * DTS örneği:
 *   pinctrl-names = "default";
 *   pinctrl-0     = <&ts_in_a_pins &ts_in_b_pins>;
 * ====================================================================== */
/* aml_pinctrl_init/release: amlogic-dvb-core.c'de tanımlı */

/* ======================================================================
 * aml_demod_power_reset() — Frontend I2C düzeyinde güç+reset sekansı
 *
 * Frontend probe sırasında çağrılır. GPIO'lar I2C client DTS node'undan
 * alınır. Consumer name "amlogic-dvb-feN-{power,reset}" formatında atanır
 * → /sys/kernel/debug/gpio'da görünür.
 *
 * DTS örneği (demod I2C node'u):
 *   power-gpios          = <&gpio GPIOH_3 GPIO_ACTIVE_HIGH>;
 *   reset-gpios          = <&gpio GPIOH_4 GPIO_ACTIVE_LOW>;
 *   tuner-pwr-gpios      = <&gpio GPIOZ_6 GPIO_ACTIVE_HIGH>;
 *   power-off-delay-ms   = <200>;
 *   power-on-delay-ms    = <200>;
 *   reset-assert-ms      = <20>;
 *   reset-release-ms     = <50>;
 * ====================================================================== */
int aml_demod_power_reset(struct device *dev)
{
	struct gpio_desc *pwr, *rst, *tpwr;
	u32 pwr_off_ms  = AML_GPIO_PWR_OFF_MS_DEFAULT;
	u32 pwr_on_ms   = AML_GPIO_PWR_ON_MS_DEFAULT;
	u32 rst_assert  = AML_GPIO_RST_ASSERT_MS_DEFAULT;
	u32 rst_release = AML_GPIO_RST_RELEASE_MS_DEFAULT;
	char consumer[64];
	static atomic_t fe_idx = ATOMIC_INIT(0);
	int idx = atomic_fetch_inc(&fe_idx);

	of_property_read_u32(dev->of_node, "power-off-delay-ms",  &pwr_off_ms);
	of_property_read_u32(dev->of_node, "power-on-delay-ms",   &pwr_on_ms);
	of_property_read_u32(dev->of_node, "reset-assert-ms",     &rst_assert);
	of_property_read_u32(dev->of_node, "reset-release-ms",    &rst_release);

	/* ── power GPIO ── */
	pwr = devm_gpiod_get_optional(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(pwr))
		return dev_err_probe(dev, PTR_ERR(pwr),
				     "fe%d: power GPIO alınamadı\n", idx);
	if (pwr) {
		snprintf(consumer, sizeof(consumer),
			 AML_GPIO_PREFIX "fe%d-power", idx);
		gpiod_set_consumer_name(pwr, consumer);
		dev_info(dev, "GPIO: %s → /sys/kernel/debug/gpio\n", consumer);
	}

	/* ── tuner-pwr GPIO ── */
	tpwr = devm_gpiod_get_optional(dev, "tuner-pwr", GPIOD_OUT_LOW);
	if (!IS_ERR_OR_NULL(tpwr)) {
		snprintf(consumer, sizeof(consumer),
			 AML_GPIO_PREFIX "fe%d-tuner-pwr", idx);
		gpiod_set_consumer_name(tpwr, consumer);
		dev_info(dev, "GPIO: %s → /sys/kernel/debug/gpio\n", consumer);
	}

	/* ── reset GPIO ── */
	rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "fe%d: reset GPIO alınamadı\n", idx);
	if (rst) {
		snprintf(consumer, sizeof(consumer),
			 AML_GPIO_PREFIX "fe%d-reset", idx);
		gpiod_set_consumer_name(rst, consumer);
		dev_info(dev, "GPIO: %s → /sys/kernel/debug/gpio\n", consumer);
	}

	if (!pwr && !rst && !tpwr) {
		dev_dbg(dev, "fe%d: GPIO tanımlanmamış, sekans atlandı\n", idx);
		return 0;
	}

	/*
	 * Güç ve reset sekansı:
	 *   1. Güç kapat → bekle (kapasitörler boşalsın)
	 *   2. Tuner güç aç (varsa)
	 *   3. Demod güç aç → bekle (regülatör stabilize)
	 *   4. Reset assert → bekle
	 *   5. Reset release → bekle (IC boot)
	 */
	if (pwr) {
		dev_dbg(dev, "[GPIO-fe%d] power OFF (wait %u ms)\n", idx, pwr_off_ms);
		gpiod_set_value_cansleep(pwr, 0);
		msleep(pwr_off_ms);
	}
	if (tpwr)
		gpiod_set_value_cansleep(tpwr, 1);
	if (pwr) {
		dev_dbg(dev, "[GPIO-fe%d] power ON (wait %u ms)\n", idx, pwr_on_ms);
		gpiod_set_value_cansleep(pwr, 1);
		msleep(pwr_on_ms);
	}
	if (rst) {
		dev_dbg(dev, "[GPIO-fe%d] reset assert (%u ms)\n", idx, rst_assert);
		gpiod_set_value_cansleep(rst, 1);
		msleep(rst_assert);
		dev_dbg(dev, "[GPIO-fe%d] reset release (boot wait %u ms)\n", idx, rst_release);
		gpiod_set_value_cansleep(rst, 0);
		msleep(rst_release);
	}

	dev_info(dev,
		 "fe%d: GPIO sekans tamam (pwr_off=%u pwr_on=%u rst=%u+%u ms)\n",
		 idx, pwr_off_ms, pwr_on_ms, rst_assert, rst_release);
	return 0;
}
EXPORT_SYMBOL_GPL(aml_demod_power_reset);

/* ======================================================================
 * debugfs: /sys/kernel/debug/<dvb>/gpio
 *
 * Tüm platform GPIO'larının anlık durumunu gösterir.
 * /sys/kernel/debug/gpio ile tamamlayıcı — orada pin fiziksel konumu,
 * burada sürücü perspektifinden logical durum.
 *
 * Örnek çıktı:
 *   # Amlogic DVB GPIO Tablosu
 *   # Detaylı fiziksel bilgi: /sys/kernel/debug/gpio
 *   # Consumer adları:        /sys/kernel/debug/gpio (| consumer sütunu)
 *   # Pinctrl durumu:         /sys/kernel/debug/pinctrl/<dev>/pingroups
 *   #
 *   # idx  name             dir  logical  raw  polarity       consumer
 *     [0]  demod-reset      out       1    0   ACTIVE_LOW     amlogic-dvb-demod-reset
 *     [1]  demod-power      out       1    1   ACTIVE_HIGH    amlogic-dvb-demod-power
 *     [2]  ant-power        out       0    0   ACTIVE_HIGH    amlogic-dvb-ant-power
 *
 *   # Pinctrl
 *     state: default
 *     hint:  cat /sys/kernel/debug/pinctrl/<dev>/pingroups
 * ====================================================================== */
static int aml_gpio_debugfs_show(struct seq_file *s, void *v)
{
	struct aml_dvb *dvb = s->private;
	int i;

	seq_puts(s,
		 "# Amlogic DVB GPIO Tablosu\n"
		 "# /sys/kernel/debug/gpio         — tam GPIO listesi (consumer sütunu)\n"
		 "# /sys/kernel/debug/pinctrl/...  — pinctrl mux detayları\n"
		 "#\n");

	if (dvb->gpio_count == 0) {
		seq_puts(s, "# GPIO tanımlanmamış (DTS'de gpios property eksik)\n");
		goto pinctrl_section;
	}

	seq_printf(s, "# %-4s  %-16s  %-4s  %-7s  %-4s  %-14s  %s\n",
		   "idx", "name", "dir", "logical", "raw", "polarity",
		   "consumer (/sys/kernel/debug/gpio)");
	seq_puts(s,
		 "#----  ----------------  ----  -------  ----"
		 "  --------------  ---------------------------------\n");

	for (i = 0; i < dvb->gpio_count; i++) {
		struct gpio_desc *gd = dvb->gpios[i].gd;
		int logical, raw;
		bool active_low;
		const char *dir;

		if (!gd)
			continue;

		logical    = gpiod_get_value_cansleep(gd);
		raw        = gpiod_get_raw_value_cansleep(gd);
		active_low = (logical != raw);
		dir        = gpiod_get_direction(gd) ? "in" : "out";

		seq_printf(s, "  [%d]  %-16s  %-4s  %-7d  %-4d  %-14s  %s%s\n",
			   i,
			   dvb->gpios[i].label,
			   dir,
			   logical,
			   raw,
			   active_low ? "ACTIVE_LOW" : "ACTIVE_HIGH",
			   AML_GPIO_PREFIX,
			   dvb->gpios[i].consumer);
	}

pinctrl_section:
	seq_puts(s, "\n# Pinctrl\n");

	if (!IS_ERR_OR_NULL(dvb->pinctrl)) {
		const char *state_name = dvb->pins_default ?
			"default (aktif)" : "tanımlanmamış";
		struct device *pindev = &dvb->pdev->dev;
		(void)pindev;

		seq_printf(s, "  state    : %s\n", state_name);
		seq_puts(s,
			 "  detay    : cat /sys/kernel/debug/pinctrl/"
			 "<periphs-pinctrl>/pingroups\n"
			 "  ts-pins  : cat /sys/kernel/debug/pinctrl/"
			 "<periphs-pinctrl>/pins\n");
	} else {
		seq_puts(s, "  pinctrl  : yapılandırılmamış\n");
	}

	seq_puts(s,
		 "\n# Nasıl okunur?\n"
		 "#   logical=1 + raw=0 + ACTIVE_LOW  → pin fiziksel LOW, logic HIGH (aktif)\n"
		 "#   logical=1 + raw=1 + ACTIVE_HIGH → pin fiziksel HIGH, logic HIGH (aktif)\n"
		 "#   direction=out → sürücü kontrol ediyor\n"
		 "#   direction=in  → pin state okunuyor (input)\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_gpio_debugfs);

/* ======================================================================
 * debugfs: /sys/kernel/debug/<dvb>/pinctrl_state
 *
 * Pinctrl'ün mevcut ve kullanılabilir state'lerini listeler.
 * TS pin grubu adlarını kernel pinctrl altyapısından okur.
 * ====================================================================== */
static int aml_pinctrl_debugfs_show(struct seq_file *s, void *v)
{
	struct aml_dvb *dvb = s->private;

	seq_puts(s, "# Amlogic DVB Pinctrl Durumu\n#\n");

	if (IS_ERR_OR_NULL(dvb->pinctrl)) {
		seq_puts(s, "pinctrl yapılandırılmamış\n\n");
		seq_puts(s,
			 "DTS'e ekle:\n"
			 "  pinctrl-names = \"default\";\n"
			 "  pinctrl-0     = <&ts_in_a_pins>;\n");
		return 0;
	}

	seq_printf(s, "pinctrl device  : %s\n",
		   dev_name(dvb->dev));
	seq_printf(s, "current state   : %s\n",
		   dvb->pins_default ? "default" : "(none)");

	seq_puts(s,
		 "\n# TS pin mux referansı (SM1/S905X3 örneği):\n"
		 "#\n"
		 "#  Paralel mod (DTS: ts_in_a_pins):\n"
		 "#    GPIOX_0..7 → ts0_d0..d7\n"
		 "#    GPIOX_8    → ts0_clk\n"
		 "#    GPIOX_9    → ts0_sync\n"
		 "#    GPIOX_10   → ts0_valid (opsiyonel)\n"
		 "#\n"
		 "#  Seri mod S_TS0 (DTS: s_ts0_pins):\n"
		 "#    GPIOZ_2    → ts0_seri_data\n"
		 "#    GPIOZ_3    → ts0_seri_clk\n"
		 "#\n"
		 "#  Pinctrl pin gruplarını listele:\n"
		 "#    cat /sys/kernel/debug/pinctrl/<periphs>/pingroups\n"
		 "#\n"
		 "#  Hangi pin hangi grupta:\n"
		 "#    cat /sys/kernel/debug/pinctrl/<periphs>/pins\n");

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aml_pinctrl_debugfs);

/* ======================================================================
 * aml_gpio_debugfs_init() — debugfs dosyalarını oluştur
 * ====================================================================== */
void aml_gpio_debugfs_init(struct aml_dvb *dvb, struct dentry *root)
{
	if (!root)
		return;

	/* /sys/kernel/debug/<dvb>/gpio — tüm platform GPIO tablosu */
	debugfs_create_file("gpio", 0444, root, dvb,
			    &aml_gpio_debugfs_fops);

	/* /sys/kernel/debug/<dvb>/pinctrl_state — pinctrl durumu */
	debugfs_create_file("pinctrl_state", 0444, root, dvb,
			    &aml_pinctrl_debugfs_fops);
}
EXPORT_SYMBOL_GPL(aml_gpio_debugfs_init);
