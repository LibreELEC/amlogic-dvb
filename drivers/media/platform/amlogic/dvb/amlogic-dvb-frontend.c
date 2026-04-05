// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB Frontend Platform Glue
 *
 * MİMARİ: avl6862_probe() frontend'i kendisi kayıt eder.
 *
 * Platform probe olunca kendini global listeye ekler.
 * avl6862_probe() → aml_dvb_find_by_fe_node() ile adapter'ı bulur
 *                 → dvb_register_frontend() çağırır.
 *
 * Modül yükleme sırası bağımsız: platform önce yüklenirse liste hazır,
 * avl6862 önce yüklenirse -EPROBE_DEFER döner ve kernel tekrar dener.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-hwops.h"   // <-- EKLENDİ: hwops yapısının tanımı için

/* ── Global DVB adapter listesi ─────────────────────────────────────── */

struct aml_dvb_entry {
	struct aml_dvb   *dvb;
	struct list_head  node;
};

#define MAX_DVB_ADAPTERS 4	/* Sistemde olabilecek max DVB platform adapter */

static LIST_HEAD(aml_dvb_list);
static DEFINE_MUTEX(aml_dvb_list_lock);

/* Platform probe tamamlanınca çağrılır */
void aml_dvb_list_add(struct aml_dvb *dvb)
{
	struct aml_dvb_entry *entry;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return;

	entry->dvb = dvb;
	mutex_lock(&aml_dvb_list_lock);
	list_add(&entry->node, &aml_dvb_list);
	mutex_unlock(&aml_dvb_list_lock);
	dev_info(dvb->dev, "DVB adapter registered in global list\n");
}
EXPORT_SYMBOL_GPL(aml_dvb_list_add);

/* Platform remove'da çağrılır */
void aml_dvb_list_del(struct aml_dvb *dvb)
{
	struct aml_dvb_entry *entry, *tmp;

	mutex_lock(&aml_dvb_list_lock);
	list_for_each_entry_safe(entry, tmp, &aml_dvb_list, node) {
		if (entry->dvb == dvb) {
			list_del(&entry->node);
			kfree(entry);
			break;
		}
	}
	mutex_unlock(&aml_dvb_list_lock);
}
EXPORT_SYMBOL_GPL(aml_dvb_list_del);

/**
 * aml_dvb_find_by_fe_node - fe_node'u dvb-frontends listesinde içeren
 *                            adapter'ı ve fe_idx'i döndürür.
 *
 * of_parse_phandle() sleeping olabileceğinden mutex dışında çağrılır.
 * Liste snapshot'ı mutex altında alınır, OF işlemleri dışında yapılır.
 */
static struct aml_dvb *aml_dvb_find_by_fe_node(struct device_node *fe_node,
						int *fe_idx_out)
{
	struct aml_dvb_entry *entry;
	struct aml_dvb *found = NULL;

	/* Snapshot: listedeki tüm dvb pointer'larını kopyala */
	struct aml_dvb *adapters[MAX_DVB_ADAPTERS];
	int nadapters = 0, i, j;

	mutex_lock(&aml_dvb_list_lock);
	list_for_each_entry(entry, &aml_dvb_list, node) {
		if (nadapters < MAX_DVB_ADAPTERS)
			adapters[nadapters++] = entry->dvb;
	}
	mutex_unlock(&aml_dvb_list_lock);

	/* OF işlemleri mutex dışında — sleeping güvenli */
	for (i = 0; i < nadapters && !found; i++) {
		struct aml_dvb *dvb = adapters[i];
		struct device_node *np = dvb->dev->of_node;
		int count = of_count_phandle_with_args(np, "dvb-frontends", NULL);

		for (j = 0; j < count && j < AML_MAX_FRONTEND; j++) {
			struct device_node *ref =
				of_parse_phandle(np, "dvb-frontends", j);
			bool match = (ref == fe_node);
			of_node_put(ref);
			if (match) {
				*fe_idx_out = j;
				found = dvb;
				break;
			}
		}
	}
	return found;
}

/* ── TS kaynak yönlendirme (artık ayrı okuma + uygulama) ───────────── */

/**
 * aml_dvb_get_ts_source - DTS'den ts-source ve ts-port değerlerini okur,
 *                          geçerli değerleri hesaplar.
 * @dvb: aml_dvb yapısı
 * @np: frontend device_node
 * @fe_idx: frontend index (0'dan başlayan)
 * @ts_demux_out: hangi demux'un kullanılacağı (çıktı)
 * @ts_port_out: hangi fiziksel TS portunun kullanılacağı (çıktı)
 *
 * Dönen değer: 0 başarı, negatif hata
 */
static int aml_dvb_get_ts_source(struct aml_dvb *dvb, struct device_node *np,
				  int fe_idx, u32 *ts_demux_out, u32 *ts_port_out)
{
	u32 ts_demux, ts_port;
	int ret;

	/* ts-source: hangi demux kullanılacak (0,1,2) */
	ret = of_property_read_u32(np, "ts-source", &ts_demux);
	if (ret) {
		ts_demux = fe_idx % dvb->caps.num_demux;
		dev_info(dvb->dev, "Frontend %d: no ts-source, defaulting to demux %d\n",
			 fe_idx, ts_demux);
	}
	if (ts_demux >= dvb->caps.num_demux) {
		dev_err(dvb->dev, "Frontend %d: invalid ts-source %d, using 0\n",
			fe_idx, ts_demux);
		ts_demux = 0;
	}

	/* ts-port: fiziksel TS input port (0=tsin_a, 1=tsin_b, ...) */
	ret = of_property_read_u32(np, "ts-port", &ts_port);
	if (ret) {
		/* Eski DTS uyumluluğu: port = demux index */
		ts_port = ts_demux;
		dev_info(dvb->dev, "Frontend %d: no ts-port, using demux index (%d)\n",
			 fe_idx, ts_port);
	}
	if (ts_port >= dvb->caps.num_demux) {
		dev_err(dvb->dev, "Frontend %d: invalid ts-port %d, using demux index (%d)\n",
			fe_idx, ts_port, ts_demux);
		ts_port = ts_demux;
	}

	*ts_demux_out = ts_demux;
	*ts_port_out = ts_port;
	return 0;
}

/**
 * aml_dvb_apply_ts_source - Verilen demux ve port değerlerini donanıma uygular.
 * @dvb: aml_dvb yapısı
 * @ts_demux: hangi demux
 * @ts_port: hangi fiziksel TS port
 */
static void aml_dvb_apply_ts_source(struct aml_dvb *dvb, u32 ts_demux, u32 ts_port)
{
	aml_dmx_set_source(&dvb->demux[ts_demux],
			   AML_TS_SRC_FRONTEND_TS0 + ts_port);
	dev_info(dvb->dev,
		 "Demux %d <- tsin_%c (AML_TS_SRC_FRONTEND_TS%d)\n",
		 ts_demux, 'a' + ts_port, ts_port);
}

/* ── avl6862_probe() tarafından çağrılır ────────────────────────────── */

/**
 * aml_dvb_register_frontend - demod probe'undan frontend kayıt
 * @fe_node: demod'un DTS node'u
 * @fe:      hazır dvb_frontend (tuner bağlı)
 *
 * Dönen değer:
 *  0          - başarılı
 * -EPROBE_DEFER - platform henüz hazır değil, kernel tekrar dener
 *  diğer      - hata
 */
int aml_dvb_register_frontend(struct device_node *fe_node,
			       struct dvb_frontend *fe,
			       struct i2c_client *client)
{
	struct aml_dvb *dvb;
	int fe_idx = -1;
	u32 ts_demux, ts_port;
	int ret;

	if (!fe_node || !fe)
		return -EINVAL;

	dvb = aml_dvb_find_by_fe_node(fe_node, &fe_idx);
	if (!dvb) {
		pr_warn("aml_dvb: no adapter found for %pOF — deferring\n",
			fe_node);
		return -EPROBE_DEFER;
	}

	/* Runtime PM: cihazı uyandır (register yazma için gerekli) */
	ret = pm_runtime_get_sync(dvb->dev);
	if (ret < 0) {
		dev_err(dvb->dev, "Failed to get runtime PM: %d\n", ret);
		pm_runtime_put_noidle(dvb->dev);
		return ret;
	}

	if (dvb->frontend[fe_idx]) {
		dev_warn(dvb->dev, "Frontend %d already registered\n", fe_idx);
		ret = 0;
		goto out_pm_put;
	}

	ret = dvb_register_frontend(&dvb->adapter, fe);
	if (ret < 0) {
		dev_err(dvb->dev, "Frontend %d: dvb_register_frontend: %d\n",
			fe_idx, ret);
		goto out_pm_put;
	}

	dvb->frontend[fe_idx] = fe;
	if (client) {
		/* I2C client referansını sakla — /proc/bus/nim_sockets için */
		get_device(&client->dev);
		dvb->demod_client[fe_idx] = client;
	}
	if (fe_idx + 1 > dvb->num_frontend)
		dvb->num_frontend = fe_idx + 1;

	/* TS source ayarı: DTS'den oku ve uygula */
	ret = aml_dvb_get_ts_source(dvb, fe_node, fe_idx, &ts_demux, &ts_port);
	if (ret) {
		dev_err(dvb->dev, "Failed to get TS source: %d\n", ret);
		goto out_unregister;
	}
	aml_dvb_apply_ts_source(dvb, ts_demux, ts_port);

	/* Async FIFO source ayarı (eğer varsa) */
	if (ts_demux < dvb->caps.num_asyncfifo) {
		ret = aml_asyncfifo_set_source(&dvb->asyncfifo[ts_demux], ts_demux);
		if (ret)
			dev_warn(dvb->dev, "Failed to set async FIFO %d source: %d\n",
				 ts_demux, ret);
		else
			dev_info(dvb->dev, "Async FIFO %d source set to demux %d\n",
				 ts_demux, ts_demux);
	} else {
		dev_dbg(dvb->dev, "No async FIFO for demux %d\n", ts_demux);
	}

	/* TS clock ayarı (54 MHz paralel mod için) */
	if (dvb->hwops && dvb->hwops->set_ts_rate) {
		dvb->hwops->set_ts_rate(dvb, 54000);
		dev_info(dvb->dev, "TS rate set to 54 Mbps\n");
	}

	dev_info(dvb->dev, "Frontend %d registered (demod-initiated)\n", fe_idx);
	ret = 0;

out_unregister:
	if (ret)
		dvb_unregister_frontend(fe);
out_pm_put:
	pm_runtime_put(dvb->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(aml_dvb_register_frontend);

/**
 * aml_dvb_unregister_frontend - avl6862_remove() tarafından çağrılır
 * @fe_node: demod'un DTS node'u
 *
 * avl6862 platform'dan önce kaldırılırsa frontend'i adapter'dan çıkarır.
 * Platform remove'da zaten kaldırıldıysa ikinci çağrı güvenle görmezden gelinir.
 */
void aml_dvb_unregister_frontend(struct device_node *fe_node)
{
	struct aml_dvb *dvb;
	int fe_idx = -1;

	dvb = aml_dvb_find_by_fe_node(fe_node, &fe_idx);
	if (!dvb || fe_idx < 0)
		return;

	if (!dvb->frontend[fe_idx])
		return;	/* zaten kaldırılmış */

	/* Runtime PM: kaldırma sırasında da uyanık olmak iyidir */
	pm_runtime_get_sync(dvb->dev);
	dvb_unregister_frontend(dvb->frontend[fe_idx]);
	dvb->frontend[fe_idx] = NULL;
	pm_runtime_put(dvb->dev);
}
EXPORT_SYMBOL_GPL(aml_dvb_unregister_frontend);

/* ── Platform probe: adapter'ı listeye ekle ─────────────────────────── */

int aml_dvb_probe_frontends(struct aml_dvb *dvb)
{
	struct device *dev = dvb->dev;
	struct device_node *np = dev->of_node;
	struct device_node *fe_node;
	int count, i;

	/* Adapter artık hazır — listeye ekle */
	aml_dvb_list_add(dvb);

	if (!np)
		return 0;

	count = of_count_phandle_with_args(np, "dvb-frontends", NULL);
	if (count <= 0)
		return 0;

	dev_info(dev, "DVB adapter ready, expecting %d frontend(s)\n", count);
	for (i = 0; i < count; i++) {
		fe_node = of_parse_phandle(np, "dvb-frontends", i);
		if (!fe_node)
			continue;
		dev_info(dev, "  [%d] %pOF\n", i, fe_node);
		of_node_put(fe_node);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(aml_dvb_probe_frontends);

/* ── Platform remove: listeden çıkar ve frontend'leri serbest bırak ── */

void aml_dvb_release_frontends(struct aml_dvb *dvb)
{
	int i;

	aml_dvb_list_del(dvb);

	for (i = 0; i < dvb->num_frontend; i++) {
		if (dvb->frontend[i]) {
			dvb_unregister_frontend(dvb->frontend[i]);
			dvb->frontend[i] = NULL;
		}
		if (dvb->demod_client[i]) {
			put_device(&dvb->demod_client[i]->dev);
			dvb->demod_client[i] = NULL;
		}
	}
	dvb->num_frontend = 0;
}
EXPORT_SYMBOL_GPL(aml_dvb_release_frontends);
