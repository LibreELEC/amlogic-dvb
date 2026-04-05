// SPDX-License-Identifier: GPL-2.0-only
/*
 * Amlogic DVB Async FIFO (DVR) – workqueue‑based poll engine, lockless ring
 *
 * Copyright (C) 2025 Neil Armstrong <neil.armstrong@linaro.org>
 */

#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <media/dvb_demux.h>
#include "amlogic_dvb.h"
#include "amlogic-dvb-regs.h"
#include "amlogic-dvb-trace.h"

#define RING_ENTRIES(afifo)    ((afifo)->buf_size / (afifo)->flush_size)
#define RING_MASK(afifo)        (RING_ENTRIES(afifo) - 1)

static inline u32 ring_avail(struct aml_asyncfifo *afifo)
{
    u32 head = smp_load_acquire(&afifo->head);
    u32 tail = READ_ONCE(afifo->tail);
    return (head - tail) & RING_MASK(afifo);
}

static inline void ring_consume(struct aml_asyncfifo *afifo, u32 chunks)
{
    u32 new_tail = (READ_ONCE(afifo->tail) + chunks) & RING_MASK(afifo);
    /* smp_store_release: tail güncellemesi DMA okumalarından sonra görünür */
    smp_store_release(&afifo->tail, new_tail);
}

static void aml_asyncfifo_update_irq_threshold(struct aml_asyncfifo *afifo)
{
	struct aml_dvb *dvb = afifo->dvb;
	u32 fill = ring_avail(afifo);
	u32 new_thresh;

	fill = fill * 100 / afifo->ring_entries;
	if (fill > 75)
		new_thresh = afifo->irq_threshold_max;
	else if (fill > 50)
		new_thresh = afifo->irq_threshold_min * 2;
	else if (fill > 25)
		new_thresh = afifo->irq_threshold_min;
	else
		new_thresh = afifo->irq_threshold_min / 2;

	new_thresh = clamp(new_thresh, afifo->irq_threshold_min,
			   afifo->irq_threshold_max);

	if (new_thresh != afifo->irq_threshold) {
		u32 thresh_val;

		afifo->irq_threshold = new_thresh;

		/*
		 * IRQ threshold REG3 (+0x0C) bit[15:0] alanında.
		 * 128-byte blok sayısı olarak verilir.
		 */
		thresh_val = min_t(u32, new_thresh >> 7, 0xFFFF);
		thresh_val |= AFIFO_IRQ_EN;   /* BIT21: IRQ arm korunmalı */
		aml_write_async(dvb, ASYNC_FIFO_REG3(afifo->id), thresh_val);

		dev_dbg(dvb->dev, "afifo%d: irq_threshold=%u thresh_val=0x%x\n",
			afifo->id, new_thresh, thresh_val);
	}
}

static void aml_asyncfifo_poll_work(struct work_struct *work)
{
    struct aml_asyncfifo *afifo = container_of(work, struct aml_asyncfifo,
                           poll_work);
    struct aml_dvb *dvb = afifo->dvb;
    struct aml_dmx *dmx;
    u32 budget = afifo->poll_budget;
    int work_done = 0;
    u32 avail, tail;
    void *buf;
    size_t len;

    if (unlikely(!afifo->enabled))
        goto out;

    dev_dbg(dvb->dev, "afifo%d: poll_work starting head=%u tail=%u avail=%u\n",
             afifo->id, READ_ONCE(afifo->head), afifo->tail, ring_avail(afifo));

    while (budget--) {
        avail = ring_avail(afifo);
        if (avail == 0)
            break;

        tail = READ_ONCE(afifo->tail);
        buf = afifo->buf_virt + tail * afifo->flush_size;
        len = afifo->flush_size;

        prefetch(buf);
        prefetch(buf + 64);
        prefetch(buf + 128);
        prefetch(buf + 192);

        dma_rmb();

        if (unlikely(afifo->source >= dvb->caps.num_demux)) {
            dev_err(dvb->dev, "afifo%d: invalid source %d, dropping chunk\n", afifo->id, afifo->source);
            trace_aml_dvb_dma_starvation(afifo->source);
            ring_consume(afifo, 1);
            continue;
        }
        dmx = &dvb->demux[afifo->source];

        /*
         * V2.60: DVR path — vendor dvr_process_channel mantığı.
         *
         * HW section filter (SEC_BUFF_READY IRQ) ve DVR recorder path
         * (TS_RECORDER_ENABLE) birbirinden bağımsızdır:
         *   - Section PAT/PMT/SDT → SEC_BUFF DMA → dmx IRQ → process_section()
         *   - Raw TS → AFIFO DMA → AFIFO IRQ → bu fonksiyon → cb.ts()
         *
         * DVR feed varsa doğrudan cb.ts() ile ilet.
         * DVR feed yoksa (sadece section feed): chunk'ı atla.
         */
        /*
         * V2.60: Çift path desteği
         *   sf_mode=true  → dvb_dmx_swfilter() (SW section/PES filter)
         *   sf_mode=false → cb.ts() (DVR recorder, raw TS)
         */
        if (dmx->sf_mode) {
            /* SW filter path — raw TS tümünü sw filtreye gönder */
            dvb_dmx_swfilter(&dmx->demux, buf, len);
            dev_dbg(dvb->dev, "afifo%d: SW filter chunk=%zu\n",
                    afifo->id, len);
        } else {
            /* DVR path — cb.ts() ile ilet */
            int i;
            bool delivered = false;
            for (i = 0; i < AML_CHANNEL_COUNT; i++) {
                if (dmx->channel[i].used && dmx->channel[i].dvr_feed) {
                    struct dvb_demux_feed *dvr = dmx->channel[i].dvr_feed;
                    if (dvr->cb.ts)
                        dvr->cb.ts(buf, len, NULL, 0,
                                   &dvr->feed.ts, 0);
                    delivered = true;
                    break;
                }
            }
            if (!delivered)
                dev_dbg(dvb->dev, "afifo%d: no DVR feed, chunk skipped\n",
                        afifo->id);
        }

        ring_consume(afifo, 1);
        work_done++;
    }

    dev_dbg(dvb->dev, "afifo%d: poll_work done: %d chunks processed\n", afifo->id, work_done);

    if (work_done >= afifo->poll_budget / 2)
        afifo->poll_budget = min(afifo->poll_budget * 2, 1024U);
    else if (work_done == 0)
        afifo->poll_budget = max(afifo->poll_budget / 2, 64U);

    aml_asyncfifo_update_irq_threshold(afifo);

out:
    /*
     * Race condition düzeltmesi:
     * atomic_set(&pending, 0) ÖNCE yapılmalı, ring_avail kontrolü SONRA.
     *
     * Yanlış sıra:
     *   1. ring_avail() == 0 → döngü bitişi
     *   2. IRQ gelir → pending=1, work kuyruğa eklenir
     *   3. atomic_set(pending, 0) → IRQ'dan gelen 1 silinir
     *   → Work tekrar kuyruğa girmez, yeni veri işlenmez! ←
     *
     * Doğru sıra:
     *   1. atomic_set(pending, 0)  ← pending'i temizle
     *   2. ring_avail() > 0 → hâlâ veri var → pending=1 + yeniden kuyruğa ekle
     *   Eğer 1 ile 2 arasında IRQ gelirse → pending 1 olur → koşul sağlanmaz
     *   ama work zaten IRQ tarafından kuyruğa eklenmiştir. Güvenli.
     */
    atomic_set(&afifo->pending, 0);
    smp_mb();   /* memory barrier: pending=0 store'u ring_avail okumadan önce görünür */

    if (ring_avail(afifo) > 0 && !atomic_xchg(&afifo->pending, 1))
        queue_work_on(afifo->cpu, system_highpri_wq,
                  &afifo->poll_work);
}

void aml_asyncfifo_check_backpressure(struct aml_dvb *dvb)
{
    bool throttle = false;
    int i;

    for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
        struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
        if (!afifo->enabled)
            continue;
        if (ring_avail(afifo) >= afifo->ring_entries * 3 / 4) {
            throttle = true;
            dev_dbg(dvb->dev, "backpressure: afifo%d fill high, throttling\n", i);
            break;
        }
    }

    aml_update_bits(dvb, TS_TOP_CONFIG, BIT(0),
            throttle ? 0 : BIT(0));
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_check_backpressure);

int aml_asyncfifo_init(struct aml_dvb *dvb)
{
	int i, ret;

	dev_info(dvb->dev, "asyncfifo_init: starting, num=%d\n", dvb->caps.num_asyncfifo);

	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
		u32 ctrl_val;

		dev_info(dvb->dev, "asyncfifo_init: initializing FIFO %d\n", i);

		afifo->id     = i;
		afifo->dvb    = dvb;
		afifo->buf_size    = AML_ASYNC_BUF_SIZE;
		afifo->flush_size  = ALIGN(AML_ASYNC_FLUSH_SIZE, SMP_CACHE_BYTES);
		afifo->enabled     = false;
		afifo->source      = i;
		afifo->head        = 0;
		afifo->tail        = 0;
		afifo->prev_chunk  = 0;
		atomic_set(&afifo->pending, 0);
		afifo->poll_budget = 256;
		afifo->ring_entries = RING_ENTRIES(afifo);

		if (!is_power_of_2(afifo->ring_entries)) {
			dev_err(dvb->dev, "FIFO %d: ring_entries=%u not power of 2\n",
				i, afifo->ring_entries);
			ret = -EINVAL;
			goto err_free_buf;
		}

		afifo->irq_threshold_min = AML_ASYNC_FLUSH_SIZE;
		/* threshold_max, ring boyutunun yarısını geçemez.
		 * flush_size * 4 > buf_size/2 olursa IRQ hiç tetiklenmez. */
		afifo->irq_threshold_max = min_t(size_t,
						 AML_ASYNC_FLUSH_SIZE * 4,
						 afifo->buf_size / 2);
		afifo->irq_threshold     = afifo->irq_threshold_min;
		afifo->cpu               = i % num_online_cpus();

		INIT_WORK(&afifo->poll_work, aml_asyncfifo_poll_work);

		/* DMA ring buffer tahsis et */
		dev_info(dvb->dev, "afifo%d: allocating coherent buffer (%zu bytes)\n",
			 i, afifo->buf_size);
		afifo->buf_virt = dma_alloc_coherent(dvb->dev,
						     afifo->buf_size,
						     &afifo->buf_addr,
						     GFP_KERNEL | GFP_DMA32);
		if (!afifo->buf_virt) {
			dev_err(dvb->dev, "afifo%d: dma_alloc_coherent failed\n", i);
			ret = -ENOMEM;
			goto err_free_buf;
		}
		dev_info(dvb->dev, "afifo%d: dma buffer virt=%p dma=%pad\n",
			 i, afifo->buf_virt, &afifo->buf_addr);

		/*
		 * Vendor async_fifo_set_regs() sırası (c_stb_define.h bit tanımları):
		 *
		 * REG0 (+0x00): DMA başlangıç adresi
		 * REG1 (+0x04): FLUSH kontrolü
		 *   bit22 = ASYNC_FIFO_RESET   (pulse — önce set, sonra temizle)
		 *   bit21 = ASYNC_FIFO_WRAP_EN (ring buffer wrap)
		 *   bit20 = ASYNC_FIFO_FLUSH_EN (flush aktif — bu olmadan IRQ gelmez)
		 *   bit[14:0] = ASYNC_FIFO_FLUSH_CNT (128-byte blok sayısı = size>>7)
		 * REG2 (+0x08): FILL kontrolü
		 *   bit[24:23] = ASYNC_FIFO_SOURCE (DMX0=3, DMX1=2, DMX2=0)
		 *   bit[22:21] = ASYNC_FIFO_ENDIAN (1)
		 *   bit20 = ASYNC_FIFO_FILL_EN
		 * REG3 (+0x0C): IRQ threshold (128-byte blok sayısı)
		 *
		 * FIFO[0] → 0xFFD0A000 (ana kanal, aktif)
		 * FIFO[1] → 0xFFD09000 (ikincil, idle)
		 *
		 * devmem doğrulaması: CoreELEC'te REG1=0x80301000
		 *   bit31 = FLUSH_STATUS (RO, veri akıyor)
		 *   bit21 = WRAP_EN ✓  bit20 = FLUSH_EN ✓  FLUSH_CNT=0x1000
		 */
		/* Tüm FIFO'lar eşit başlatılır — ikinci frontend için afifo1 lazım */

		/* ADIM 1: REG0 — DMA ring buffer başlangıç adresi */
		ret = aml_write_async(dvb, ASYNC_FIFO_REG0(i), afifo->buf_addr);
		if (ret) {
			dev_err(dvb->dev, "afifo%d: REG0 write failed: %d\n", i, ret);
			goto err_free_dma;
		}
		dev_info(dvb->dev, "afifo%d: REG0 (DMA addr) = 0x%08x\n", i,
			 (u32)afifo->buf_addr);

		/* ADIM 2: REG1 — RESET pulse: RESET|WRAP_EN|FLUSH_CNT */
		{
			/*
			 * V2.18 KRİTİK DÜZELTME: FLUSH_CNT birimi 128 BYTE'tır, 1KB DEĞİL!
			 *
			 * SM1 cihaz devmem kanıtı (V2.17 çalışma anında):
			 *   flush_cnt = 512 (buf_size >> 10) yazıldığında
			 *   WR_PTR HİÇBİR ZAMAN 64KB'yi geçmedi (22 ölçümün tamamı 0-63KB içinde).
			 *   512 × 128B = 64KB  ← gözlemle TAM EŞLEŞME ✓
			 *   512 × 1KB  = 512KB ← EŞLEŞMIYORDU ✗
			 *
			 * Bug etkisi (V2.17):
			 *   HW ring = 64KB, SW ring_entries = 8 (8×64KB = 512KB).
			 *   Sadece slot 0 (0-64KB) gerçek TS verisi alıyordu.
			 *   Slot 1-7 (64KB-512KB): sıfır/çöp → swfilter section bulamıyor.
			 *   Efektif veri penceresi: 8 IRQ'da 1 kez → PAT kısmen çalışıyor,
			 *   PMT/SDT/NIT zaman aşımına uğruyordu.
			 *
			 * Doğru formül: buf_size >> 7
			 *   512KB / 128B = 4096 = 0x1000 → ring = 4096 × 128B = 512KB ✓
			 *
			 * V2.08 "1KB birim" gözlemi muhtemelen farklı SoC (GXL/GXM) içindi.
			 * SM1 (S905X3) için birim 128 byte olduğu devmem ölçümüyle doğrulandı.
			 */
			u32 flush_cnt = (afifo->buf_size >> 7) & 0x7FFF;

			ctrl_val = AFIFO_RESET | AFIFO_WRAP_EN | flush_cnt;
			aml_write_async(dvb, ASYNC_FIFO_REG1(i), ctrl_val);
			dev_info(dvb->dev, "afifo%d: REG1 reset pulse = 0x%08x (flush_cnt=0x%x)\n",
				 i, ctrl_val, flush_cnt);

			/* RESET bitini temizle */
			ctrl_val &= ~AFIFO_RESET;
			aml_write_async(dvb, ASYNC_FIFO_REG1(i), ctrl_val);

			/* FLUSH_EN set et — bu olmadan IRQ tetiklenmiyor */
			ctrl_val |= AFIFO_FLUSH_EN;
			ret = aml_write_async(dvb, ASYNC_FIFO_REG1(i), ctrl_val);
			if (ret) {
				dev_err(dvb->dev, "afifo%d: REG1 write failed: %d\n",
					i, ret);
				goto err_free_dma;
			}
			dev_info(dvb->dev, "afifo%d: REG1 (WRAP+FLUSH) = 0x%08x\n",
				 i, ctrl_val);
		}

		/* ADIM 3: REG2 — init'te FILL_EN YOK, source da yok.
		 * FILL_EN sadece aml_asyncfifo_set_source() çağrısında açılır.
		 * Burada FILL_EN set edilirse source=0(dmx0) default olur →
		 * asyncfifo0 ve asyncfifo1 aynı kaynaktan beslenir. */
		aml_write_async(dvb, ASYNC_FIFO_REG2(i), 0);
		dev_info(dvb->dev, "afifo%d: REG2 (FILL_EN off, no src) = 0x00000000\n", i);

		/* ADIM 4: REG3 — IRQ threshold + AFIFO_IRQ_EN (BIT21)
		 *
		 * V2.17 KRITIK DÜZELTME: BIT21 (AFIFO_IRQ_EN) OLMADAN IRQ HİÇ TETİKLENMİYOR!
		 *
		 * Kanıt (cihaz devmem):
		 *   - Idle FIFO[1] REG3 = 0x00200000 (BIT21=1, hardware reset default)
		 *   - Vendor aktif FIFO  REG3 = 0x002007FF (BIT21=1 + threshold=0x7FF)
		 *   - V2.16: sadece 0x7FF yazıldı (BIT21=0) → WR_PTR hareket etmesine
		 *     rağmen IRQ hiç gelmedi → poll_work çalışmadı → PAT/PMT sıfır
		 *
		 * REG3 birimi 128 byte: 64KB / 128 - 1 = 511 = 0x1FF
		 * BIT21 ile birlikte: 0x002001FF
		 */
		{
			u32 irq_thresh = (afifo->flush_size >> 7) - 1;

			irq_thresh = min_t(u32, irq_thresh, 0x7FFF);
			irq_thresh |= AFIFO_IRQ_EN;   /* BIT21: IRQ arm — OLMADAN IRQ gelmez! */
			aml_write_async(dvb, ASYNC_FIFO_REG3(i), irq_thresh);
			dev_info(dvb->dev, "afifo%d: REG3 (IRQ thresh) = 0x%06x (every %zuKB)\n",
				 i, irq_thresh, afifo->flush_size / 1024);
		}

		afifo->enabled = true;
		dev_info(dvb->dev,
			 "Async FIFO %d: base=%pad size=%zu flush=%zu REG1=0x%08x CPU%d\n",
			 i, &afifo->buf_addr, afifo->buf_size,
			 afifo->flush_size, ctrl_val, afifo->cpu);
		continue;

err_free_dma:
		dma_free_coherent(dvb->dev, afifo->buf_size,
				  afifo->buf_virt, afifo->buf_addr);
		afifo->buf_virt = NULL;
err_free_buf:
		while (--i >= 0) {
			struct aml_asyncfifo *prev = &dvb->asyncfifo[i];

			if (!prev->enabled)
				continue;
			cancel_work_sync(&prev->poll_work);
			aml_write_async(dvb, ASYNC_FIFO_REG1(i), 0);
			dma_free_coherent(dvb->dev, prev->buf_size,
					  prev->buf_virt, prev->buf_addr);
			prev->enabled = false;
		}
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_init);

void aml_asyncfifo_release(struct aml_dvb *dvb)
{
	int i;

	dev_info(dvb->dev, "asyncfifo_release\n");

	for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
		struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];

		if (!afifo->enabled)
			continue;

		dev_dbg(dvb->dev, "afifo%d: releasing\n", i);
		cancel_work_sync(&afifo->poll_work);
		/* FIFO'yu durdur: FLUSH_EN ve FILL_EN temizle */
		aml_write_async(dvb, ASYNC_FIFO_REG1(i), 0);
		aml_write_async(dvb, ASYNC_FIFO_REG2(i), 0);
		dma_free_coherent(dvb->dev, afifo->buf_size,
				  afifo->buf_virt, afifo->buf_addr);
		afifo->buf_virt = NULL;
		afifo->enabled = false;
	}
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_release);

int aml_asyncfifo_set_source(struct aml_asyncfifo *afifo, unsigned int source)
{
	struct aml_dvb *dvb = afifo->dvb;
	/* Vendor reset_async_fifos: DMX0→3, DMX1→2, DMX2→0 */
	static const u8 src_map[] = { AFIFO_SRC_DMX0, AFIFO_SRC_DMX1, AFIFO_SRC_DMX2 };
	u32 src_val, reg2;

	dev_dbg(dvb->dev, "afifo%d: set_source %u\n", afifo->id, source);

	if (!afifo->enabled)
		return -EINVAL;
	if (source >= dvb->caps.num_demux)
		return -EINVAL;

	afifo->source = source;
	src_val = (source < ARRAY_SIZE(src_map)) ? src_map[source] : 0;

	/* REG2: SOURCE[24:23] | ENDIAN | FILL_EN
	 * Kaynak bağlantısını değiştir, FILL_EN ve ENDIAN sabit kalır */
	reg2 = AFIFO_SOURCE(src_val) | AFIFO_ENDIAN_VAL | AFIFO_FILL_EN;
	dev_info(dvb->dev, "afifo%d: REG2 source=%u val=%u reg2=0x%08x\n",
		 afifo->id, source, src_val, reg2);
	return aml_write_async(dvb, ASYNC_FIFO_REG2(afifo->id), reg2);
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_set_source);

irqreturn_t aml_asyncfifo_irq_handler(int irq, void *dev_id)
{
	struct aml_asyncfifo *afifo = dev_id;
	struct aml_dvb *dvb = afifo->dvb;
	u32 wr_ptr, new_head;

	dev_dbg(dvb->dev, "afifo%d: IRQ fired! irq=%d\n", afifo->id, irq);

	/*
	 * Delta yaklaşımı — en güvenli yöntem.
	 *
	 * abs_chunk = WR_PTR ring içindeki anlık pozisyon (0..ring_entries-1).
	 * prev_chunk = bir önceki IRQ'daki abs_chunk.
	 * delta = kaç yeni chunk doldu (wrap-safe).
	 *
	 * head monoton artan tam sayıdır (asla maskelenmez).
	 * ring_avail = (head - tail) & RING_MASK → her zaman doğru.
	 */
	if (aml_read_async(dvb, ASYNC_FIFO_WR_PTR(afifo->id), &wr_ptr) == 0) {
		u32 ring_mask  = afifo->ring_entries - 1;
		u32 base_addr  = (u32)afifo->buf_addr;
		u32 ring_size  = (u32)afifo->buf_size;
		u32 offset     = (wr_ptr - base_addr) & (ring_size - 1);
		u32 abs_chunk  = offset / (u32)afifo->flush_size;
		u32 prev_abs   = READ_ONCE(afifo->prev_chunk);
		u32 delta      = (abs_chunk - prev_abs) & ring_mask;

		/* En az 1 yeni chunk garanti et */
		if (delta == 0)
			delta = 1;

		WRITE_ONCE(afifo->prev_chunk, abs_chunk);
		new_head = READ_ONCE(afifo->head) + delta;
	} else {
		new_head = READ_ONCE(afifo->head) + 1;
	}

	smp_store_release(&afifo->head, new_head);

	aml_asyncfifo_check_backpressure(dvb);

	if (!atomic_xchg(&afifo->pending, 1))
		queue_work_on(afifo->cpu, system_highpri_wq,
			      &afifo->poll_work);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_irq_handler);

bool aml_asyncfifo_has_data(struct aml_dvb *dvb)
{
    int i;
    for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
        struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
        if (READ_ONCE(afifo->head) != READ_ONCE(afifo->tail))
            return true;
    }
    return false;
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_has_data);

void aml_asyncfifo_process_all(struct aml_dvb *dvb)
{
    int i;
    for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
        struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
        if (afifo->enabled && !atomic_xchg(&afifo->pending, 1))
            queue_work_on(afifo->cpu, system_highpri_wq,
                      &afifo->poll_work);
    }
}
EXPORT_SYMBOL_GPL(aml_asyncfifo_process_all);

u32 aml_get_ring_fill_percent(struct aml_dvb *dvb)
{
    u32 max_fill = 0;
    int i;

    for (i = 0; i < dvb->caps.num_asyncfifo; i++) {
        struct aml_asyncfifo *afifo = &dvb->asyncfifo[i];
        if (!afifo->enabled)
            continue;
        u32 fill = ring_avail(afifo);
        u32 percent = fill * 100 / afifo->ring_entries;
        if (percent > max_fill)
            max_fill = percent;
    }
    return max_fill;
}
EXPORT_SYMBOL_GPL(aml_get_ring_fill_percent);
