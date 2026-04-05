# AMLOGIC DVB VENDOR KERNEL DRIVER
## Full In-Depth Technical Analysis

**`aml_dmx.c` / `aml_dvb.c` — Kernel 5.15 Vendor Driver**

*Mainline TS Connection Fault Analysis for Amlogic-DVB-V2*

**Produced:** 4 March 2026

---

## 1. System Architecture and Hardware Flow

### 1.1 Physical TS Input Hierarchy

The complete hardware signal flow of the Amlogic STB demultiplexer is as follows:

```
Physical TS Pins
|
[Pinctrl/Pad Mux] <-- DTS selected with pinctrl-names
|
+----+----+----+
TS0  TS1  TS2  TS3 <-- Parallel mode: direct input
|
[S2P Converter 0/1/2] <-- Serial mode: serial-to-parallel conversion
|
[FEC Input MUX] <-- Selected with register FEC_INPUT_CONTROL
| fec_sel=0..3: Parallel TS0-3
| fec_sel=4: S2P2   fec_sel=5: S2P1   fec_sel=6: S2P0
| fec_sel=7: HIU(file)   fec_sel=8: HIU1
|
[DMX0 / DMX1 / DMX2] <-- 3 demultiplexers
|
[AsyncFIFO 0/1/2] <-- DVR/recording output
```

### 1.2 Hardware Differences Based on CPU Type

| CPU Family | STB_CBUS_BASE | TS Input Count | S2P Count | AsyncFIFO |
|---|---|---|---|---|
| GXBB/GXL/GXM/TXL/GXLX | `0x1600` | 3 | 2 | 2 |
| G12A/G12B/SM1 | `0x1800` | 3 | 2 | 2 |
| TL1/TM2/T7 and above | `0x1800` | 4 | 3 | 3 |

**STB_CBUS_BASE selection condition:**

```c
(get_cpu_type() > MESON_CPU_MAJOR_ID_TXL) && (get_cpu_type() != MESON_CPU_MAJOR_ID_GXLX)
    ? 0x1800 : 0x1600
```

---

## 2. Driver Probe Function — Complete Flow

### 2.1 `aml_dvb_probe()` Step by Step

1. **Determine CPU type** (`get_cpu_type()` call)

2. **Clock management — split depending on CPU:**
   - If `get_cpu_type() < MESON_CPU_MAJOR_ID_G12A`:
     ```
     devm_clk_get(&pdev->dev, "demux")      -> clk_prepare_enable()
     devm_clk_get(&pdev->dev, "asyncfifo")  -> clk_prepare_enable()
     devm_clk_get(&pdev->dev, "ahbarb0")    -> clk_prepare_enable()
     devm_clk_get(&pdev->dev, "uparsertop") -> clk_prepare_enable()
     ```
   - If `get_cpu_type() >= MESON_CPU_MAJOR_ID_G12A` (G12A, SM1, TL1…):
     ```
     amports_switch_gate("demux", 1)
     amports_switch_gate("ahbarb0", 1)
     amports_switch_gate("parser_top", 1)
     // For TL1 additionally: devm_clk_get("asyncfifo") -> clk_prepare_enable()
     ```

3. **`ts_in_total_count` and `s2p_total_count` are determined:**
   ```
   CPU < TL1:  ts_in_total_count=3, s2p_total_count=2, async_fifo_total_count=2
   CPU >= TL1: ts_in_total_count=4, s2p_total_count=3, async_fifo_total_count=3
   ```

4. **TS configuration is read from DTS (`CONFIG_OF` block):**
   For each `i = 0..ts_in_total_count-1`:
   ```
   DTS property name: "ts0", "ts1", "ts2", "ts3"
   Value "serial"   --> pinctrl name: "s_ts0", "s_ts1"...
                    --> devm_pinctrl_get_select(&pdev->dev, "s_ts0")
                    --> advb->ts[i].mode = AM_TS_SERIAL
                    --> advb->ts[i].s2p_id = s2p_id++ (increments in order)
   Value "parallel" --> pinctrl name: "p_ts0", "p_ts1"...
                    --> devm_pinctrl_get_select(&pdev->dev, "p_ts0")
                    --> advb->ts[i].mode = AM_TS_PARALLEL
   Other/missing    --> AM_TS_DISABLE, pinctrl=NULL
   ts_control DTS:  "ts0_control", "ts1_control" (u32, FEC_INPUT_CONTROL sub-bits [11:0])
   ts_invert DTS:   "ts0_invert", "ts1_invert" (S2P clock invert)
   ```

5. **DSC, DMX, AsyncFIFO hardware is initialised**

6. **Each active TS input is assigned to DMX:**
   ```c
   for (ts=0, i=0; i < ts_in_total_count; i++)
       if (ts[i].mode != DISABLE)
           aml_dmx_hw_set_source(dmx[ts].demux, DMX_SOURCE_FRONT0+i)
           ts++
   ```

7. The sysfs class is registered, and `tsdemux_ops` is checked.

---

## 3. Complete Register Map

### 3.1 `STB_TOP_CONFIG` (`STB_CBUS_BASE + 0xF0`)

This register specifies the physical pin selection of S2P converters and the TSO output source.

| Bit Field | Bits | Value | Description |
|---|---|---|---|
| `CIPLUS_OUT_SEL` | [30:28] | 0..7 | CI+ output DMX selection (bitmask) |
| `CIPLUS_IN_SEL` | [27:26] | 0..2 | CI+ input DMX selection |
| `INVERT_S2P1_FEC_ERROR` | [22] | 0/1 | S2P1 fault signal inverter |
| `INVERT_S2P1_FEC_DATA` | [21] | 0/1 | S2P1 data inverter |
| `INVERT_S2P1_FEC_SYNC` | [20] | 0/1 | S2P1 sync inverter |
| `INVERT_S2P1_FEC_VALID` | [19] | 0/1 | S2P1 valid inverter |
| `INVERT_S2P1_FEC_CLK` | [18] | 0/1 | S2P1 clock inverter |
| `S2P1_FEC_SERIAL_SEL` | [17:16] | 0..3 | Which TS pin does S2P1 use? |
| `ENABLE_DES_PL_CLK` | [15] | 0/1 | Descrambler clock |
| `ENABLE_DES_PL` | [7] | 0/1 | Descrambler enabled |
| `DES_INPUT_SEL` | [9:8] | 0..2 | Descrambler input DMX (0=DMX0) |
| `TS_OUTPUT_SOURCE` | [12:10] | 0..7 | TSO output source |
| `INVERT_S2P0_FEC_CLK` | [2] | 0/1 | S2P0 clock inverter |
| `S2P0_FEC_SERIAL_SEL` | [1:0] | 0..3 | Which TS pin does S2P0 use? |

> ⚠ **CRITICAL:** The `S2P_FEC_SERIAL_SEL` value equals the physical TS pin index. If `ts[0].s2p_id==0`, then `fec_s0=0` (TS0 pin to S2P0). Serial TS will not work without writing this to `STB_TOP_CONFIG`.

### 3.2 `FEC_INPUT_CONTROL` (`STB_CBUS_BASE + 0x02 / 0x52 / 0xA2`)

Offsets: DMX0=`0x02`, DMX1=`0x52`, DMX2=`0xA2` (on G12A: `0x1802`, `0x1852`, `0x18A2`)

| Bit Field | Bits | Value | Description |
|---|---|---|---|
| `FEC_SEL_3BIT` | [16] | 0/1 | Additional bit selector for HIU1 |
| `FEC_CORE_SEL` | [15] | 0/1 | 1: For Descrambler/CIPLUS |
| `FEC_SEL` | [14:12] | 0..7 | TS source selector (see below) |
| `fec_ctrl`/`ts_control` | [11:0] | From DTS | `ts_control` DTS property |

**`FEC_SEL` value meanings:**

| FEC_SEL | Source | Code |
|---|---|---|
| 0 | Parallel TS0 (`AM_TS_SRC_TS0`) | `fec_sel = 0` |
| 1 | Parallel TS1 (`AM_TS_SRC_TS1`) | `fec_sel = 1` |
| 2 | Parallel TS2 (`AM_TS_SRC_TS2`) | `fec_sel = 2` |
| 3 | Parallel TS3 (`AM_TS_SRC_TS3`) | `fec_sel = 3` |
| 4 | Serial S2P2 (highest s2p_id) | `fec_sel = 6 - s2p_id` (s2p_id=2) |
| 5 | Serial S2P1 | `fec_sel = 6 - s2p_id` (s2p_id=1) |
| 6 | Serial S2P0 (lowest s2p_id) | `fec_sel = 6 - s2p_id` (s2p_id=0) |
| 7 | HIU / file source | `fec_sel = 7` |
| 8 (bit16=1) | HIU1 | `fec_sel = 8` (custom code) |
| -1 | DMX cascade (DMX0/1/2) | `dmx_cascade_set()` call |

> ⚠ **CRITICAL: REVERSE ORDER** — As S2P ID increases, FEC_SEL decreases. Formula: `fec_sel = 6 - s2p_id`. If the mainline driver directly maps these, the wrong TS source is selected.

### 3.3 `DEMUX_CONTROL` (`STB_CBUS_BASE + 0x01 / 0x51 / 0xA1`)

| Bit Field | Bit | Value | Description |
|---|---|---|---|
| `ENABLE_FREE_CLK_FEC_DATA_VALID` | 31 | 1 | Must always be 1 |
| `ENABLE_FREE_CLK_STB_REG` | 30 | 1 | Must always be 1 |
| `TS_RECORDER_SELECT` | 10 | 0/1 | `dump_ts_select` DTS property |
| `TS_RECORDER_ENABLE` | 9 | 0/1 | AsyncFIFO recording active |
| `NOT_USE_OF_SOP_INPUT` | 5 | 0/1 | SOP input preference |
| `STB_DEMUX_ENABLE` | 4 | 1 | Demux enabled — no data without this |
| `SECTION_END_WITH_TABLE_ID` | 3 | 1 | Section end table ID |
| `KEEP_DUPLICATE_PACKAGE` | 1 | 1 | Default: 1 (module parameter) |

### 3.4 `STB_S2P2_CONFIG` (`STB_CBUS_BASE + 0xEF`)

Used only on TL1 and higher SoCs. Content is in the same format as `STB_TOP_CONFIG` (for S2P2).

### 3.5 `TS_FILE_CONFIG` (`STB_CBUS_BASE + 0xF2`)

HIU (file) mode configuration — for `demux_skipbyte`, clock divider, and HIU active bit.

| Field | Bits | Description |
|---|---|---|
| `demux_skipbyte` | [31:16] | Number of bytes skipped (default: 0) |
| `DES_OUT_DLY` | [10:8] | Descrambler output delay (6) |
| `TS_HIU_ENABLE` | [5] | HIU mode enabled (reading TS from file) |
| `FEC_FILE_CLK_DIV` | [4:0] | FEC clock divider (`tsfile_clkdiv=5`) |

### 3.6 Other Important Registers

| Register | Offset | Description |
|---|---|---|
| `STB_VERSION` | `+0x00` | Hardware version (readable; write test performed) |
| `STB_TEST_REG` | `+0x01` | Register access test (`0x5550` and `0xAAA0`) |
| `DEMUX_CONTROL` | `+0x01` | DMX main control (see §3.3) |
| `FEC_INPUT_CONTROL` | `+0x02` | FEC source selector (see §3.2) |
| `STB_INT_MASK` | `+0x04` | Interrupt mask — enabled with `DEMUX_INT_MASK` |
| `STB_INT_STATUS` | `+0x05` | Interrupt status — cleared after processing |
| `DEMUX_MEM_REQ_EN` | `+0x06` | AHB DMA active bits |
| `PES_STRONG_SYNC` | `+0x0A` | PES sync (`0x1234`) |
| `DEMUX_ENDIAN` | `+0x0C` | Byte order settings |
| `MAX_FM_COMP_ADDR` | `+0x0D` | Filter memory instruction address |
| `FM_WR_DATA/ADDR` | `+0x10/11` | PID table write |
| `SEC_BUFF_BASE` | `+0x14` | Section buffer base address |
| `SEC_BUFF_01_START` | `+0x15` | Section buffer 0–1 start |
| `SEC_BUFF_23_START` | `+0x16` | Section buffer 2–3 start |
| `STB_TOP_CONFIG` | `+0xF0` | S2P routing and TSO output (see §3.1) |
| `TS_TOP_CONFIG` | `+0xF1` | TS output clock inverter |
| `TS_FILE_CONFIG` | `+0xF2` | HIU/file mode (see §3.5) |

---

## 4. Complete Analysis of Critical Functions

### 4.1 `stb_enable()` — S2P and TSO Configuration Writer

This function sets the `STB_TOP_CONFIG` and `TS_FILE_CONFIG` registers. `stb_enable()` is called from within `dmx_reset_hw_ex()`.

#### 4.1.1 Source Identification Logic

```
If dvb->stb_source == AM_TS_SRC_DMXx:
    src = dvb->dmx[x].source  (use the actual source of that DMX)
Otherwise:
    src = dvb->stb_source
```

#### 4.1.2 Writing `STB_TOP_CONFIG`

```c
// Determine which TS pin feeds each S2P:
for i in ts_in_total_count:
    if ts[i].s2p_id == 0: fec_s0 = i
    if ts[i].s2p_id == 1: fec_s1 = i
    if ts[i].s2p_id == 2: fec_s2 = i

WRITE(STB_TOP_CONFIG,
    (s2p[1].invert << 18) |   // INVERT_S2P1_FEC_CLK
    (fec_s1 << 16)        |   // S2P1_FEC_SERIAL_SEL: point to TS pin
    (out_src << 10)       |   // TS_OUTPUT_SOURCE: TSO output
    (des_in << 8)         |   // DES_INPUT_SEL
    (en_des << 7)         |   // ENABLE_DES_PL
    (dec_clk_en << 15)    |   // ENABLE_DES_PL_CLK
    (s2p[0].invert << 2)  |   // INVERT_S2P0_FEC_CLK
    (fec_s0 << 0)         )   // S2P0_FEC_SERIAL_SEL: point to TS pin

// Additional S2P2 register for TL1 and above:
if CPU >= TL1:
    WRITE(STB_S2P2_CONFIG,
        (s2p[2].invert << 2) |
        (fec_s2 << 0)        )
```

### 4.2 `dmx_enable()` — FEC Resource Selector

The main function that enables the demux and writes the `FEC_INPUT_CONTROL` and `DEMUX_CONTROL` registers.

#### 4.2.1 FEC_SEL Calculation for Serial Mode (CRITICAL)

```
case AM_TS_SRC_S_TS0: s2p_id = 0; fec_sel = 6 - 0 = 6
case AM_TS_SRC_S_TS1: s2p_id = 1; fec_sel = 6 - 1 = 5
case AM_TS_SRC_S_TS2: s2p_id = 2; fec_sel = 6 - 2 = 4
```

> ⚠ **CRITICAL: REVERSE ORDER OF FEC_SEL** — Formula `fec_sel = 6 - s2p_id`. s2p_id=0 → fec_sel=6, s2p_id=1 → fec_sel=5, s2p_id=2 → fec_sel=4. This reverse order is **mandatory** due to the hardware design.

#### 4.2.2 `STB_TOP_CONFIG` Update (in Serial Mode)

```c
if set_stb == 1 (serial mode):
    v = READ(STB_TOP_CONFIG)
    v &= ~(mask: S2P0_FEC_SERIAL_SEL, INVERT_S2P0, S2P1_FEC_SERIAL_SEL, INVERT_S2P1)
    v |= (fec_s0 << 0) | (invert0 << 2) | (fec_s1 << 16) | (invert1 << 18)
    WRITE(STB_TOP_CONFIG, v)
```

#### 4.2.3 All Registers Written When Channel is Active

```
STB_INT_MASK        <- DEMUX_INT_MASK         (enable interrupts)
DEMUX_MEM_REQ_EN    <- AHB DMA and packet type bits
PES_STRONG_SYNC     <- 0x1234
DEMUX_ENDIAN        <- endian settings
TS_HIU_CTL          <- USE_HI_BSF_INTERFACE bit
FEC_INPUT_CONTROL   <- fec_core_sel | (fec_sel << 12) | fec_ctrl
STB_OM_CTL          <- DMA counter and address limits
VIDEO_STREAM_ID     <- 0xFFFF0000 or 0 depending on recording mode
DEMUX_CONTROL       <- STB_DEMUX_ENABLE | ENABLE_FREE_CLK bits | register bits
```

### 4.3 `aml_dmx_hw_set_source()` — Source Selection

`DMX_SOURCE_FRONT0..3` translation to hardware source:

```
DMX_SOURCE_FRONT0 → is ts[0].mode == SERIAL?
    YES: hw_src = ts[0].s2p_id + AM_TS_SRC_S_TS0
         (AM_TS_SRC_S_TS0 if s2p_id=0, AM_TS_SRC_S_TS1 if s2p_id=1)
    NO:  hw_src = AM_TS_SRC_TS0
If changed: dmx_reset_dmx_hw_ex_unlock() -> stb_enable() + dmx_enable()
```

### 4.4 `dmx_reset_hw_ex()` — Full Reset/Restart

This function clears and rebuilds all driver registers — all demux initialisation passes through here:

1. Disable IRQs
2. `WRITE(RESET1_REGISTER, RESET_DEMUXSTB)` — reset hardware
3. Wait for `OM_CMD_STATUS` to reset (max 1,000,000 iterations)
4. `WRITE(STB_TOP_CONFIG, 0)` — clear
5. `WRITE(STB_S2P2_CONFIG, 0)` — clear
6. Re-enable IRQs
7. `DEMUX_CONTROL=0`, register access test (`0x5550`, `0xAAA0`)
8. `stb_enable(dvb)` — S2P routing and file config
9. For each DMX: buffer addresses, channel/filter registers, `dmx_enable()`
10. Restore DSC (descrambler) PID and switches

---

## 5. DTS Requirements — Full Configuration

### 5.1 Mandatory DTS Properties

| DTS Property | Type | Example Value | Description |
|---|---|---|---|
| `ts0` | string | `"serial"` or `"parallel"` | TS0 mode |
| `ts1` | string | `"serial"` or `"parallel"` | TS1 mode |
| `ts2` | string | `"serial"` or `"parallel"` | TS2 mode (optional) |
| `ts0_control` | u32 | `0x0` | `FEC_INPUT_CONTROL[11:0]` sub-bits |
| `ts1_control` | u32 | `0x0` | `FEC_INPUT_CONTROL[11:0]` sub-bits |
| `ts0_invert` | u32 | `0` or `1` | Serial clock invert (serial mode only) |
| `ts1_invert` | u32 | `0` or `1` | Serial clock inverter |
| `asyncfifo_buff_len` | u32 | `524288` | AsyncFIFO buffer size (bytes) |
| `sub_ttx` | u32 | `1` | Subtitles/teletext enabled/disabled |
| `tsin_deglitch` | u32 | `0` or `1` | TSIN glitch suppression |
| `ts_out_invert` | u32 | `0` or `1` | TS output clock inverter |

### 5.2 Pinctrl State Names — Exact Syntax

The vendor driver generates pinctrl state names in the following format:

```c
// Serial TS:   snprintf(buf, "s_ts%d", i)
// Parallel TS: snprintf(buf, "p_ts%d", i)
```

| DTS `pinctrl-names` | TS Mode | Description |
|---|---|---|
| `"s_ts0"` | serial | TS0 serial pins (MDAT, CLK, SYNC, VALID, ERR) |
| `"s_ts1"` | serial | TS1 serial pins |
| `"s_ts2"` | serial | TS2 serial pins (TL1+) |
| `"p_ts0"` | parallel | TS0 parallel pins (8-bit data path) |
| `"p_ts1"` | parallel | TS1 parallel pins |
| `"p_ts2"` | parallel | TS2 parallel pins |

> ⚠ **CRITICAL:** Pinctrl names are **CASE-SENSITIVE**. `'S_TS0'` or `'serial_ts0'` **WILL NOT WORK**. `devm_pinctrl_get_select()` returns an error, TS pins are not muxed, and the physical signal never reaches the hardware.

### 5.3 Example DTS Block (G12A, ts0=serial, ts1=serial)

```dts
dvb@1800 {
    compatible = "amlogic, dvb-demux";
    reg = <0x0 0xfe018000 0x0 0x400>;
    interrupts = <0 20 1>, <0 21 1>, <0 22 1>;
    interrupt-names = "demux0_irq","demux1_irq","demux2_irq";
    clocks = <&clkc CLKID_DEMUX>,<&clkc CLKID_ASYNC_FIFO>,
             <&clkc CLKID_AHB_ARB0>,<&clkc CLKID_PARSER_TOP>;
    clock-names = "demux","asyncfifo","ahbarb0","uparsertop";
    resets = <&reset RESET_DEMUX>;

    ts0 = "serial";
    ts0_control = <0x0>;
    ts0_invert = <0>;

    ts1 = "serial";
    ts1_control = <0x0>;
    ts1_invert = <0>;

    asyncfifo_buf_len = <0x80000>;  /* 512KB */
    sub_ttx = <1>;

    pinctrl-names = "s_ts0", "s_ts1";  /* EXACT ORDER: serial first */
    pinctrl-0 = <&dvb_s_ts0_pins>;
    pinctrl-1 = <&dvb_s_ts1_pins>;
};
```

---

## 6. Interrupt Processing and Data Flow

### 6.1 `dmx_irq_handler()` — Demux Interrupt Handler

A separate interrupt vector exists for each DMX unit (`demux0_irq`, `demux1_irq`, `demux2_irq`).

1. `STB_INT_STATUS` is read
2. Branch according to status:
   ```
   SECTION_BUFFER_READY -> process_section()
   SUB_PES_READY        -> process_sub()  (subtitles)
   OTHER_PES_READY      -> process_pes()
   OM_CMD_READ_PENDING  -> process_om_read()
   TS_ERROR_PIN         -> pr_error (TS error pin)
   NEW_PDTS_READY       -> update video_pts/audio_pts
   ```
3. Clear `STB_INT_STATUS` (by writing)
4. Monitor IRQ storm (>1000 interrupts within 100 ms = reset)

### 6.2 AsyncFIFO / DVR Flow

DVR (recording) data flows through AsyncFIFO hardware:

```
dvr_irq_handler():
    afifo->buf_toggle++ % (total/flush_size)
    tasklet_schedule(&asyncfifo_tasklet)

dvr_irq_bh_handler():
    DMA synchronisation (dma_sync_single_for_cpu)
    dvr_process_channel() -> feed->cb.ts() callback
```

### 6.3 Section Filtering Process

```
Hardware match -> hardware_match_section() -> section_crc() -> section_notify()
CRC error      -> trigger_crc_monitor() -> >100 errors within 100ms = DMX reset
Software match -> software_match_section() (fallback mechanism)
Watchdog: every 250ms, check SEC_BUFF_BUSY for all DMX units.
          If a fault occurs, execute dmx_reset_dmx_hw_ex_unlock().
```

---

## 7. Buffer Management

| Buffer Type | Allocation Function | Size Formula | Use |
|---|---|---|---|
| Section | `dmx_alloc_sec_buffer()` | 4 groups × (1<<12)×8 = 131 KB/DMX | SI/PSI table data |
| PES | `dmx_alloc_pes_buffer()` | `of_node` or default | Audio/video PES packets |
| Subtitles | `dmx_alloc_sub_buffer()` | Shared (DVB-wide) | Subtitles and teletext |
| AsyncFIFO | `asyncfifo_alloc_buffer()` | `asyncfifo_buff_len` (512 KB) | DVR recording buffers |
| SmallSec | `kmalloc` (on demand) | `SS_BUFSIZE_DEF` ~4 KB | Fast small-section path |

All buffers are allocated in kernel space with `__get_free_pages()` and DMA-mapped with `dma_map_single()`. Release: `dma_unmap_single()` + `free_pages()`.

---

## 8. Complete Analysis of the Reasons for Mainline Driver Failure

The following 7 combinations of causes can lead to a failed TS connection on the mainline `amlogic-dvb-v2` driver.

### Reason 1: `STB_CBUS_BASE` Incorrect Value

> ⚠ **CRITICAL:** The value is `0x1800` for G12A/SM1/TL1 and `0x1600` for the older GXBB/GXL. An incorrect base means all register writes go to the wrong physical address.

```c
// Vendor calculation:
(CPU > MESON_CPU_MAJOR_ID_TXL) && (CPU != MESON_CPU_MAJOR_ID_GXLX)
    ? 0x1800 : 0x1600

// Mainline fix — use SoC-specific data table:
static const struct meson_dmx_data g12a_data = {
    .stb_cbus_base = 0x1800,
    .ts_in_count = 3,
    .s2p_count = 2,
};
```

### Reason 2: Pinctrl State Names Do Not Match

> ⚠ **CRITICAL:** If the `devm_pinctrl_get_select(dev, "s_ts0")` call fails, `IS_ERR_VALUE()` is true, the TS pins are not muxed, and there is no physical signal.

Essential DTS entries:

```dts
pinctrl-names = "s_ts0";  // for serial TS0
pinctrl-names = "p_ts0";  // for parallel TS0
```

Verification:

```bash
cat /sys/kernel/debug/pinctrl/*/pinmux-pins | grep -i ts
```

### Reason 3: `STB_TOP_CONFIG` Not Written

> ⚠ **CRITICAL:** This register determines which physical TS pin group the S2P converters will use. Serial TS will not function at all without it.

`stb_enable()` writes to this register. If the mainline driver omits it, serial mode will not generate any data.

```c
// Which pin group feeds S2P0:
// ts[0] serial, ts[0].s2p_id=0, therefore fec_s0=0
WRITE(STB_TOP_CONFIG, (fec_s0 << S2P0_FEC_SERIAL_SEL) | ...)
```

### Reason 4: FEC_SEL Reverse Ordering Not Applied

> ⚠ **CRITICAL:** The formula `fec_sel = 6 - s2p_id` is **mandatory**. Direct mapping (`fec_sel = s2p_id`) will bind an incorrect S2P output to the demux.

```c
// WRONG (mainline error):
fec_sel = s2p_id;       // s2p_id=0 -> fec_sel=0 = parallel TS0!

// CORRECT (vendor code):
fec_sel = 6 - s2p_id;  // s2p_id=0 -> fec_sel=6 = S2P0
```

### Reason 5: Incomplete or Incorrect Clock Management

> ⚠ **CRITICAL:** Use `amports_switch_gate()` for G12A and above, and `clk_prepare_enable()` for models prior to G12A. An incorrect clock configuration will silently prevent data flow through hardware registers.

```c
// For G12A and above:
amports_switch_gate("demux", 1);      // STB clock gate
amports_switch_gate("ahbarb0", 1);    // AHB arbitration clock
amports_switch_gate("parser_top", 1); // Parser clock

// Mainline CCF equivalent:
clk_prepare_enable(priv->clk_demux);
clk_prepare_enable(priv->clk_ahbarb0);
reset_control_deassert(priv->reset_demux);
```

### Reason 6: `s2p_id` Assignment is Incorrect

> ⚠ **CRITICAL:** `s2p_id` is assigned in probe via an incrementing counter for each serial TS input. ts0=serial → s2p_id=0, ts1=serial → s2p_id=1. This mapping is reflected jointly in `STB_TOP_CONFIG` and `FEC_SEL`.

```c
int s2p_id = 0;  // zero at probe start
for (i = 0; i < ts_in_count; i++) {
    if (mode == "serial") {
        ts[i].s2p_id = s2p_id++;
    } else {
        ts[i].s2p_id = -1;
    }
}
```

### Reason 7: IRQ Mapping is Incomplete

> ⚠ **CRITICAL:** `dmx_irq_handler()` must be registered via `request_threaded_irq()` with the `IRQF_SHARED|IRQF_TRIGGER_RISING` flags. Without the IRQ, data remains stuck in kernel buffers and never reaches userspace.

```c
request_threaded_irq(dmx->dmx_irq,
    dmx_irq_handler,
    dmx_irq_thread_handler,
    IRQF_SHARED|IRQF_TRIGGER_RISING,
    "dmx irq", dmx);
```

---

## 9. Complete Correct Initialisation Sequence

### 9.1 Full Probe Sequence

1. **Determine CPU type** — select `STB_CBUS_BASE`
2. **Enable clocks** — G12A+: `amports_switch_gate()`, others: `clk_prepare_enable()`
3. **Read DTS** — ts0/ts1 mode, `ts_control`, `ts_invert`
4. **Apply pinctrl** — `devm_pinctrl_get_select()` `s_ts0`/`p_ts0`
5. **Assign `s2p_id`** — incrementally to serial inputs: 0, 1, 2
6. **Request IRQs** — `request_threaded_irq()` for each DMX
7. **Allocate buffers** — section, PES, subtitle, AsyncFIFO
8. **Initial reset** — `WRITE(RESET1_REGISTER, RESET_DEMUXSTB)`
9. **`stb_enable()`** — `STB_TOP_CONFIG`, `TS_FILE_CONFIG`
10. **`dmx_enable()`** — `FEC_INPUT_CONTROL`, `DEMUX_CONTROL`
11. **Initialise AsyncFIFO** — `async_fifo_init()`, connect source
12. **TS-DMX mapping** — `aml_dmx_hw_set_source()` for each active TS

### 9.2 `FEC_INPUT_CONTROL` Correct Value Examples

| Scenario | FEC_SEL | fec_ctrl | Full Value |
|---|---|---|---|
| TS0 parallel → DMX0 | 0 (bit14:12=0) | ts0_control DTS | `0x000000` |
| TS0 serial (s2p_id=0) → DMX0 | 6 (bit14:12=6) | ts0_control DTS | `0x006000` |
| TS1 serial (s2p_id=1) → DMX0 | 5 (bit14:12=5) | ts1_control DTS | `0x005000` |
| TS2 serial (s2p_id=2) → DMX0 | 4 (bit14:12=4) | ts2_control DTS | `0x004000` |
| HIU/file → DMX0 | 7 (bit14:12=7) | 0 | `0x007000` |

---

## 10. Working Diagnostic Commands

### 10.1 Register Reads (G12A Example)

For G12A, `STB_CBUS_BASE = 0x1800`, physical address `0xFE018000`:

```bash
# DEMUX_CONTROL (DMX0)
devmem2 0xFE018001 w
# Bit4=1 (STB_DEMUX_ENABLE), bit30=1, bit31=1

# FEC_INPUT_CONTROL (DMX0)
devmem2 0xFE018002 w
# Serial TS0: bit14:12=6 (0x6000), parallel TS0: bit14:12=0

# STB_TOP_CONFIG
devmem2 0xFE0180F0 w
# S2P0_FEC_SERIAL_SEL bit1:0 = TS pin index

# FEC_INPUT_CONTROL (DMX1)
devmem2 0xFE018052 w

# FEC_INPUT_CONTROL (DMX2)
devmem2 0xFE0180A2 w
```

### 10.2 Clock Status

```bash
cat /sys/kernel/debug/clk/clk_summary | grep -E 'demux|ahbarb|parser'
# demux and ahbarb0 must be enabled and clk != 0
```

### 10.3 Pinctrl Status

```bash
cat /sys/kernel/debug/pinctrl/*/pinmux-pins | grep -i 'ts\|dvb'
# ts_d[0..7], ts_clk, ts_sync, ts_valid, ts_fail must be assigned to dvb-demux
```

### 10.4 Sysfs Source Control

```bash
# Show STB source
cat /sys/class/stb/source

# Set DMX0 source to ts0
echo ts0 > /sys/class/stb/demux0_source
echo ts0 > /sys/class/stb/source

# DMX IRQ statistics
cat /proc/interrupts | grep dmx
```

### 10.5 Hardware Version Check

```bash
# STB_VERSION register (G12A: address 0xFE018000)
devmem2 0xFE018000 w
# If the value is 0, register access is unavailable
# -> CBUS_BASE is incorrect or the clock is disabled
```

---

## 11. Mainline Driver Correction Examples

### 11.1 SoC Data Table

```c
struct meson_dmx_data {
    u32  stb_cbus_base;
    int  ts_in_count;
    int  s2p_count;
    int  afifo_count;
    bool use_amports_gate;  /* true for G12A+ */
};

static const struct meson_dmx_data gxbb_data = {
    .stb_cbus_base = 0x1600, .ts_in_count=3,
    .s2p_count=2, .afifo_count=2, .use_amports_gate=false
};
static const struct meson_dmx_data g12a_data = {
    .stb_cbus_base = 0x1800, .ts_in_count=3,
    .s2p_count=2, .afifo_count=2, .use_amports_gate=true
};
static const struct meson_dmx_data tl1_data = {
    .stb_cbus_base = 0x1800, .ts_in_count=4,
    .s2p_count=3, .afifo_count=3, .use_amports_gate=true
};
```

### 11.2 FEC_SEL Conversion Function

```c
static int s2p_id_to_fec_sel(int s2p_id)
{
    /* CRITICAL REVERSE ORDER: s2p_id=0 -> fec_sel=6 */
    return 6 - s2p_id;
}

/* Usage */
case AM_TS_SRC_S_TS0:
    fec_sel = s2p_id_to_fec_sel(0);  /* = 6 */
    break;
```

### 11.3 `STB_TOP_CONFIG` Write

```c
static void meson_dmx_stb_top_config(struct meson_dmx_priv *priv)
{
    u32 val = 0;
    int fec_s0 = 0, fec_s1 = 0;

    /* Which TS pin goes to each S2P? */
    for (int i = 0; i < priv->ts_in_count; i++) {
        if (priv->ts[i].s2p_id == 0) fec_s0 = i;
        if (priv->ts[i].s2p_id == 1) fec_s1 = i;
    }

    val |= (priv->s2p[0].invert << INVERT_S2P0_FEC_CLK);
    val |= (fec_s0 << S2P0_FEC_SERIAL_SEL);
    val |= (priv->s2p[1].invert << INVERT_S2P1_FEC_CLK);
    val |= (fec_s1 << S2P1_FEC_SERIAL_SEL);

    regmap_write(priv->regmap, STB_TOP_CONFIG, val);
}
```

### 11.4 Pinctrl DTS Example

```dts
&pinctrl {
    dvb_s_ts0_pins: dvb-s-ts0 {
        mux {
            groups = "tsin_a_sop","tsin_a_din0",
                     "tsin_a_clk","tsin_a_valid","tsin_a_fail";
            function = "tsin_a";
        };
    };
    dvb_s_ts1_pins: dvb-s-ts1 {
        mux {
            groups = "tsin_b_sop","tsin_b_din0",
                     "tsin_b_clk","tsin_b_valid","tsin_b_fail";
            function = "tsin_b";
        };
    };
};
```

---

## 12. Technical Summary and Corrections in Order of Priority

The mainline `amlogic-dvb-v2` driver requires the following corrections, in priority order:

| Priority | Problem | Effect | Correction |
|---|---|---|---|
| **1. CRITICAL** | `STB_CBUS_BASE` is incorrect | All registers written to the wrong address | Correct base selection via SoC `match_data` |
| **2. CRITICAL** | Pinctrl state names are incorrect | TS pins are never muxed | Use definitive names `'s_ts0'` and `'p_ts0'` in DTS |
| **3. CRITICAL** | `STB_TOP_CONFIG` not written | Serial mode produces no data | Implement `stb_enable()` equivalent |
| **4. CRITICAL** | FEC_SEL reverse ordering not applied | Wrong S2P source is selected | Apply formula `fec_sel = 6 - s2p_id` |
| **5. IMPORTANT** | Incomplete clock management | Hardware fails silently | Use `amports_gate` or CCF clocks as appropriate |
| **6. IMPORTANT** | `s2p_id` assignment is incorrect | S2P mapping is wrong | Use incrementing counter in probe |
| **7. IMPORTANT** | IRQ not registered correctly | Data never reaches userspace | Use `request_threaded_irq` with `IRQF_SHARED|IRQF_TRIGGER_RISING` |

---

*This analysis was produced by a complete examination of the vendor kernel 5.15 driver source code (`aml_dmx.c`: 6,108 lines; `aml_dvb.c`: 2,943 lines).*
