/*
 * ESP32 SoC and machine
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 *  Try git diff  64db39ba688ebb71b102fd0edffede79f928ed3a
 *  To see major changes compared to espressifs original qemu.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/xtensa/xtensa_memory.h"
#include "hw/misc/unimp.h"
#include "hw/irq.h"
#include "hw/core/split-irq.h"
#include "hw/i2c/i2c.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32.h"
#include "hw/misc/ssi_psram.h"
#include "hw/sd/dwc_sdmmc.h"
#include "hw/misc/servo.h"
#include "core-esp32/core-isa.h"
#include "qemu/datadir.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/cpus.h"
#include "sysemu/runstate.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "exec/exec-all.h"
#include "net/net.h"
#include "elf.h"

#define TYPE_ESP32_SOC "xtensa.esp32"
#define ESP32_SOC(obj) OBJECT_CHECK(Esp32SocState, (obj), TYPE_ESP32_SOC)

#define TYPE_ESP32_CPU XTENSA_CPU_TYPE_NAME("esp32")


enum {
    ESP32_MEMREGION_IROM,
    ESP32_MEMREGION_DROM,
    ESP32_MEMREGION_DRAM,
    ESP32_MEMREGION_IRAM,
    ESP32_MEMREGION_ICACHE0,
    ESP32_MEMREGION_ICACHE1,
    ESP32_MEMREGION_RTCSLOW,
    ESP32_MEMREGION_RTCFAST_D,
    ESP32_MEMREGION_RTCFAST_I,
    ESP32_MEMREGION_FRAMEBUF,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} esp32_memmap[] = {
    [ESP32_MEMREGION_DROM] = { 0x3ff90000, 0x10000 },
    [ESP32_MEMREGION_IROM] = { 0x40000000, 0x70000 },
    [ESP32_MEMREGION_DRAM] = { 0x3ffae000, 0x52000 },
    [ESP32_MEMREGION_IRAM] = { 0x40080000, 0x40000 },
    [ESP32_MEMREGION_ICACHE0] = { 0x40070000, 0x8000 },
    [ESP32_MEMREGION_ICACHE1] = { 0x40078000, 0x8000 },
    [ESP32_MEMREGION_RTCSLOW] = { 0x50000000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_I] = { 0x400C0000, 0x2000 },
    [ESP32_MEMREGION_RTCFAST_D] = { 0x3ff80000, 0x2000 },
};


#define ESP32_SOC_RESET_PROCPU    0x1
#define ESP32_SOC_RESET_APPCPU    0x2
#define ESP32_SOC_RESET_PERIPH    0x4
#define ESP32_SOC_RESET_DIG       (ESP32_SOC_RESET_PROCPU | ESP32_SOC_RESET_APPCPU | ESP32_SOC_RESET_PERIPH)
#define ESP32_SOC_RESET_RTC       0x8
#define ESP32_SOC_RESET_ALL       (ESP32_SOC_RESET_RTC | ESP32_SOC_RESET_DIG)

#include "bt_dbg.h"
#include "qemu/timer.h"

/* Debug level for EM access logging (0=off, 1=NVDS only, 2=annotated, 3=verbose+desc) */
static int btdm_em_debug_level = 1;

/* BLE interrupt bits for BLEINTRAWSTAT/BLEINTSTAT */
#define BLE_CSCNT_INTSTAT_BIT       (1 << 0)   /* Clock counter wrap */
#define BLE_RXINT_INTSTAT_BIT       (1 << 1)   /* RX interrupt */
#define BLE_SLPINT_INTSTAT_BIT      (1 << 2)   /* Sleep mode interrupt */
#define BLE_EVENTAPFAINT_INTSTAT_BIT (1 << 3)  /* Event apfa interrupt */
#define BLE_FINETGTINT_INTSTAT_BIT  (1 << 4)   /* Fine timer target interrupt */
#define BLE_GROSSTGTINT_INTSTAT_BIT (1 << 5)   /* Gross timer target interrupt */
#define BLE_ERRORINT_INTSTAT_BIT    (1 << 6)   /* Error interrupt */
#define BLE_CRYPTINT_INTSTAT_BIT    (1 << 7)   /* Encryption interrupt */
#define BLE_EVENTINT_INTSTAT_BIT    (1 << 8)   /* Event done interrupt */
#define BLE_SWINT_INTSTAT_BIT       (1 << 9)   /* Software interrupt */

/* BLE timer state */
static struct {
    QEMUTimer *timer;
    qemu_irq irq;
    bool enabled;
    uint64_t period_ns;
} ble_timer_state;

/* Forward declarations for BLE timer functions */
static void ble_timer_update(void);

/* Intercepts the Data Plane (Exchange Memory) */
static uint64_t btdm_em_read(void *opaque, hwaddr addr, unsigned size) {
    uint8_t *em_ptr = (uint8_t *)opaque; 
    uint32_t val = 0;
    memcpy(&val, em_ptr + addr, size);
    
    if (btdm_em_debug_level >= 2) {
        fprintf(stderr, "EM_RD [%s] 0x%04" HWADDR_PRIx " (%d) -> 0x%0*x%s\n",
                 EM_REGION_NAME(addr), addr, size, size*2, val, em_field_comment(addr, val));
    } else if (btdm_em_debug_level >= 1 && addr < 0x10) {
        /* Always log NVDS magic reads */
        fprintf(stderr, "EM_RD NVDS 0x%04" HWADDR_PRIx " -> 0x%08x\n", addr, val);
    }
    
    /* Print full RX descriptor when reading RXSTAT (first field) */
    if (btdm_em_debug_level >= 3) {
        if (addr >= EM_BT_RXDESC_OFFSET && addr < EM_BT_TXDESC_OFFSET) {
            uint32_t desc_idx = (addr - EM_BT_RXDESC_OFFSET) / sizeof(em_bt_rxdesc_t);
            uint32_t desc_off = (addr - EM_BT_RXDESC_OFFSET) % sizeof(em_bt_rxdesc_t);
            if (desc_off == 0) {
                em_bt_rxdesc_t *desc = (em_bt_rxdesc_t *)(em_ptr + EM_BT_RXDESC_OFFSET + desc_idx * sizeof(em_bt_rxdesc_t));
                fprintf(stderr, "BT_RXDESC[%d] @ EM+0x%04x:\n", desc_idx, (uint32_t)(EM_BT_RXDESC_OFFSET + desc_idx * sizeof(em_bt_rxdesc_t)));
                fprintf(stderr, "  rxstat=0x%04x bt_hdr=0x%04x acl_hdr=0x%04x data_ptr=0x%04x\n",
                        desc->rxstat, desc->bt_header, desc->acl_header, desc->data_ptr);
            }
        } else if (addr >= EM_BT_TXDESC_OFFSET && addr < EM_BLE_RXDESC_OFFSET) {
            uint32_t desc_idx = (addr - EM_BT_TXDESC_OFFSET) / sizeof(em_bt_txdesc_t);
            uint32_t desc_off = (addr - EM_BT_TXDESC_OFFSET) % sizeof(em_bt_txdesc_t);
            if (desc_off == 0) {
                em_bt_txdesc_t *desc = (em_bt_txdesc_t *)(em_ptr + EM_BT_TXDESC_OFFSET + desc_idx * sizeof(em_bt_txdesc_t));
                fprintf(stderr, "BT_TXDESC[%d] @ EM+0x%04x:\n", desc_idx, (uint32_t)(EM_BT_TXDESC_OFFSET + desc_idx * sizeof(em_bt_txdesc_t)));
                fprintf(stderr, "  txctrl=0x%04x bt_hdr=0x%04x acl_hdr=0x%04x data_ptr=0x%04x\n",
                        desc->txctrl, desc->bt_header, desc->acl_header, desc->txdataptr);
            }
        }
    }
    
    return val;
}

static void btdm_em_write(void *opaque, hwaddr addr, uint64_t val, unsigned size) {
    uint8_t *em_ptr = (uint8_t *)opaque;
    memcpy(em_ptr + addr, &val, size);
    
    if (btdm_em_debug_level >= 2) {
        fprintf(stderr, "EM_WR [%s] 0x%04" HWADDR_PRIx " (%d) <- 0x%0*" PRIx64 "%s\n",
                 EM_REGION_NAME(addr), addr, size, size*2, val, em_field_comment(addr, (uint32_t)val));
    }
    
    /* Log descriptor writes for debugging */
    if (btdm_em_debug_level >= 3) {
        /* Check if this could be an RX descriptor */
        if (addr >= EM_BT_RXDESC_OFFSET && addr < EM_BT_TXDESC_OFFSET) {
            uint32_t desc_idx = (addr - EM_BT_RXDESC_OFFSET) / sizeof(em_bt_rxdesc_t);
            fprintf(stderr, "  -> BT_RXDESC[%d] write\n", desc_idx);
        } else if (addr >= EM_BT_TXDESC_OFFSET && addr < EM_BLE_RXDESC_OFFSET) {
            uint32_t desc_idx = (addr - EM_BT_TXDESC_OFFSET) / sizeof(em_bt_txdesc_t);
            fprintf(stderr, "  -> BT_TXDESC[%d] write\n", desc_idx);
        } else if (addr >= EM_BLE_RXDESC_OFFSET && addr < EM_BLE_TXDESC_OFFSET) {
            fprintf(stderr, "  -> BLE_RXDESC region write\n");
        } else if (addr >= EM_BLE_TXDESC_OFFSET && addr < EM_BLE_CS_OFFSET) {
            fprintf(stderr, "  -> BLE_TXDESC region write\n");
        }
    }
}

static const MemoryRegionOps btdm_em_ops = {
    .read = btdm_em_read,
    .write = btdm_em_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* PHY/Baseband / BTDM BLE registers (0x3ff71000) read/write handlers 
 * This region contains the RivieraWaves BLE controller registers.
 * Offsets are relative to 0x3ff71000 (DR_REG_PHY_BASE).
 * The BLE section starts at offset 0x200.
 */

/* Simple monotonic clock counter for BTDM - increments on each read */
static uint32_t btdm_clock_counter = 0;

/* BTDM register state - covers both BR/EDR (0x000-0x1FF) and BLE (0x200+) sections */
static struct {
    /* BR/EDR Section (offset 0x000-0x1FF) */
    uint32_t btcntl;            /* 0x000 */
    uint32_t btversion;         /* 0x004 */
    uint32_t btintcntl;         /* 0x00C */
    uint32_t btintstat;         /* 0x010 */
    uint32_t btintrawstat;      /* 0x014 */
    uint32_t btintack;          /* 0x018 */
    uint32_t clk_latch;         /* 0x01C - Clock latch register (bit31=sample, bits0-27=clock) */
    uint32_t reg_028;           /* 0x028 */
    uint32_t reg_02c;           /* 0x02C */
    uint32_t reg_030;           /* 0x030 */
    uint32_t reg_034;           /* 0x034 */
    uint32_t reg_038;           /* 0x038 */
    uint32_t reg_03c;           /* 0x03C */
    uint32_t reg_040;           /* 0x040 */
    uint32_t reg_044;           /* 0x044 */
    uint32_t reg_050;           /* 0x050 */
    uint32_t reg_060;           /* 0x060 */
    uint32_t reg_064;           /* 0x064 */
    uint32_t reg_070;           /* 0x070 */
    uint32_t reg_074;           /* 0x074 */
    uint32_t reg_080;           /* 0x080 */
    uint32_t reg_098;           /* 0x098 */
    uint32_t reg_0a4;           /* 0x0A4 */
    uint32_t reg_0b0;           /* 0x0B0 */
    uint32_t reg_0b4;           /* 0x0B4 */
    uint32_t reg_0bc;           /* 0x0BC */
    uint32_t reg_0c0;           /* 0x0C0 */
    uint32_t reg_0c4;           /* 0x0C4 */
    uint32_t reg_0d0;           /* 0x0D0 - WiFi/BT coexistence control */
    uint32_t reg_140;           /* 0x140 */
    uint32_t reg_160;           /* 0x160 */
    uint32_t reg_180;           /* 0x180 */
    uint32_t reg_1a0;           /* 0x1A0 */
    uint32_t reg_1b0;           /* 0x1B0 */
    uint32_t reg_1e0;           /* 0x1E0 */
    /* BLE Section (offset 0x200+) */
    uint32_t blecntl;           /* 0x200 */
    uint32_t bleversion;        /* 0x204 */
    uint32_t bleconf;           /* 0x208 */
    uint32_t bleintcntl;        /* 0x20C */
    uint32_t bleintstat;        /* 0x210 */
    uint32_t bleintrawstat;     /* 0x214 */
    uint32_t bleintack;         /* 0x218 */
    uint32_t blebasetimecnt;    /* 0x21C */
    uint32_t blefinetimecnt;    /* 0x220 */
    uint32_t blebdaddrl;        /* 0x224 */
    uint32_t blebdaddru;        /* 0x228 */
    uint32_t blecurrentrxdescptr; /* 0x22C */
    uint32_t blediagcntl;       /* 0x250 */
    uint32_t blediagstat;       /* 0x254 */
    uint32_t bleerrortypestat;  /* 0x260 */
    uint32_t bleradiocntl0;     /* 0x270 */
    uint32_t bleradiocntl1;     /* 0x274 */
    uint32_t bleradiopwrupdn;   /* 0x280 */
    uint32_t bleadvchmap;       /* 0x290 */
    uint32_t bleadvtim;         /* 0x2A0 */
    uint32_t blewlpubaddrptr;   /* 0x2B0 */
    uint32_t blewlprivaddrptr;  /* 0x2B4 */
    uint32_t blewlnbdev;        /* 0x2B8 */
    uint32_t bleaescntl;        /* 0x2C0 */
    uint32_t bleaeskey[4];      /* 0x2C4-0x2D0 */
    uint32_t bleaesptr;         /* 0x2D4 */
    uint32_t blerftestcntl;     /* 0x2E0 */
    uint32_t blerftesttxstat;   /* 0x2E4 */
    uint32_t blerftestrxstat;   /* 0x2E8 */
    uint32_t bletimgencntl;     /* 0x2F0 */
    uint32_t blecoexifcntl0;    /* 0x300 */
    uint32_t bleralptr;         /* 0x320 */
    uint32_t bleralnbdev;       /* 0x324 */
    /* RivieraWaves Interrupt Controller (offset 0x350+) */
    uint32_t rwintcntl;         /* 0x350 - RW interrupt control */
    uint32_t rwintstat;         /* 0x354 - RW interrupt status (returns IRQ number) */
    uint32_t rwintrawstat;      /* 0x358 - RW raw interrupt status */
    uint32_t rwintack;          /* 0x35C - RW interrupt acknowledge */
    uint32_t rwintset;          /* 0x360 - RW interrupt set */
    uint32_t rwintclr;          /* 0x364 - RW interrupt clear */
    uint32_t reg_390;           /* 0x390 */
    bool initialized;
} btdm_state;

static void btdm_state_init(void)
{
    if (btdm_state.initialized) {
        return;
    }
    /* Initialize BR/EDR section with values from real hardware */
    btdm_state.btcntl = 0x0000710c;
    btdm_state.btversion = 0x08000b00;
    btdm_state.btintcntl = 0x0003c802;
    btdm_state.btintstat = 0x00000000;
    btdm_state.btintrawstat = 0x00000001;
    btdm_state.btintack = 0x00000000;
    btdm_state.clk_latch = 0x00000000;
    btdm_state.reg_028 = 0x00000000;
    btdm_state.reg_02c = 0x00000000;
    btdm_state.reg_030 = 0x00000000;
    btdm_state.reg_034 = 0x00000000;
    btdm_state.reg_038 = 0x00000000;
    btdm_state.reg_03c = 0x00000000;
    btdm_state.reg_040 = 0x00000000;
    btdm_state.reg_044 = 0x00000000;
    btdm_state.reg_050 = 0x00000000;
    btdm_state.reg_060 = 0x00000000;
    btdm_state.reg_064 = 0x00000000;
    btdm_state.reg_070 = 0x00000000;
    btdm_state.reg_074 = 0x00000000;
    btdm_state.reg_080 = 0x00000000;
    btdm_state.reg_098 = 0x00000000;
    btdm_state.reg_0a4 = 0x00000000;
    btdm_state.reg_0b0 = 0x00000000;
    btdm_state.reg_0b4 = 0x00000000;
    btdm_state.reg_0bc = 0x00000000;
    btdm_state.reg_0c0 = 0x00000000;
    btdm_state.reg_0c4 = 0x00000000;
    btdm_state.reg_0d0 = 0x00000000;
    btdm_state.reg_140 = 0x00000000;
    btdm_state.reg_160 = 0x00000000;
    btdm_state.reg_180 = 0x00000000;
    btdm_state.reg_1a0 = 0x00000000;
    btdm_state.reg_1b0 = 0x00000000;
    btdm_state.reg_1e0 = 0x00000000;
    /* Initialize BLE section with values from real hardware */
    btdm_state.blecntl = 0x000003e0;
    btdm_state.bleversion = 0x08000900;  /* TYP=8, REL=0, UPG=9, BUILD=0 */
    btdm_state.bleconf = 0x00000000;
    btdm_state.bleintcntl = 0x00000000;
    btdm_state.bleintstat = 0x00000000;
    btdm_state.bleintrawstat = 0x00000001;
    btdm_state.bleintack = 0x00000000;
    btdm_state.blebasetimecnt = 0x00000000;
    btdm_state.blefinetimecnt = 0x00000000;
    btdm_state.blebdaddrl = 0x00000000;
    btdm_state.blebdaddru = 0x00000000;
    btdm_state.blecurrentrxdescptr = 0x00000000;
    btdm_state.blediagcntl = 0x00000000;
    btdm_state.blediagstat = 0x00000000;
    btdm_state.bleerrortypestat = 0x00000000;
    btdm_state.bleradiocntl0 = 0x00000002;
    btdm_state.bleradiocntl1 = 0x00020000;
    btdm_state.bleradiopwrupdn = 0x00d203d2;
    btdm_state.bleadvchmap = 0x00000007;  /* All 3 advertising channels */
    btdm_state.bleadvtim = 0x00000000;
    btdm_state.blewlpubaddrptr = 0x00000000;
    btdm_state.blewlprivaddrptr = 0x00000000;
    btdm_state.blewlnbdev = 0x00000000;
    btdm_state.bleaescntl = 0x00000000;
    btdm_state.bleaesptr = 0x00000000;
    btdm_state.blerftestcntl = 0x00000000;
    btdm_state.blerftesttxstat = 0x00000000;
    btdm_state.blerftestrxstat = 0x00000000;
    btdm_state.bletimgencntl = 0x81be00d2;
    btdm_state.blecoexifcntl0 = 0x00000000;
    btdm_state.bleralptr = 0x00000000;
    btdm_state.bleralnbdev = 0x00000000;
    /* RW Interrupt Controller - return proper IRQ numbers */
    btdm_state.rwintcntl = 0x00000000;
    btdm_state.rwintstat = 0x00000004;   /* ETS_BT_BB_INTR_SOURCE = 4 */
    btdm_state.rwintrawstat = 0x00000000;
    btdm_state.rwintack = 0x00000000;
    btdm_state.rwintset = 0x00000000;
    btdm_state.rwintclr = 0x00000000;
    btdm_state.reg_390 = 0x00000000;
    btdm_state.initialized = true;
}

static uint64_t phy_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t val = 0;

    btdm_state_init();

    switch (addr) {
    /* BR/EDR Section (offset 0x000-0x1FF) */
    case 0x000:
        /* BTCNTL - bits 31 and 30 auto-clear to indicate reset/operation complete */
        val = btdm_state.btcntl & 0x3FFFFFFF;  /* Always read with bits 31,30 cleared */
        break;
    case 0x004: val = btdm_state.btversion; break;
    case 0x00C: val = btdm_state.btintcntl; break;
    case 0x010: val = btdm_state.btintstat; break;
    case 0x014: val = btdm_state.btintrawstat; break;
    case 0x018: val = btdm_state.btintack; break;
    case 0x01C:
        /* Clock latch register - return latched clock value with bit 31 cleared */
        btdm_clock_counter += 312;  /* Simulate time passing (~312.5us per BT slot) */
        val = btdm_state.clk_latch & 0x0FFFFFFF;  /* Bit 31 always reads as 0 (sample complete) */
        break;
    case 0x028: val = btdm_state.reg_028; break;
    case 0x02C: val = btdm_state.reg_02c; break;
    case 0x030: val = btdm_state.reg_030; break;
    case 0x034: val = btdm_state.reg_034; break;
    case 0x038: val = btdm_state.reg_038; break;
    case 0x03C: val = btdm_state.reg_03c; break;
    case 0x040: val = btdm_state.reg_040; break;
    case 0x044: val = btdm_state.reg_044; break;
    case 0x050: val = btdm_state.reg_050; break;
    case 0x060: val = btdm_state.reg_060; break;
    case 0x064: val = btdm_state.reg_064; break;
    case 0x070: val = btdm_state.reg_070; break;
    case 0x074: val = btdm_state.reg_074; break;
    case 0x080: val = btdm_state.reg_080; break;
    case 0x098: val = btdm_state.reg_098; break;
    case 0x0A4: val = btdm_state.reg_0a4; break;
    case 0x0B0: val = btdm_state.reg_0b0; break;
    case 0x0B4: val = btdm_state.reg_0b4; break;
    case 0x0BC: val = btdm_state.reg_0bc; break;
    case 0x0C0: val = btdm_state.reg_0c0; break;
    case 0x0C4: val = btdm_state.reg_0c4; break;
    case 0x0D0: val = btdm_state.reg_0d0; break;
    case 0x140: val = btdm_state.reg_140; break;
    case 0x160: val = btdm_state.reg_160; break;
    case 0x180: val = btdm_state.reg_180; break;
    case 0x1A0: val = btdm_state.reg_1a0; break;
    case 0x1B0: val = btdm_state.reg_1b0; break;
    case 0x1E0: val = btdm_state.reg_1e0; break;
    /* BLE Section (offset 0x200+) */
    case 0x200: val = btdm_state.blecntl; break;
    case 0x204: val = btdm_state.bleversion; break;
    case 0x208: val = btdm_state.bleconf; break;
    case 0x20C: val = btdm_state.bleintcntl; break;
    case 0x210: val = btdm_state.bleintstat; break;
    case 0x214: val = btdm_state.bleintrawstat; break;
    case 0x218: val = btdm_state.bleintack; break;
    case 0x21C:
        /* BLEBASETIMECNT - bit 31 auto-clears to indicate operation complete */
        val = btdm_state.blebasetimecnt & 0x7FFFFFFF;
        break;
    case 0x220: val = btdm_state.blefinetimecnt; break;
    case 0x224: val = btdm_state.blebdaddrl; break;
    case 0x228: val = btdm_state.blebdaddru; break;
    case 0x22C: val = btdm_state.blecurrentrxdescptr; break;
    case 0x250: val = btdm_state.blediagcntl; break;
    case 0x254: val = btdm_state.blediagstat; break;
    case 0x260: val = btdm_state.bleerrortypestat; break;
    case 0x270: val = btdm_state.bleradiocntl0; break;
    case 0x274: val = btdm_state.bleradiocntl1; break;
    case 0x280: val = btdm_state.bleradiopwrupdn; break;
    case 0x290: val = btdm_state.bleadvchmap; break;
    case 0x2A0: val = btdm_state.bleadvtim; break;
    case 0x2B0: val = btdm_state.blewlpubaddrptr; break;
    case 0x2B4: val = btdm_state.blewlprivaddrptr; break;
    case 0x2B8: val = btdm_state.blewlnbdev; break;
    case 0x2C0: val = btdm_state.bleaescntl; break;
    case 0x2C4: val = btdm_state.bleaeskey[0]; break;
    case 0x2C8: val = btdm_state.bleaeskey[1]; break;
    case 0x2CC: val = btdm_state.bleaeskey[2]; break;
    case 0x2D0: val = btdm_state.bleaeskey[3]; break;
    case 0x2D4: val = btdm_state.bleaesptr; break;
    case 0x2E0: val = btdm_state.blerftestcntl; break;
    case 0x2E4: val = btdm_state.blerftesttxstat; break;
    case 0x2E8: val = btdm_state.blerftestrxstat; break;
    case 0x2F0: val = btdm_state.bletimgencntl; break;
    case 0x300: val = btdm_state.blecoexifcntl0; break;
    case 0x320: val = btdm_state.bleralptr; break;
    case 0x324: val = btdm_state.bleralnbdev; break;
    /* RivieraWaves Interrupt Controller */
    case 0x350: val = btdm_state.rwintcntl; break;
    case 0x354: val = btdm_state.rwintstat; break;  /* Returns IRQ number (4) */
    case 0x358: val = btdm_state.rwintrawstat; break;
    case 0x35C: val = btdm_state.rwintack; break;
    case 0x360: val = btdm_state.rwintset; break;
    case 0x364: val = btdm_state.rwintclr; break;
    case 0x390: val = btdm_state.reg_390; break;
    default:
        /* Return 0 for unknown registers instead of 0xffffffff */
        val = 0;
        qemu_log_mask(LOG_UNIMP, "BTDM READ: unhandled offset 0x%" HWADDR_PRIx
                      " (size %u)\n", addr, size);
        break;
    }

    return val;
}

static void phy_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    btdm_state_init();

    switch (addr) {
    /* BR/EDR Section (offset 0x000-0x1FF) */
    case 0x000: 
        /* BTCNTL - store value, bits 31/30 will auto-clear on read */
        btdm_state.btcntl = val;
        if (val & (1 << 31)) {
            qemu_log_mask(LOG_GUEST_ERROR, "BTDM: Guest set BTCNTL bit 31 (soft reset)\n");
        }
        if (val & (1 << 30)) {
            qemu_log_mask(LOG_GUEST_ERROR, "BTDM: Guest set BTCNTL bit 30\n");
        }
        break;
    case 0x004: btdm_state.btversion = val; break;
    case 0x00C: btdm_state.btintcntl = val; break;
    case 0x010: btdm_state.btintstat = val; break;
    case 0x014: btdm_state.btintrawstat = val; break;
    case 0x018: btdm_state.btintack = val; break;
    case 0x01C:
        /* Clock latch register - when bit 31 is set, latch current clock and clear bit 31 */
        if (val & 0x80000000) {
            /* Sample request: latch current clock value, bit 31 cleared immediately */
            btdm_state.clk_latch = (btdm_clock_counter & 0x07FFFFFF);  /* 27-bit clock value */
        } else {
            btdm_state.clk_latch = val;
        }
        break;
    case 0x028: btdm_state.reg_028 = val; break;
    case 0x02C: btdm_state.reg_02c = val; break;
    case 0x030: btdm_state.reg_030 = val; break;
    case 0x034: btdm_state.reg_034 = val; break;
    case 0x038: btdm_state.reg_038 = val; break;
    case 0x03C: btdm_state.reg_03c = val; break;
    case 0x040: btdm_state.reg_040 = val; break;
    case 0x044: btdm_state.reg_044 = val; break;
    case 0x050: btdm_state.reg_050 = val; break;
    case 0x060: btdm_state.reg_060 = val; break;
    case 0x064: btdm_state.reg_064 = val; break;
    case 0x070: btdm_state.reg_070 = val; break;
    case 0x074: btdm_state.reg_074 = val; break;
    case 0x080: btdm_state.reg_080 = val; break;
    case 0x098: btdm_state.reg_098 = val; break;
    case 0x0A4: btdm_state.reg_0a4 = val; break;
    case 0x0B0: btdm_state.reg_0b0 = val; break;
    case 0x0B4: btdm_state.reg_0b4 = val; break;
    case 0x0BC: btdm_state.reg_0bc = val; break;
    case 0x0C0: btdm_state.reg_0c0 = val; break;
    case 0x0C4: btdm_state.reg_0c4 = val; break;
    case 0x0D0: btdm_state.reg_0d0 = val; break;
    case 0x140: btdm_state.reg_140 = val; break;
    case 0x160: btdm_state.reg_160 = val; break;
    case 0x180: btdm_state.reg_180 = val; break;
    case 0x1A0: btdm_state.reg_1a0 = val; break;
    case 0x1B0: btdm_state.reg_1b0 = val; break;
    case 0x1E0: btdm_state.reg_1e0 = val; break;
    /* BLE Section (offset 0x200+) */
    case 0x200: 
        btdm_state.blecntl = val;
        if (val & (1 << 31)) {
            qemu_log("BTDM: Guest requested BLE Master Soft Reset\n");
            /* Auto-clear bit 31 */
            btdm_state.blecntl &= ~(1 << 31);
        }
        /* Update BLE timer based on enable bit */
        ble_timer_update();
        break;
    case 0x204: btdm_state.bleversion = val; break;
    case 0x208: btdm_state.bleconf = val; break;
    case 0x20C: btdm_state.bleintcntl = val; break;
    case 0x210: btdm_state.bleintstat = val; break;
    case 0x214: btdm_state.bleintrawstat = val; break;
    case 0x218:
        /* BLEINTACK - writing 1 clears the corresponding interrupt bits */
        btdm_state.bleintack = val;
        btdm_state.bleintstat &= ~val;
        btdm_state.bleintrawstat &= ~val;
        /* Lower IRQ if no more interrupts pending */
        if (ble_timer_state.irq && btdm_state.bleintstat == 0) {
            qemu_irq_lower(ble_timer_state.irq);
        }
        break;
    case 0x21C: btdm_state.blebasetimecnt = val; break;
    case 0x220: btdm_state.blefinetimecnt = val; break;
    case 0x224: btdm_state.blebdaddrl = val; break;
    case 0x228: btdm_state.blebdaddru = val; break;
    case 0x22C: btdm_state.blecurrentrxdescptr = val; break;
    case 0x250: btdm_state.blediagcntl = val; break;
    case 0x254: btdm_state.blediagstat = val; break;
    case 0x260: btdm_state.bleerrortypestat = val; break;
    case 0x270: btdm_state.bleradiocntl0 = val; break;
    case 0x274: btdm_state.bleradiocntl1 = val; break;
    case 0x280: btdm_state.bleradiopwrupdn = val; break;
    case 0x290: btdm_state.bleadvchmap = val; break;
    case 0x2A0: btdm_state.bleadvtim = val; break;
    case 0x2B0: btdm_state.blewlpubaddrptr = val; break;
    case 0x2B4: btdm_state.blewlprivaddrptr = val; break;
    case 0x2B8: btdm_state.blewlnbdev = val; break;
    case 0x2C0: btdm_state.bleaescntl = val; break;
    case 0x2C4: btdm_state.bleaeskey[0] = val; break;
    case 0x2C8: btdm_state.bleaeskey[1] = val; break;
    case 0x2CC: btdm_state.bleaeskey[2] = val; break;
    case 0x2D0: btdm_state.bleaeskey[3] = val; break;
    case 0x2D4: btdm_state.bleaesptr = val; break;
    case 0x2E0: btdm_state.blerftestcntl = val; break;
    case 0x2E4: btdm_state.blerftesttxstat = val; break;
    case 0x2E8: btdm_state.blerftestrxstat = val; break;
    case 0x2F0: btdm_state.bletimgencntl = val; break;
    case 0x300: btdm_state.blecoexifcntl0 = val; break;
    case 0x320: btdm_state.bleralptr = val; break;
    case 0x324: btdm_state.bleralnbdev = val; break;
    /* RivieraWaves Interrupt Controller */
    case 0x350: btdm_state.rwintcntl = val; break;
    case 0x354: btdm_state.rwintstat = val; break;
    case 0x358: btdm_state.rwintrawstat = val; break;
    case 0x35C: btdm_state.rwintack = val; break;
    case 0x360: btdm_state.rwintset = val; break;
    case 0x364: btdm_state.rwintclr = val; break;
    case 0x390: btdm_state.reg_390 = val; break;
    default:
        qemu_log_mask(LOG_UNIMP, "BTDM WRITE: unhandled offset 0x%" HWADDR_PRIx
                      " (size %u) <- 0x%08" PRIx64 "\n", addr, size, val);
        break;
    }
}

static const MemoryRegionOps phy_mmio_ops = {
    .read = phy_mmio_read,
    .write = phy_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* BLE timer callback - generates periodic interrupts for the BLE scheduler */
static void ble_timer_cb(void *opaque)
{
    if (!ble_timer_state.enabled) {
        return;
    }

    /* Increment the base time counter (simulates 625µs slots) */
    btdm_clock_counter += 625;
    btdm_state.blebasetimecnt = (btdm_state.blebasetimecnt + 1) & 0x07FFFFFF;
    btdm_state.blefinetimecnt = (btdm_state.blefinetimecnt + 312) & 0x3FF;

    /* Set CSCNT interrupt bit to wake up scheduler */
    btdm_state.bleintrawstat |= BLE_CSCNT_INTSTAT_BIT;
    
    /* Also set FINETGTINT periodically (every 4 slots) for timer target events */
    if ((btdm_state.blebasetimecnt & 0x3) == 0) {
        btdm_state.bleintrawstat |= BLE_FINETGTINT_INTSTAT_BIT;
    }
    
    /* Set EVENTINT periodically (every 8 slots) to complete advertising/scanning events */
    if ((btdm_state.blebasetimecnt & 0x7) == 0) {
        btdm_state.bleintrawstat |= BLE_EVENTINT_INTSTAT_BIT;
    }
    
    /* Check which enabled interrupts should fire */
    uint32_t pending = btdm_state.bleintrawstat & btdm_state.bleintcntl;
    if (pending) {
        btdm_state.bleintstat |= pending;
        if (ble_timer_state.irq) {
            qemu_irq_raise(ble_timer_state.irq);
        }
    }

    /* Reschedule the timer */
    timer_mod(ble_timer_state.timer, 
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ble_timer_state.period_ns);
}

/* Initialize BLE timer - called when BLE is enabled */
static void ble_timer_init(qemu_irq irq)
{
    if (ble_timer_state.timer) {
        return;  /* Already initialized */
    }
    
    ble_timer_state.irq = irq;
    ble_timer_state.period_ns = 625 * 1000;  /* 625µs = BLE slot time */
    ble_timer_state.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ble_timer_cb, NULL);
    ble_timer_state.enabled = false;
}

/* Start/stop BLE timer based on BLECNTL register */
static void ble_timer_update(void)
{
    bool should_run = (btdm_state.blecntl & 0x1);  /* Bit 0 = RWBLE_EN */
    
    if (should_run && !ble_timer_state.enabled) {
        ble_timer_state.enabled = true;
        timer_mod(ble_timer_state.timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ble_timer_state.period_ns);
        qemu_log("BLE Timer: Started (period=%"PRId64"ns)\n", ble_timer_state.period_ns);
    } else if (!should_run && ble_timer_state.enabled) {
        ble_timer_state.enabled = false;
        timer_del(ble_timer_state.timer);
        qemu_log("BLE Timer: Stopped\n");
    }
}


static void remove_cpu_watchpoints(XtensaCPU* xcs)
{
    for (int i = 0; i < MAX_NDBREAK; ++i) {
        if (xcs->env.cpu_watchpoint[i]) {
            cpu_watchpoint_remove_by_ref(CPU(xcs), xcs->env.cpu_watchpoint[i]);
            xcs->env.cpu_watchpoint[i] = NULL;
        }
    }
}

static void esp32_full_reset(void *opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            s->rtc_cntl.reset_cause[i] = ESP32_SW_CPU_RESET;
        }
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}
static void esp32_dig_reset(void *opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        esp32_dport_clear_ill_trap_state(&s->dport);
        s->requested_reset = ESP32_SOC_RESET_DIG;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->rtc_cntl.reset_cause[n] = ESP32_SW_CPU_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32_timg_cpu_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
//    printf("esp32_timg_cpu_reset\n");
    if (level) {
        s->requested_reset = (n == 0) ? ESP32_SOC_RESET_PROCPU : ESP32_SOC_RESET_APPCPU;
        /* Use different cause for APP CPU so that its reset doesn't cause QEMU to exit,
         * when -no-reboot option is given.
         */
        ShutdownCause cause = (n == 0) ? SHUTDOWN_CAUSE_GUEST_RESET : SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->rtc_cntl.reset_cause[n] = ESP32_TGWDT_CPU_RESET;
        qemu_system_reset_request(cause);
    }
}

static void esp32_rtc_reset(void* opaque, int n, int level)
{
 //   printf("esp32_rtc_reset %d\n",level);
    Esp32SocState *s = ESP32_SOC(opaque);

    if(level && !FIELD_EX32(s->rtc_cntl.sdio_conf,RTC_CNTL_SDIO_CONF,VREG_PD_EN) /*s->rtc_cntl.mem_conf==0*/) {
        s->rtc_cntl.int_raw=1;
        
   //     s->requested_reset = ESP32_SOC_RESET_RTC;
   //     qemu_system_reset_request(SHUTDOWN_CAUSE_NONE);
        //s->gpio.rtcio_regs[0x20]=0;

        return;
    }

    if (level) {
    //    esp32_dport_clear_ill_trap_state(&s->dport);
      //  ShutdownCause cause = SHUTDOWN_CAUSE_SUBSYSTEM_RESET;
        s->requested_reset = ESP32_SOC_RESET_ALL; // ESP32_SOC_RESET_RTC | ESP32_SOC_RESET_APPCPU | ESP32_SOC_RESET_PROCPU;
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            s->rtc_cntl.reset_cause[i] = ESP32_DEEPSLEEP_RESET;
      //      s->rtc_cntl.stat_vector_sel[i] = true;
        }
    //    s->timg[0].wdt_en_at_reset=false;
//        s->rtc_cntl.reset_cause[0] = ESP32_DEEPSLEEP_RESET;
      //  for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
      //      s->rtc_cntl.reset_cause[i] = ESP32_DEEPSLEEP_RESET;
      //  }
      //  s->rtc_cntl.reset_cause[0] = ESP32_EXT_CPU_RESET;//ESP32_DEEPSLEEP_RESET;
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        s->rtc_cntl.sdio_conf=FIELD_DP32(s->rtc_cntl.sdio_conf,RTC_CNTL_SDIO_CONF,VREG_PD_EN,0);
    }
}

static void esp32_timg_sys_reset(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (level) {
        esp32_dport_clear_ill_trap_state(&s->dport);
        s->requested_reset = ESP32_SOC_RESET_DIG;
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            s->rtc_cntl.reset_cause[i] = ESP32_TG0WDT_SYS_RESET + n;
        }
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void esp32_soc_reset(DeviceState *dev)
{ 
    Esp32SocState *s = ESP32_SOC(dev);

    qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
    uint32_t strap_mode = s->gpio.strap_mode;
 //   printf("soc_reset %d %d\n",s->requested_reset,s->rtc_cntl.reset_cause[0]);

    bool flash_boot_mode = ((strap_mode & 0x10) || (strap_mode & 0x1f) == 0x0c);
    qemu_set_irq(qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_DL_MODE_GPIO, 0), !flash_boot_mode);

    if (s->requested_reset == 0) {
        s->requested_reset = ESP32_SOC_RESET_ALL;
    }
    if (s->requested_reset & ESP32_SOC_RESET_RTC) {
        device_cold_reset(DEVICE(&s->rtc_cntl));
    }
    if (s->requested_reset & ESP32_SOC_RESET_PERIPH) {
        device_cold_reset(DEVICE(&s->dport));
        device_cold_reset(DEVICE(&s->intmatrix));
        device_cold_reset(DEVICE(&s->aes));
        device_cold_reset(DEVICE(&s->sha));
        device_cold_reset(DEVICE(&s->rsa));
        device_cold_reset(DEVICE(&s->gpio));
        for (int i = 0; i < ESP32_UART_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->uart[i]));
        }
        for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->frc_timer[i]));
        }
        for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->timg[i]));
        }
        s->timg[0].flash_boot_mode = flash_boot_mode;
        for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
            device_cold_reset(DEVICE(&s->spi[i]));
        }
        for (int i = 0; i < ESP32_I2C_COUNT; i++) {
            device_cold_reset(DEVICE(&s->i2c[i]));
        }
//        device_cold_reset(DEVICE(&s->twai));
        device_cold_reset(DEVICE(&s->efuse));
        if (s->eth) {
            device_cold_reset(s->eth);
        }
        if (s->wifi_dev) {
            device_cold_reset(s->wifi_dev);
        }
        device_cold_reset(DEVICE(&s->rmt));
        device_cold_reset(DEVICE(&s->ledc));
    }
    if (s->requested_reset & ESP32_SOC_RESET_PROCPU) {
        xtensa_select_static_vectors(&s->cpu[0].env, s->rtc_cntl.stat_vector_sel[0]);
        remove_cpu_watchpoints(&s->cpu[0]);
        cpu_reset(CPU(&s->cpu[0]));
    }
    if (s->requested_reset & ESP32_SOC_RESET_APPCPU) {
        xtensa_select_static_vectors(&s->cpu[1].env, s->rtc_cntl.stat_vector_sel[1]);
        remove_cpu_watchpoints(&s->cpu[1]);
        cpu_reset(CPU(&s->cpu[1]));
    }
    s->requested_reset = 0;
}

static void esp32_cpu_stall(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);

    bool stall;
    if (n == 0) {
        stall = s->rtc_cntl.cpu_stall_state[0];
    } else {
        stall = s->rtc_cntl.cpu_stall_state[1] || s->dport.appcpu_stall_state || (!s->dport.appcpu_clkgate_state);
    }

    if (stall != s->cpu[n].env.runstall) {
        xtensa_runstall(&s->cpu[n].env, stall);
    }
}

static void esp32_clk_update(void* opaque, int n, int level)
{
    Esp32SocState *s = ESP32_SOC(opaque);
    if (!level) {
        return;
    }

    /* APB clock */
    uint32_t apb_clk_freq, cpu_clk_freq;
    if (s->rtc_cntl.soc_clk == ESP32_SOC_CLK_PLL) {
        const uint32_t cpu_clk_mul[] = {1, 2, 3};
        apb_clk_freq = s->rtc_cntl.pll_apb_freq;
        cpu_clk_freq = cpu_clk_mul[s->dport.cpuperiod_sel] * apb_clk_freq;
    } else {
        apb_clk_freq = s->rtc_cntl.xtal_apb_freq;
        cpu_clk_freq = apb_clk_freq;
    }
    qdev_prop_set_int32(DEVICE(&s->frc_timer), "apb_freq", apb_clk_freq);
    qdev_prop_set_int32(DEVICE(&s->timg[0]), "apb_freq", apb_clk_freq);
    qdev_prop_set_int32(DEVICE(&s->timg[1]), "apb_freq", apb_clk_freq);
    clock_update_hz(s->cpu[0].clock, cpu_clk_freq );
    clock_update_hz(s->cpu[1].clock, cpu_clk_freq );
}

static void esp32_soc_add_periph_device_n(MemoryRegion *dest, void* dev, hwaddr dport_base_addr, int n)
{
    MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), n);
    memory_region_add_subregion_overlap(dest, dport_base_addr, mr, 0);
    MemoryRegion *mr_apb = g_new(MemoryRegion, 1);
    char *name = g_strdup_printf("mr-apb-0x%08x", (uint32_t) dport_base_addr);
    memory_region_init_alias(mr_apb, OBJECT(dev), name, mr, 0, memory_region_size(mr));
    memory_region_add_subregion_overlap(dest, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, mr_apb, 0);
    g_free(name);
}

static void esp32_soc_add_periph_device(MemoryRegion *dest, void* dev, hwaddr dport_base_addr)
{
    esp32_soc_add_periph_device_n(dest,dev,dport_base_addr,0);
}

static void esp32_soc_add_unimp_device(MemoryRegion *dest, const char* name, hwaddr dport_base_addr, size_t size, uint32_t default_value)
{
    create_unimplemented_device_def(name, dport_base_addr, size, default_value);
    char * name_apb = g_strdup_printf("%s-apb", name);
    create_unimplemented_device_def(name_apb, dport_base_addr - DR_REG_DPORT_APB_BASE + APB_REG_BASE, size, default_value);
    g_free(name_apb);
}

/*
static void split_irq_from_named(DeviceState *src, const char* outname, int n,
                                 qemu_irq out1, qemu_irq out2) {
    DeviceState *splitter = qdev_new(TYPE_SPLIT_IRQ);
    qdev_prop_set_uint32(splitter, "num-lines", 2);
    qdev_realize_and_unref(splitter, NULL, &error_fatal);
    qdev_connect_gpio_out(splitter, 0, out1);
    qdev_connect_gpio_out(splitter, 1, out2);
    qdev_connect_gpio_out_named(src, outname, n,
                                qdev_get_gpio_in(splitter, 0));
}
*/



static void esp32_soc_realize(DeviceState *dev, Error **errp)
{
    Esp32SocState *s = ESP32_SOC(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    const struct MemmapEntry *memmap = esp32_memmap;
    MemoryRegion *sys_mem = get_system_memory();

    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *iram = g_new(MemoryRegion, 1);
    MemoryRegion *icache0 = g_new(MemoryRegion, 1);
    MemoryRegion *icache1 = g_new(MemoryRegion, 1);
    MemoryRegion *rtcslow = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_i = g_new(MemoryRegion, 1);
    MemoryRegion *rtcfast_d = g_new(MemoryRegion, 1);

    for (int i = 0; i < ms->smp.cpus; ++i) {
        assert(i >= 0 && i <= 9);
        MemoryRegion *drom = g_new(MemoryRegion, 1);
        MemoryRegion *irom = g_new(MemoryRegion, 1);

        char name[18];
        snprintf(name, sizeof(name), "esp32.irom.cpu%d", i);
        memory_region_init_rom(irom, NULL, name,
                            memmap[ESP32_MEMREGION_IROM].size, &error_fatal);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[ESP32_MEMREGION_IROM].base, irom);


        snprintf(name, sizeof(name), "esp32.drom.cpu%d", i);
        memory_region_init_alias(drom, NULL, name, irom, 0x60000, memmap[ESP32_MEMREGION_DROM].size);
        memory_region_add_subregion(&s->cpu_specific_mem[i], memmap[ESP32_MEMREGION_DROM].base, drom);
    }

    memory_region_init_ram(dram, NULL, "esp32.dram",
                           memmap[ESP32_MEMREGION_DRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_DRAM].base, dram);

    memory_region_init_ram(iram, NULL, "esp32.iram",
                           memmap[ESP32_MEMREGION_IRAM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_IRAM].base, iram);

    memory_region_init_ram(icache0, NULL, "esp32.icache0",
                           memmap[ESP32_MEMREGION_ICACHE0].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE0].base, icache0);

    memory_region_init_ram(icache1, NULL, "esp32.icache1",
                           memmap[ESP32_MEMREGION_ICACHE1].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_ICACHE1].base, icache1);

    memory_region_init_ram(rtcslow, NULL, "esp32.rtcslow",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[ESP32_MEMREGION_RTCSLOW].base, rtcslow);

    /* RTC Fast memory is only accessible by the PRO CPU */

    memory_region_init_ram(rtcfast_i, NULL, "esp32.rtcfast_i",
                           memmap[ESP32_MEMREGION_RTCSLOW].size, &error_fatal);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32_MEMREGION_RTCFAST_I].base, rtcfast_i);

    memory_region_init_alias(rtcfast_d, NULL, "esp32.rtcfast_d", rtcfast_i, 0, memmap[ESP32_MEMREGION_RTCFAST_D].size);
    memory_region_add_subregion(&s->cpu_specific_mem[0], memmap[ESP32_MEMREGION_RTCFAST_D].base, rtcfast_d);

    /* 1. Link Controller / BT RF registers (0x3ff51000) - RAM for RX filter init etc */
    MemoryRegion *bt_lc_io = g_new(MemoryRegion, 1);
    memory_region_init_ram(bt_lc_io, OBJECT(dev), "esp32.bt_lc", 0x1000, &error_fatal);
    memory_region_add_subregion(sys_mem, DR_REG_BT_BASE, bt_lc_io);

    /* 1b. BT RF/Modem registers (0x3ff5c000) - RAM for filter coefficients etc */
    MemoryRegion *bt_rf_io = g_new(MemoryRegion, 1);
    memory_region_init_ram(bt_rf_io, OBJECT(dev), "esp32.bt_rf", 0x1000, &error_fatal);
    memory_region_add_subregion(sys_mem, 0x3ff5c000, bt_rf_io);

    /* 2. Radio PHY / Baseband (0x3ff71000) */
    MemoryRegion *bt_phy_io = g_new(MemoryRegion, 1);
    memory_region_init_io(bt_phy_io, OBJECT(dev), &phy_mmio_ops, s, "esp32.bt_phy", 0x1000);
    memory_region_add_subregion(sys_mem, DR_REG_PHY_BASE, bt_phy_io);

    /* 2b. APB alias for PHY region (0x60031000) - used by DPORT/instruction bus */
    MemoryRegion *bt_phy_apb = g_new(MemoryRegion, 1);
    memory_region_init_io(bt_phy_apb, OBJECT(dev), &phy_mmio_ops, s, "esp32.bt_phy_apb", 0x1000);
    memory_region_add_subregion(sys_mem, 0x60031000, bt_phy_apb);

    /* 2c. Baseband registers (0x3ff72000 / APB 0x60032000) - simple RAM storage */
    MemoryRegion *bt_bb_io = g_new(MemoryRegion, 1);
    memory_region_init_ram(bt_bb_io, OBJECT(dev), "esp32.bt_bb", 0x1000, &error_fatal);
    memory_region_add_subregion(sys_mem, 0x3ff72000, bt_bb_io);

    /* 2d. APB alias for BB region (0x60032000) */
    MemoryRegion *bt_bb_apb = g_new(MemoryRegion, 1);
    memory_region_init_alias(bt_bb_apb, OBJECT(dev), "esp32.bt_bb_apb", bt_bb_io, 0, 0x1000);
    memory_region_add_subregion(sys_mem, 0x60032000, bt_bb_apb);

    /* 3. Exchange Memory (0x3ffb0000) - Logged RAM */
    uint8_t *em_storage = g_malloc0(BT_EM_SIZE);
    
    /* Initialize NVDS magic number at offset 0 (little-endian: 0xfadebead) */
    em_storage[0] = 0xad;
    em_storage[1] = 0xbe;
    em_storage[2] = 0xde;
    em_storage[3] = 0xfa;
    
    MemoryRegion *em_io = g_new(MemoryRegion, 1);
    memory_region_init_io(em_io, OBJECT(dev), &btdm_em_ops, em_storage, "esp32.bt_em", BT_EM_SIZE);
    memory_region_add_subregion(sys_mem, DR_REG_BT_EM_BASE, em_io);

    qemu_log("BTDM Interception Enabled: LC(0x%x), PHY(0x%x), EM(0x%x)\n", 
             DR_REG_BT_BASE, DR_REG_PHY_BASE, DR_REG_BT_EM_BASE);

    /* Initialize BLE timer for interrupt generation - connect to BT_BB interrupt */
    {
        qemu_irq bt_bb_irq = qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_BT_BB_INTR_SOURCE);
        ble_timer_init(bt_bb_irq);
        qemu_log("BLE Timer initialized (IRQ connected to BT_BB_INTR_SOURCE=%d)\n", 
                 ETS_BT_BB_INTR_SOURCE);
    }

    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_realize(DEVICE(&s->cpu[i]), NULL, &error_fatal);
    }

    qdev_realize(DEVICE(&s->dport), &s->periph_bus, &error_fatal);
    MemoryRegion* dport_mem = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->dport), 0);

    memory_region_add_subregion(sys_mem, DR_REG_DPORT_BASE, dport_mem);
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_APPCPU_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, 1));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_APPCPU_STALL_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, 1));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_DPORT_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CLK_UPDATE_GPIO, 0));

    qdev_connect_gpio_out_named(DEVICE(&s->gpio), ESP32_RTCIO_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_RTCIO_RESET_GPIO, 0));

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), name, OBJECT(qemu_get_cpu(i)), &error_abort);
    }
    qdev_realize(DEVICE(&s->intmatrix), &s->periph_bus, &error_fatal);
    DeviceState* intmatrix_dev = DEVICE(&s->intmatrix);
    memory_region_add_subregion_overlap(dport_mem, ESP32_DPORT_PRO_INTMATRIX_BASE, sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->intmatrix), 0), -1);

    bool init_cache_err = false;
    if (s->dport.flash_blk) {
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            Esp32CacheRegionState *drom0 = &s->dport.cache_state[i].drom0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], drom0->base, &drom0->mem, -1);
            Esp32CacheRegionState *iram0 = &s->dport.cache_state[i].iram0;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], iram0->base, &iram0->mem, -1);
        }
        init_cache_err = true;
    }
    if (s->dport.has_psram) {
        for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
            Esp32CacheRegionState *dram1 = &s->dport.cache_state[i].dram1;
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], dram1->base, &dram1->illegal_access_trap_mem, -2);
            memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], dram1->base, &dram1->mem, -1);
        }
        init_cache_err = true;
    }
    if (init_cache_err) {
        qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_CACHE_ILL_IRQ_GPIO, 0,
                                    qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_CACHE_IA_INTR_SOURCE));
    }

    int n_crosscore_irqs = ESP32_DPORT_CROSSCORE_INT_COUNT;
    object_property_set_int(OBJECT(&s->crosscore_int), "n_irqs", n_crosscore_irqs, &error_abort);
    qdev_realize(DEVICE(&s->crosscore_int), &s->periph_bus, &error_fatal);
    memory_region_add_subregion_overlap(dport_mem, ESP32_DPORT_CROSSCORE_INT_BASE, &s->crosscore_int.iomem, -1);

    for (int index = 0; index < ESP32_DPORT_CROSSCORE_INT_COUNT; ++index) {
        qemu_irq target = qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_FROM_CPU_INTR0_SOURCE + index);
        assert(target);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->crosscore_int), index, target);
    }

    qdev_realize(DEVICE(&s->rsa), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rsa, DR_REG_RSA_BASE);

    qdev_realize(DEVICE(&s->sha), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->sha, DR_REG_SHA_BASE);

    qdev_realize(DEVICE(&s->aes), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->aes, DR_REG_AES_BASE);

    qdev_realize(DEVICE(&s->ledc), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->ledc, DR_REG_LEDC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->ledc), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_LEDC_INTR_SOURCE));

    object_property_set_int(OBJECT(&s->mcpwm0),"func_sig_start",32, &error_abort);
    qdev_realize(DEVICE(&s->mcpwm0), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->mcpwm0, DR_REG_PWM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->mcpwm0), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_PWM0_INTR_SOURCE));

    object_property_set_int(OBJECT(&s->mcpwm1),"func_sig_start",108, &error_abort);
    qdev_realize(DEVICE(&s->mcpwm1), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->mcpwm1, DR_REG_PWM1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->mcpwm1), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_PWM1_INTR_SOURCE));


    qdev_realize(DEVICE(&s->rtc_cntl), &s->rtc_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rtc_cntl, DR_REG_RTCCNTL_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_DIG_RESET_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_DIG_RESET_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CLK_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(dev, ESP32_RTC_CLK_UPDATE_GPIO, 0));
    for (int i = 0; i < ms->smp.cpus; ++i) {
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_RESET_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->rtc_cntl), ESP32_RTC_CPU_STALL_GPIO, i,
                                    qdev_get_gpio_in_named(dev, ESP32_RTC_CPU_STALL_GPIO, i));
    }

    qdev_realize(DEVICE(&s->gpio), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->gpio, DR_REG_GPIO_BASE);
    esp32_soc_add_periph_device_n(sys_mem, &s->gpio, DR_REG_IO_MUX_BASE,1);
    esp32_soc_add_periph_device_n(sys_mem, &s->gpio, DR_REG_RTCIO_BASE,2);


    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio),0,qdev_get_gpio_in(intmatrix_dev, ETS_GPIO_INTR_SOURCE));

    qdev_connect_gpio_out_named(DEVICE(&s->ledc),"func_irq",0,
    							qdev_get_gpio_in_named(DEVICE(&s->gpio),ESP32_GPIOS_FUNC,0));
	qdev_connect_gpio_out_named(DEVICE(&s->mcpwm0),"func_irq",0,
                                qdev_get_gpio_in_named(DEVICE(&s->gpio),ESP32_GPIOS_FUNC,0));
	qdev_connect_gpio_out_named(DEVICE(&s->mcpwm1),"func_irq",0,
	                            qdev_get_gpio_in_named(DEVICE(&s->gpio),ESP32_GPIOS_FUNC,0));


    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        const hwaddr uart_base[] = {DR_REG_UART_BASE, DR_REG_UART1_BASE, DR_REG_UART2_BASE};
        qdev_realize(DEVICE(&s->uart[i]), &s->periph_bus, &error_fatal);
        esp32_soc_add_periph_device(sys_mem, &s->uart[i], uart_base[i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_UART0_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        qdev_realize(DEVICE(&s->frc_timer[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->frc_timer[i], DR_REG_FRC_TIMER_BASE + i * ESP32_FRC_TIMER_STRIDE);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->frc_timer[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_TIMER1_INTR_SOURCE + i));
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        s->timg[i].id = i;

        const hwaddr timg_base[] = {DR_REG_TIMERGROUP0_BASE, DR_REG_TIMERGROUP1_BASE};
        qdev_realize(DEVICE(&s->timg[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->timg[i], timg_base[i]);

        int timg_level_int[] = { ETS_TG0_T0_LEVEL_INTR_SOURCE, ETS_TG1_T0_LEVEL_INTR_SOURCE };
        int timg_edge_int[] = { ETS_TG0_T0_EDGE_INTR_SOURCE, ETS_TG1_T0_EDGE_INTR_SOURCE };
        for (Esp32TimgInterruptType it = TIMG_T0_INT; it < TIMG_INT_MAX; ++it) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), it, qdev_get_gpio_in(intmatrix_dev, timg_level_int[i] + it));
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->timg[i]), TIMG_INT_MAX + it, qdev_get_gpio_in(intmatrix_dev, timg_edge_int[i] + it));
        }

        qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_CPU_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_CPU_RESET_GPIO, i));
        qdev_connect_gpio_out_named(DEVICE(&s->timg[i]), ESP32_TIMG_WDT_SYS_RESET_GPIO, 0,
                                    qdev_get_gpio_in_named(dev, ESP32_TIMG_WDT_SYS_RESET_GPIO, i));
    }
 //   s->timg[0].wdt_en_at_reset = true;
    const hwaddr spi_base[] = {
            DR_REG_SPI0_BASE, DR_REG_SPI1_BASE, DR_REG_SPI2_BASE, DR_REG_SPI3_BASE
    };
    // speed up vspi and hspi by allowng the controller to send 32 bits at a time.
    // this is only suppoerted by the st7789v 
    object_property_set_bool(OBJECT(&s->spi[2]),"xfer_32_bits",true, &error_abort);
    object_property_set_bool(OBJECT(&s->spi[3]),"xfer_32_bits",true, &error_abort);
    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {        
        qdev_realize(DEVICE(&s->spi[i]), &s->periph_bus, &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->spi[i], spi_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_SPI0_INTR_SOURCE + i));
    }

    qdev_realize(DEVICE(&s->rmt), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rmt, DR_REG_RMT_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rmt), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_RMT_INTR_SOURCE));

    for (int i = 0; i < ESP32_I2C_COUNT; i++) {
        const hwaddr i2c_base[] = {
            DR_REG_I2C_EXT_BASE, DR_REG_I2C1_EXT_BASE
        };
        qdev_realize(DEVICE(&s->i2c[i]), sysbus_get_default(), &error_fatal);

        esp32_soc_add_periph_device(sys_mem, &s->i2c[i], i2c_base[i]);

        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(intmatrix_dev, ETS_I2C_EXT0_INTR_SOURCE + i));
    }

    /* TWAI model passes intmatrix IRQs to the SJA1000 controller model
     * in realize function. That means that irq linking MUST be
     * performed before realization of TWAI peripheral.
     */
//    qdev_realize(DEVICE(&s->twai), &s->periph_bus, &error_fatal);
//    esp32_soc_add_periph_device(sys_mem, &s->twai, DR_REG_CAN_BASE); 
//    sysbus_connect_irq(SYS_BUS_DEVICE(&s->twai), 0,
//                       qdev_get_gpio_in(intmatrix_dev, ETS_CAN_INTR_SOURCE));

    qdev_realize(DEVICE(&s->rng), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->rng, ESP32_RNG_BASE);

    qdev_realize(DEVICE(&s->efuse), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->efuse, DR_REG_EFUSE_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->efuse), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_EFUSE_INTR_SOURCE));

    qdev_realize(DEVICE(&s->sens), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->sens, DR_REG_SENS_BASE);

    qdev_realize(DEVICE(&s->ana), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->ana, DR_REG_ANA_BASE);

    qdev_realize(DEVICE(&s->fe), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->fe, DR_REG_FE_BASE);

    qdev_realize(DEVICE(&s->phya), &s->periph_bus, &error_fatal);
    esp32_soc_add_periph_device(sys_mem, &s->phya, DR_REG_PHYA_BASE);

    qdev_realize(DEVICE(&s->flash_enc), &s->periph_bus, &error_abort);
    esp32_soc_add_periph_device(sys_mem, &s->flash_enc, DR_REG_SPI_ENCRYPT_BASE);

    qdev_connect_gpio_out_named(DEVICE(&s->efuse), ESP32_EFUSE_UPDATE_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_EFUSE_UPDATE_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_FLASH_ENC_EN_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_ENC_EN_GPIO, 0));
    qdev_connect_gpio_out_named(DEVICE(&s->dport), ESP32_DPORT_FLASH_DEC_EN_GPIO, 0,
                                qdev_get_gpio_in_named(DEVICE(&s->flash_enc), ESP32_FLASH_ENCRYPTION_DEC_EN_GPIO, 0));

    qdev_realize(DEVICE(&s->sdmmc), &s->periph_bus, &error_abort);
    esp32_soc_add_periph_device(sys_mem, &s->sdmmc, DR_REG_SDMMC_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdmmc), 0,
                       qdev_get_gpio_in(intmatrix_dev, ETS_SDIO_HOST_INTR_SOURCE));

 //   esp32_soc_add_unimp_device(sys_mem, "esp32.rtcio", DR_REG_RTCIO_BASE, 0x400,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.wdg", DR_REG_WDG_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.hinf", DR_REG_HINF_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.slc", DR_REG_SLC_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.slchost", DR_REG_SLCHOST_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.uhci0", DR_REG_UHCI0_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.uhci1", DR_REG_UHCI1_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.apbctrl", DR_REG_APB_CTRL_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.i2s0", DR_REG_I2S_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.i2s1", DR_REG_I2S1_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.fe2", DR_REG_FE2_BASE, 0x1000, -1);
    
    /* Note: DR_REG_PHY_BASE (0x3ff71000) and DR_REG_BT_BASE (0x3ff51000) are handled
     * by bt_phy_io (phy_mmio_ops) and bt_lc_io (bt_controller_ops) respectively,
     * which are initialized earlier in this function. */
    
    esp32_soc_add_unimp_device(sys_mem, "esp32.chipv7_phyb", DR_REG_WDEV_BASE, 0x1000,0);
    esp32_soc_add_unimp_device(sys_mem, "esp32.unknown_wifi", DR_REG_NRX_BASE  , 0x1000,-1);
    esp32_soc_add_unimp_device(sys_mem, "esp32.unknown_wifi1", DR_REG_BB_BASE , 0x1000,-1);
    /* Extended WIFI/PHY region - covers 0x3ff73000-0x3ff73fff + extended area for phy_chip_v7 */
    esp32_soc_add_unimp_device(sys_mem, "esp32.wifi_phy", DR_REG_WIFI_BASE, 0x1000, 1);


    /* st7789v is attached to SPI2 and SPI3 so the both HSPI and VSPI will work,
    they share a single console*/
    DeviceState *disp=ssi_create_peripheral(s->spi[2].spi, "st7789v");
    DeviceState *disp1=ssi_create_peripheral(s->spi[3].spi, "st7789v");
    ssi_create_peripheral(s->rmt.rmt, "rgbled");

    ServoState *servo=servo_create_simple(OBJECT(s),"servo");
    qdev_connect_gpio_out_named(DEVICE(&s->gpio), ESP32_GPIOS, 27, qdev_get_gpio_in(DEVICE(servo), 0));

	// use gpio 16 and 4 for cmd and backlight of both displays
//    split_irq_from_named(DEVICE(&s->gpio),ESP32_GPIOS, 16, qdev_get_gpio_in_named(disp, "cmd", 0), qdev_get_gpio_in_named(disp1, "cmd", 0));
//    split_irq_from_named(DEVICE(&s->gpio),ESP32_GPIOS, 4, qdev_get_gpio_in_named(disp, "backlight", 0), qdev_get_gpio_in_named(disp1, "backlight", 0));

    qdev_connect_gpio_out_named(DEVICE(&s->gpio), ESP32_GPIOS, 4, qdev_get_gpio_in_named(disp, "backlight", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->gpio), ESP32_GPIOS, 16, qdev_get_gpio_in_named(disp, "cmd", 0));

    qemu_irq in0=qdev_get_gpio_in_named(DEVICE(&s->gpio), ESP32_GPIOS_IN, 0);
    qemu_irq in35=qdev_get_gpio_in_named(DEVICE(&s->gpio), ESP32_GPIOS_IN, 35);
    qdev_connect_gpio_out_named(disp, "buttons", 0, in0);
    qdev_connect_gpio_out_named(disp, "buttons", 1, in35);
    qdev_connect_gpio_out_named(disp1, "buttons", 0, in0);
    qdev_connect_gpio_out_named(disp1, "buttons", 1, in35);

    qdev_connect_gpio_out_named(disp, "reset", 0,
                                    qdev_get_gpio_in_named(dev, "full_reset", 0));



    /* Emulation of a fake register used to mark that the chip is run via QEMU */
    MemoryRegion *apbctrl_mem = g_new(MemoryRegion, 1);
    memory_region_init_ram(apbctrl_mem, NULL, "esp32.apbctrl_date_reg", 8 /* bytes */, &error_fatal);

    /* This register is not used in the real hardware (hardwired to 0), but is still accesible, reading
     * it won't trigger an exception, so we can override it */
    const hwaddr apb_ctrl_emu_reg = DR_REG_APB_CTRL_BASE + 0x78;
    /* Store "QEMU" as a 32-bit value */
    const uint32_t apb_ctrl_emu_val = 0x51454d55;
    /* The memory region must be added before writing to the CPU memory */
    memory_region_add_subregion(sys_mem, apb_ctrl_emu_reg, apbctrl_mem);
    cpu_physical_memory_write(apb_ctrl_emu_reg, &apb_ctrl_emu_val, 4);

    /* Emulation of APB_CTRL_DATE_REG, needed for ECO3 revision detection.
     * This is a small hack to avoid creating a whole new device just to emulate one
     * register.
     */
    const hwaddr apb_ctrl_date_reg = DR_REG_APB_CTRL_BASE + 0x7c;
    uint32_t apb_ctrl_date_reg_val = 0x16042000 | 0x80000000;  /* MSB indicates ECO3 silicon revision */
    cpu_physical_memory_write(apb_ctrl_date_reg, &apb_ctrl_date_reg_val, 4);

    qemu_register_reset((QEMUResetHandler*) esp32_soc_reset, dev);
}

static void esp32_soc_init(Object *obj)
{
    Esp32SocState *s = ESP32_SOC(obj);
    MachineState *ms = MACHINE(qdev_get_machine());
    char name[16];

    MemoryRegion *system_memory = get_system_memory();

    qbus_init(&s->periph_bus, sizeof(s->periph_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-periph-bus");
    qbus_init(&s->rtc_bus, sizeof(s->rtc_bus),
                        TYPE_SYSTEM_BUS, DEVICE(s), "esp32-rtc-bus");

    for (int i = 0; i < ms->smp.cpus; ++i) {
        snprintf(name, sizeof(name), "cpu%d", i);
        object_initialize_child(obj, name, &s->cpu[i], TYPE_ESP32_CPU);

        const uint32_t cpuid[ESP32_CPU_COUNT] = { 0xcdcd, 0xabab };
        s->cpu[i].env.sregs[PRID] = cpuid[i];

        snprintf(name, sizeof(name), "cpu%d-mem", i);
        memory_region_init(&s->cpu_specific_mem[i], NULL, name, UINT32_MAX);

        CPUState* cs = CPU(&s->cpu[i]);
        cs->num_ases = 1;
        cpu_address_space_init(cs, 0, "cpu-memory", &s->cpu_specific_mem[i]);

        MemoryRegion *cpu_view_sysmem = g_new(MemoryRegion, 1);
        snprintf(name, sizeof(name), "cpu%d-sysmem", i);
        memory_region_init_alias(cpu_view_sysmem, NULL, name, system_memory, 0, UINT32_MAX);
        memory_region_add_subregion_overlap(&s->cpu_specific_mem[i], 0, cpu_view_sysmem, 0);
        cs->memory = &s->cpu_specific_mem[i];
    }

    for (int i = 0; i < ESP32_UART_COUNT; ++i) {
        snprintf(name, sizeof(name), "uart%d", i);
        object_initialize_child(obj, name, &s->uart[i], TYPE_ESP32_UART);
    }

    object_property_add_alias(obj, "serial0", OBJECT(&s->uart[0]), "chardev");
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart[1]), "chardev");
    object_property_add_alias(obj, "serial2", OBJECT(&s->uart[2]), "chardev");

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_ESP32_GPIO);

    object_initialize_child(obj, "dport", &s->dport, TYPE_ESP32_DPORT);

    object_initialize_child(obj, "intmatrix", &s->intmatrix, TYPE_ESP32_INTMATRIX);

    object_initialize_child(obj, "crosscore_int", &s->crosscore_int, TYPE_ESP32_CROSSCORE_INT);

    object_initialize_child(obj, "rtc_cntl", &s->rtc_cntl, TYPE_ESP32_RTC_CNTL);

    for (int i = 0; i < ESP32_FRC_COUNT; ++i) {
        snprintf(name, sizeof(name), "frc%d", i);
        object_initialize_child(obj, name, &s->frc_timer[i], TYPE_ESP32_FRC_TIMER);
    }

    for (int i = 0; i < ESP32_TIMG_COUNT; ++i) {
        snprintf(name, sizeof(name), "timg%d", i);
        object_initialize_child(obj, name, &s->timg[i], TYPE_ESP32_TIMG);
    }
    for (int i = 0; i < ESP32_SPI_COUNT; ++i) {
        snprintf(name, sizeof(name), "spi%d", i);
        object_initialize_child(obj, name, &s->spi[i], TYPE_ESP32_SPI);
    }

    for (int i = 0; i < ESP32_I2C_COUNT; ++i) {
        snprintf(name, sizeof(name), "i2c%d", i);
        object_initialize_child(obj, name, &s->i2c[i], TYPE_ESP32_I2C);
    }

//    object_initialize_child(obj, "twai", &s->twai, TYPE_ESP32_TWAI);

    object_initialize_child(obj, "rng", &s->rng, TYPE_ESP32_RNG);

    object_initialize_child(obj, "sha", &s->sha, TYPE_ESP32_SHA);

    object_initialize_child(obj, "aes", &s->aes, TYPE_ESP32_AES);

    object_initialize_child(obj, "ledc", &s->ledc, TYPE_ESP32_LEDC);

    object_initialize_child(obj, "mcpwm0", &s->mcpwm0, TYPE_ESP32_MCPWM);

    object_initialize_child(obj, "mcpwm1", &s->mcpwm1, TYPE_ESP32_MCPWM);

    object_initialize_child(obj, "rsa", &s->rsa, TYPE_ESP32_RSA);

    object_initialize_child(obj, "sens", &s->sens, TYPE_ESP32_SENS);

    object_initialize_child(obj, "ana", &s->ana, TYPE_ESP32_ANA);

    object_initialize_child(obj, "rmt", &s->rmt, TYPE_ESP32_RMT);

    if(qemu_find_nic_info(TYPE_ESP32_WIFI, false, NULL)!=NULL)
	    object_initialize_child(obj, "wifi", &s->wifi, TYPE_ESP32_WIFI);

	// these peripherals need to know which device we are
    s->wifi.iss3=0;
    s->ana.iss3=0;

    object_initialize_child(obj, "fe", &s->fe, TYPE_ESP32_FE);

    object_initialize_child(obj, "phya", &s->phya, TYPE_ESP32_PHYA);

    object_initialize_child(obj, "efuse", &s->efuse, TYPE_ESP32_EFUSE);

    object_initialize_child(obj, "flash_enc", &s->flash_enc, TYPE_ESP32_FLASH_ENCRYPTION);

    object_initialize_child(obj, "sdmmc", &s->sdmmc, TYPE_DWC_SDMMC);

    qdev_init_gpio_in_named(DEVICE(s), esp32_dig_reset, ESP32_RTC_DIG_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_reset, ESP32_RTC_CPU_RESET_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32_cpu_stall, ESP32_RTC_CPU_STALL_GPIO, ESP32_CPU_COUNT);
    qdev_init_gpio_in_named(DEVICE(s), esp32_clk_update, ESP32_RTC_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_rtc_reset, ESP32_RTCIO_RESET_GPIO, 1);
    qdev_init_gpio_in_named(DEVICE(s), esp32_timg_cpu_reset, ESP32_TIMG_WDT_CPU_RESET_GPIO, 2);
    qdev_init_gpio_in_named(DEVICE(s), esp32_timg_sys_reset, ESP32_TIMG_WDT_SYS_RESET_GPIO, 2);
    qdev_init_gpio_in_named(DEVICE(s), esp32_full_reset, "full_reset", 1);

}

static Property esp32_soc_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32_soc_realize;
    device_class_set_props(dc, esp32_soc_properties);
}

static const TypeInfo esp32_soc_info = {
    .name = TYPE_ESP32_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(Esp32SocState),
    .instance_init = esp32_soc_init,
    .class_init = esp32_soc_class_init
};

static void esp32_soc_register_types(void)
{
    type_register_static(&esp32_soc_info);
}

type_init(esp32_soc_register_types)



static uint64_t translate_phys_addr(void *opaque, uint64_t addr)
{
    XtensaCPU *cpu = opaque;

    return cpu_get_phys_page_debug(CPU(cpu), addr);
}


struct Esp32MachineState {
    MachineState parent;

    Esp32SocState esp32;
    DeviceState *flash_dev;
};
#define TYPE_ESP32_MACHINE MACHINE_TYPE_NAME("esp32")

OBJECT_DECLARE_SIMPLE_TYPE(Esp32MachineState, ESP32_MACHINE)


static void esp32_machine_init_spi_flash(Esp32SocState *ss, BlockBackend* blk)
{
    /* "main" flash chip is attached to SPI1, CS0 */
    DeviceState *spi_master = DEVICE(&ss->spi[1]);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");

    /* select the flash chip based on the image size */
    int64_t image_size = blk_getlength(blk);
    const char* flash_chip_model = NULL;
    switch (image_size) {
        case 2 * 1024 * 1024: flash_chip_model = "w25x16"; break;
        case 4 * 1024 * 1024: flash_chip_model = "gd25q32"; break;
        case 8 * 1024 * 1024: flash_chip_model = "gd25q64"; break;
        case 16 * 1024 * 1024: flash_chip_model = "is25lp128"; break;
        default: error_report("Error: only 2, 4, 8, 16 MB flash images are supported"); return;
    }

    DeviceState *flash_dev = qdev_new(flash_chip_model);
    qdev_prop_set_drive(flash_dev, "drive", blk);
    qdev_prop_set_uint8(flash_dev, "cs", 0);
    qdev_realize_and_unref(flash_dev, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 0,
                                qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0));
}

static void esp32_machine_init_psram(Esp32SocState *ss, uint32_t size_mbytes)
{
    /* PSRAM attached to SPI1, CS1 */
    DeviceState *spi_master = DEVICE(&ss->spi[1]);
    BusState* spi_bus = qdev_get_child_bus(spi_master, "spi");
    DeviceState *psram = qdev_new(TYPE_SSI_PSRAM);
    qdev_prop_set_uint32(psram, "size_mbytes", size_mbytes);
    qdev_prop_set_uint8(psram, "cs", 1);
    qdev_realize_and_unref(psram, spi_bus, &error_fatal);
    qdev_connect_gpio_out_named(spi_master, SSI_GPIO_CS, 1,
                                qdev_get_gpio_in_named(psram, SSI_GPIO_CS, 0));
}


static void esp32_machine_init_i2c(Esp32SocState *s)
{
    /* It should be possible to create an I2C device from the command line,
     * however for this to work the I2C bus must be reachable from sysbus-default.
     * At the moment the peripherals are added to an unrelated bus, to avoid being
     * reset on CPU reset.
     * If we find a way to decouple peripheral reset from sysbus reset,
     * we can move them to the sysbus and thus enable creation of i2c devices.
     */
    DeviceState *i2c_master = DEVICE(&s->i2c[0]);
    I2CBus* i2c_bus = I2C_BUS(qdev_get_child_bus(i2c_master, "i2c"));
    I2CSlave* tmp105 = i2c_slave_create_simple(i2c_bus, "tmp105", 0x48);
    object_property_set_int(OBJECT(tmp105), "temperature", 25 * 1000, &error_fatal);
    i2c_slave_create_simple(i2c_bus, "mpu6050", 0x68);
}

static void esp32_machine_init_openeth(Esp32SocState *ss)
{
    SysBusDevice *sbd;
    MemoryRegion* sys_mem = get_system_memory();
    
	const char* type_openeth = "open_eth";
	NICInfo *nd = qemu_find_nic_info(type_openeth, false, NULL);
	if(nd!=NULL) {
        hwaddr reg_base = DR_REG_EMAC_BASE;
        hwaddr desc_base = reg_base + 0x400;
        qemu_irq irq = qdev_get_gpio_in(DEVICE(&ss->intmatrix), ETS_ETH_MAC_INTR_SOURCE);
		DeviceState* open_eth_dev = qdev_new(type_openeth);
        ss->eth = open_eth_dev;
        qdev_set_nic_properties(open_eth_dev, nd);
        sbd = SYS_BUS_DEVICE(open_eth_dev);
        sysbus_realize_and_unref(sbd, &error_fatal);
        sysbus_connect_irq(sbd, 0, irq);
        memory_region_add_subregion(sys_mem, reg_base, sysbus_mmio_get_region(sbd, 0));
        memory_region_add_subregion(sys_mem, desc_base, sysbus_mmio_get_region(sbd, 1));
	}
	nd = qemu_find_nic_info(TYPE_ESP32_WIFI, false, NULL);
	if(nd!=NULL) {
        qdev_set_nic_properties(DEVICE(&ss->wifi), nd);
        sbd = SYS_BUS_DEVICE(DEVICE(&ss->wifi));
        sysbus_realize_and_unref(sbd, &error_fatal);
        esp32_soc_add_periph_device(sys_mem, &ss->wifi, DR_REG_WIFI_BASE);
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->wifi), 0,
                   qdev_get_gpio_in(DEVICE(&ss->intmatrix), ETS_WIFI_MAC_INTR_SOURCE));
    }
}

static void esp32_machine_init_sd(Esp32SocState *ss)
{
    DriveInfo *dinfo = drive_get(IF_SD, 0, 0);
    if (dinfo) {
        DeviceState *card;

        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
        /* See the comment on not using sysbus-default in esp32_machine_init_i2c */
        DeviceState *sdmmc = DEVICE(&ss->sdmmc);
        SDBus* sd_bus = SD_BUS(qdev_get_child_bus(sdmmc, "sd-bus"));
        qdev_realize_and_unref(card, BUS(sd_bus), &error_fatal);
    }
}

static void esp32_machine_init(MachineState *machine)
{
    BlockBackend* blk = NULL;
    DriveInfo *dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qemu_log("Adding SPI flash device\n");
        blk = blk_by_legacy_dinfo(dinfo);
    } else {
        qemu_log("Not initializing SPI Flash\n");
    }

    Esp32MachineState *ms = ESP32_MACHINE(machine);
    object_initialize_child(OBJECT(ms), "soc", &ms->esp32, TYPE_ESP32_SOC);
    Esp32SocState *ss = ESP32_SOC(&ms->esp32);

    if (blk) {
        ss->dport.flash_blk = blk;
    }
    qdev_prop_set_chr(DEVICE(ss), "serial0", serial_hd(0));
    qdev_prop_set_chr(DEVICE(ss), "serial1", serial_hd(1));
    qdev_prop_set_chr(DEVICE(ss), "serial2", serial_hd(2));
    if (machine->ram_size > 0) {
        qdev_prop_set_bit(DEVICE(&ss->dport), "has_psram", true);
    }

    qdev_realize(DEVICE(ss), NULL, &error_fatal);

    if (blk) {
        esp32_machine_init_spi_flash(ss, blk);
    }

    if (machine->ram_size > 0) {
        esp32_machine_init_psram(ss, (uint32_t) (machine->ram_size / MiB));
    }

    esp32_machine_init_i2c(ss);

    esp32_machine_init_openeth(ss);

    esp32_machine_init_sd(ss);



    /* Need MMU initialized prior to ELF loading,
     * so that ELF gets loaded into virtual addresses
     */
    cpu_reset(CPU(&ss->cpu[0]));

    const char *load_elf_filename = NULL;
    if (machine->firmware) {
        load_elf_filename = machine->firmware;
    }
    if (machine->kernel_filename) {
        qemu_log("Warning: both -bios and -kernel arguments specified. Only loading the the -kernel file.\n");
        load_elf_filename = machine->kernel_filename;
    }

    if (load_elf_filename) {
        uint64_t elf_entry;
        uint64_t elf_lowaddr;
        int size = load_elf(load_elf_filename, NULL,
                               translate_phys_addr, &ss->cpu[0],
                               &elf_entry, &elf_lowaddr,
                               NULL, NULL, 0, EM_XTENSA, 0, 0);
        if (size < 0) {
            error_report("Error: could not load ELF file '%s'", load_elf_filename);
            exit(1);
        }

        if (elf_entry != XCHAL_RESET_VECTOR_PADDR) {
            // Since ROM is empty when loading elf file AND
            // PC value is 0x40000400 after reset
            // need to jump to elf entry point to run a programm
            uint8_t p[4];
            memcpy(p, &elf_entry, 4);
            uint8_t boot[] = {
                0x06, 0x01, 0x00,       /* j    1 */
                0x00,                   /* .literal_position */
                p[0], p[1], p[2], p[3], /* .literal elf_entry */
                                        /* 1: */
                0x01, 0xff, 0xff,       /* l32r a0, elf_entry */
                0xa0, 0x00, 0x00,       /* jx   a0 */
            };
            // Write boot function to reset-vector address (0x40000400) of the CPU 0
            rom_add_blob_fixed_as("boot", boot, sizeof(boot), XCHAL_RESET_VECTOR_PADDR, CPU(&ss->cpu[0])->as);
            ss->cpu[0].env.pc = XCHAL_RESET_VECTOR_PADDR;
        }
    } else {
        char *rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32-v3-rom.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found (1)");
            exit(1);
        }

        int size = load_image_targphys_as(rom_binary, esp32_memmap[ESP32_MEMREGION_IROM].base, esp32_memmap[ESP32_MEMREGION_IROM].size, CPU(&ss->cpu[0])->as);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);

        rom_binary = qemu_find_file(QEMU_FILE_TYPE_BIOS, "esp32-v3-rom-app.bin");
        if (rom_binary == NULL) {
            error_report("Error: -bios argument not set, and ROM code binary not found (2)");
            exit(1);
        }

        size = load_image_targphys_as(rom_binary, esp32_memmap[ESP32_MEMREGION_IROM].base, esp32_memmap[ESP32_MEMREGION_IROM].size, CPU(&ss->cpu[1])->as);
        if (size < 0) {
            error_report("Error: could not load ROM binary '%s'", rom_binary);
            exit(1);
        }
        g_free(rom_binary);
        
    }
}

static ram_addr_t esp32_fixup_ram_size(ram_addr_t requested_size)
{
    ram_addr_t size;
    if (requested_size == 0) {
        size = 0;
    } else if (requested_size <= 2 * MiB) {
        size = 2 * MiB;
    } else if (requested_size <= 4 * MiB ) {
        size = 4 * MiB;
    } else {
        qemu_log("RAM size larger than 4 MB not supported\n");
        size = 4 * MiB;
    }
    return size;
}

/* Initialize machine type */
static void esp32_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    mc->desc = "Espressif ESP32 machine";
    mc->init = esp32_machine_init;
    mc->max_cpus = 2;
    mc->is_default = true;
    mc->default_cpus = 2;
    mc->default_ram_size = 0;
    mc->fixup_ram_size = esp32_fixup_ram_size;
}

static const TypeInfo esp32_info = {
    .name = TYPE_ESP32_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Esp32MachineState),
    .class_init = esp32_machine_class_init,
};

static void esp32_machine_type_init(void)
{
    type_register_static(&esp32_info);
}

type_init(esp32_machine_type_init);
