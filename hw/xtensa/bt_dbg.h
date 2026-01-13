/*
 * ESP32 Bluetooth Debug Structures
 * RivieraWaves BT/BLE Exchange Memory Layout
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ESP32_BT_DBG_H
#define ESP32_BT_DBG_H

#include <stdint.h>
#include <stdbool.h>

/* Exchange Memory base address */
#define EM_BASE_ADDR    0x3FFB0000

/*
 * RXSTAT (offset 0x00) - RX status flags
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t pkt_valid   : 1;  /* Descriptor valid */
        uint16_t crc_err     : 1;  /* CRC error */
        uint16_t hec_err     : 1;  /* HEC error */
        uint16_t mic_err     : 1;  /* MIC failure */
        uint16_t decrypt_err : 1;  /* Decryption failure */
        uint16_t reserved    : 11;
    };
} em_bt_rxstat_t;

/*
 * BT Header (offset 0x02) - Baseband packet header
 */
typedef union {
    uint16_t raw;
    struct __attribute__((packed)) {
        uint8_t lt_addr : 3;   /* Logical Transport Address */
        uint8_t type    : 4;   /* Packet type */
        uint8_t flow    : 1;   /* Flow control */
        uint8_t arqn    : 1;   /* ARQ Number */
        uint8_t seqn    : 1;   /* Sequence Number */
        uint8_t hec     : 8;   /* Header Error Check */
    };
} em_bt_header_t;

/*
 * ACL Header (offset 0x04) - L2CAP header
 */
typedef union {
    uint16_t raw;
    struct __attribute__((packed)) {
        uint16_t llid   : 2;   /* Logical Link ID */
        uint16_t flow   : 1;   /* Flow control */
        uint16_t length : 10;  /* Payload length */
        uint16_t rfu    : 3;   /* Reserved */
    };
} em_acl_header_t;

/*
 * RX Data Pointer (offset 0x06) - EM offset to payload
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t offset : 13;  /* Offset from EM base (0x3FFB0000) */
        uint16_t rfu    : 3;
    };
} em_bt_rxdataptr_t;

/*
 * RSSI / Channel (offset 0x0A)
 */
typedef union {
    uint16_t val;
    struct {
        uint8_t rssi;          /* RSSI value */
        uint8_t channel : 7;   /* RF channel */
        uint8_t rfu     : 1;
    };
} em_bt_rssi_ch_t;

/*
 * RX Control (offset 0x0C)
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t encrypted : 1;
        uint16_t mic_ok    : 1;
        uint16_t is_edr    : 1;
        uint16_t is_fhs    : 1;
        uint16_t reserved  : 12;
    };
} em_bt_rxctrl_t;

/*
 * Complete RX Descriptor (14 bytes)
 */
typedef struct __attribute__((packed)) {
    uint16_t rxstat;       /* 0x00: RX status / flags */
    uint16_t bt_header;    /* 0x02: BT header (LT_ADDR, TYPE, FLOW, ARQN, SEQN, HEC) */
    uint16_t acl_header;   /* 0x04: ACL header (LLID, FLOW, LENGTH) */
    uint16_t data_ptr;     /* 0x06: EM offset to payload */
    uint16_t crc;          /* 0x08: CRC / HEC / error flags */
    uint16_t rssi_ch;      /* 0x0A: RSSI + channel */
    uint16_t rxctrl;       /* 0x0C: RX control flags */
} em_bt_rxdesc_t;

_Static_assert(sizeof(em_bt_rxdesc_t) == 14, "RX descriptor must be 14 bytes");

/*
 * BLE RX Descriptor (similar structure)
 */
typedef struct __attribute__((packed)) {
    uint16_t rxstat;       /* 0x00: RX status */
    uint16_t rxphce;       /* 0x02: PHY/CTE info */
    uint16_t rxchass;      /* 0x04: Channel assessment */
    uint16_t rxdataptr;    /* 0x06: Data pointer */
    uint16_t rxrssi;       /* 0x08: RSSI */
    uint16_t rxchannel;    /* 0x0A: Channel + flags */
    uint16_t rxsyncword;   /* 0x0C: Sync word match */
} em_ble_rxdesc_t;

/*
 * TX Control (offset 0x00)
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t tx_en       : 1;  /* Enable transmission */
        uint16_t encrypt     : 1;  /* Enable encryption */
        uint16_t retry_en    : 1;  /* Allow retries */
        uint16_t flush       : 1;  /* Flush after TX */
        uint16_t is_lmp      : 1;  /* LMP packet */
        uint16_t is_poll     : 1;  /* POLL/NULL handling */
        uint16_t reserved0   : 8;
        uint16_t tx_done     : 1;  /* TX complete (set by HW) */
        uint16_t tx_ready    : 1;  /* HW owns descriptor (clear to start TX) */
    };
} em_bt_txctrl_t;

/*
 * TX Data Pointer (offset 0x06)
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t offset : 13;  /* EM offset (added to 0x3FFB0000) */
        uint16_t rfu    : 3;
    };
} em_bt_txdataptr_t;

/*
 * TX Rate / Modulation (offset 0x0A)
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t rate     : 3;  /* 1M, 2M, 3M (EDR) */
        uint16_t power    : 4;  /* TX power index */
        uint16_t is_edr   : 1;  /* EDR mode */
        uint16_t reserved : 8;
    };
} em_bt_txrate_t;

/*
 * TX Status (offset 0x0C) - written by hardware after TX
 */
typedef union {
    uint16_t val;
    struct {
        uint16_t retries  : 4;  /* Retry count */
        uint16_t acked    : 1;  /* ACK received */
        uint16_t flushed  : 1;  /* Packet flushed */
        uint16_t timeout  : 1;  /* TX timeout */
        uint16_t reserved : 9;
    };
} em_bt_txstat_t;

/*
 * BT TX Descriptor (14 bytes)
 */
typedef struct __attribute__((packed)) {
    uint16_t txctrl;       /* 0x00: TX control (bit 15=tx_ready, clear to start TX) */
    uint16_t bt_header;    /* 0x02: BT header (LT_ADDR, TYPE, FLOW, ARQN, SEQN, HEC) */
    uint16_t acl_header;   /* 0x04: ACL header (LLID, FLOW, LENGTH) */
    uint16_t txdataptr;    /* 0x06: Data pointer in EM */
    uint16_t mic;          /* 0x08: MIC / CRC seed for encryption */
    uint16_t txrate;       /* 0x0A: TX rate / modulation / power */
    uint16_t txstat;       /* 0x0C: TX status (written by HW) */
} em_bt_txdesc_t;

_Static_assert(sizeof(em_bt_txdesc_t) == 14, "BT TX descriptor must be 14 bytes");

/*
 * BLE TX Descriptor
 */
typedef struct __attribute__((packed)) {
    uint16_t txcntl;       /* 0x00: TX control */
    uint16_t txphce;       /* 0x02: PHY/CTE */
    uint16_t txdataptr;    /* 0x04: Data pointer */
    uint16_t txauxptr;     /* 0x06: Aux pointer */
} em_ble_txdesc_t;

/* 
 * Known EM regions (offsets from 0x3FFB0000)
 * Total EM size is 32KB (0x8000 bytes)
 */
#define EM_NVDS_OFFSET          0x0000  /* NVDS magic at start */
#define EM_BT_RXDESC_OFFSET     0x0800  /* BT RX descriptors */
#define EM_BT_TXDESC_OFFSET     0x1000  /* BT TX descriptors */
#define EM_BLE_RXDESC_OFFSET    0x1800  /* BLE RX descriptors */
#define EM_BLE_TXDESC_OFFSET    0x2000  /* BLE TX descriptors */
#define EM_BLE_CS_OFFSET        0x2800  /* BLE Control Structures */
#define EM_KEY_OFFSET           0x3000  /* Encryption keys */
#define EM_WHITELIST_OFFSET     0x3800  /* Whitelist entries */
#define EM_BT_ET_OFFSET         0x4000  /* BT Exchange Table (16 entries) */
#define EM_SEMA_OFFSET          0x5000  /* Semaphores/Mutexes */
#define EM_HEAP_OFFSET          0x6000  /* BTDM heap / misc structures */
#define EM_RTOS_OFFSET          0x7800  /* FreeRTOS TCB/stack area */
#define EM_END_OFFSET           0x8000  /* End of EM */

/* Debug helper macros */
#define EM_REGION_NAME(off) \
    ((off) < 0x0800 ? "NVDS" : \
     (off) < 0x1000 ? "BT_RXDESC" : \
     (off) < 0x1800 ? "BT_TXDESC" : \
     (off) < 0x2000 ? "BLE_RXDESC" : \
     (off) < 0x2800 ? "BLE_TXDESC" : \
     (off) < 0x3000 ? "BLE_CS" : \
     (off) < 0x3800 ? "KEY" : \
     (off) < 0x4000 ? "WHITELIST" : \
     (off) < 0x5000 ? "BT_ET" : \
     (off) < 0x6000 ? "SEMA" : \
     (off) < 0x7800 ? "HEAP" : \
     (off) < 0x8000 ? "RTOS" : "INVALID")

/*
 * Known RTOS/SEMA field annotations
 * FreeRTOS TCB structure fields and RivieraWaves kernel objects
 */
static inline const char *em_field_comment(uint32_t off, uint32_t val)
{
    /* Spinlock magic values - check these FIRST */
    if (val == 0xb33fffff) {
        return " ; SPINLOCK_FREE";
    }
    if (val == 0x0000cdcd) {
        return " ; SPINLOCK_TAKEN";
    }
    /* Stack canary */
    if (val == 0xa5a5a5a5) {
        return " ; STACK_CANARY";
    }
    /* Check if value looks like a function pointer (bit 31 set = windowed call) */
    if ((val & 0xf0000000) == 0x80000000) {
        return " ; func_ptr";
    }
    /* Check if value looks like an EM pointer (0x3ffb0000-0x3ffb8000) */
    if ((val >= 0x3ffb0000) && (val < 0x3ffb8000)) {
        return " ; em_ptr";
    }
    /* Check if value looks like a DRAM pointer (0x3ffxxxxx) */
    if ((val >= 0x3ff00000) && (val < 0x40000000)) {
        return " ; dram_ptr";
    }
    /* Check if value looks like an IRAM/ROM pointer (0x400xxxxx) */
    if ((val >= 0x40000000) && (val < 0x50000000)) {
        return " ; code_ptr";
    }
    return "";
}

/* Packet type names (for BT header) */
static inline const char *bt_pkt_type_name(uint8_t type)
{
    static const char *names[] = {
        "NULL", "POLL", "FHS", "DM1",
        "DH1/2-DH1", "HV1/2-EV3", "HV2/2-EV5", "HV3/EV3",
        "DV/3-EV3", "AUX1/3-EV5", "DM3/2-DH3", "DH3/3-DH3",
        "EV4/2-EV5", "EV5/3-EV5", "DM5/2-DH5", "DH5/3-DH5"
    };
    return (type < 16) ? names[type] : "INVALID";
}

/* LLID names (for ACL header) */
static inline const char *acl_llid_name(uint8_t llid)
{
    static const char *names[] = {
        "RFU", "CONT", "START", "LMP"
    };
    return names[llid & 3];
}

/* Debug print helper for RX descriptor */
static inline void em_bt_rxdesc_print(const em_bt_rxdesc_t *desc, uint32_t addr)
{
    em_bt_rxstat_t stat = { .val = desc->rxstat };
    em_bt_header_t hdr = { .raw = desc->bt_header };
    em_acl_header_t acl = { .raw = desc->acl_header };
    em_bt_rssi_ch_t rssi = { .val = desc->rssi_ch };
    em_bt_rxctrl_t ctrl = { .val = desc->rxctrl };
    
    qemu_log("BT_RXDESC @ 0x%08x:\n", addr);
    qemu_log("  RXSTAT: 0x%04x (valid=%d crc_err=%d hec_err=%d)\n",
             stat.val, stat.pkt_valid, stat.crc_err, stat.hec_err);
    qemu_log("  BT_HDR: 0x%04x (lt=%d type=%s flow=%d arqn=%d seqn=%d)\n",
             hdr.raw, hdr.lt_addr, bt_pkt_type_name(hdr.type),
             hdr.flow, hdr.arqn, hdr.seqn);
    qemu_log("  ACL_HDR: 0x%04x (llid=%s flow=%d len=%d)\n",
             acl.raw, acl_llid_name(acl.llid), acl.flow, acl.length);
    qemu_log("  DATA_PTR: 0x%04x -> 0x%08x\n",
             desc->data_ptr, EM_BASE_ADDR + (desc->data_ptr & 0x1FFF));
    qemu_log("  RSSI: %d dBm, CH: %d\n", (int8_t)rssi.rssi, rssi.channel);
    qemu_log("  RXCTRL: 0x%04x (enc=%d mic=%d edr=%d fhs=%d)\n",
             ctrl.val, ctrl.encrypted, ctrl.mic_ok, ctrl.is_edr, ctrl.is_fhs);
}

/* Debug print helper for TX descriptor */
static inline void em_bt_txdesc_print(const em_bt_txdesc_t *desc, uint32_t addr)
{
    em_bt_txctrl_t ctrl = { .val = desc->txctrl };
    em_bt_header_t hdr = { .raw = desc->bt_header };
    em_acl_header_t acl = { .raw = desc->acl_header };
    em_bt_txrate_t rate = { .val = desc->txrate };
    em_bt_txstat_t stat = { .val = desc->txstat };
    
    qemu_log("BT_TXDESC @ 0x%08x:\n", addr);
    qemu_log("  TXCTRL: 0x%04x (en=%d enc=%d retry=%d flush=%d lmp=%d poll=%d done=%d ready=%d)\n",
             ctrl.val, ctrl.tx_en, ctrl.encrypt, ctrl.retry_en, ctrl.flush,
             ctrl.is_lmp, ctrl.is_poll, ctrl.tx_done, ctrl.tx_ready);
    qemu_log("  BT_HDR: 0x%04x (lt=%d type=%s flow=%d arqn=%d seqn=%d)\n",
             hdr.raw, hdr.lt_addr, bt_pkt_type_name(hdr.type),
             hdr.flow, hdr.arqn, hdr.seqn);
    qemu_log("  ACL_HDR: 0x%04x (llid=%s flow=%d len=%d)\n",
             acl.raw, acl_llid_name(acl.llid), acl.flow, acl.length);
    qemu_log("  DATA_PTR: 0x%04x -> 0x%08x\n",
             desc->txdataptr, EM_BASE_ADDR + (desc->txdataptr & 0x1FFF));
    qemu_log("  MIC: 0x%04x\n", desc->mic);
    qemu_log("  TXRATE: 0x%04x (rate=%d power=%d edr=%d)\n",
             rate.val, rate.rate, rate.power, rate.is_edr);
    qemu_log("  TXSTAT: 0x%04x (retries=%d ack=%d flush=%d timeout=%d)\n",
             stat.val, stat.retries, stat.acked, stat.flushed, stat.timeout);
}

#endif /* ESP32_BT_DBG_H */
