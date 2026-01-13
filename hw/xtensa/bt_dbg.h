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
 * RivieraWaves Kernel Element (ke_elem) / Activity structure
 * Base structure for scheduler entries (ADV, SCAN, CONN, etc.)
 * Size varies by activity type, but header is common.
 */
typedef struct __attribute__((packed)) {
    uint32_t next;          /* 0x00: Next element in linked list (EM addr or sentinel) */
    uint32_t prev;          /* 0x04: Previous element */
    uint32_t time;          /* 0x08: Scheduling time / priority */
    uint32_t type;          /* 0x0C: Activity type / state */
} rwip_elt_t;

/*
 * BLE Control Structure (Activity descriptor)
 * Located in EM at 0x2800+, multiple entries with variable size
 * Sentinel value: 0x3ffb2744 (empty list pointer)
 */
typedef struct __attribute__((packed)) {
    /* List management (0x00-0x0F) */
    uint32_t next;          /* 0x00: Next CS in linked list */
    uint32_t prev;          /* 0x04: Previous CS */
    uint32_t time;          /* 0x08: Target time for event */
    uint32_t type_state;    /* 0x0C: Activity type + state */
    
    /* Activity parameters (0x10+) - varies by type */
    uint32_t interval;      /* 0x10: Event interval */
    uint32_t duration;      /* 0x14: Duration */
    uint32_t aux_ptr;       /* 0x18: Aux data pointer */
    uint32_t txdesc_ptr;    /* 0x1C: TX descriptor pointer */
    uint32_t rxdesc_ptr;    /* 0x20: RX descriptor pointer */
    uint32_t cs_ptr;        /* 0x24: Control structure pointer */
    uint32_t env_ptr;       /* 0x28: Environment pointer */
    uint32_t callback;      /* 0x2C: Completion callback */
} rwip_cs_t;

/* BLE_CS sentinel value (empty list marker) */
#define BLE_CS_SENTINEL     0x3ffb2744

/* Activity types (based on RivieraWaves) */
#define RWIP_ACT_ADV        0x00    /* Advertising */
#define RWIP_ACT_SCAN       0x01    /* Scanning */
#define RWIP_ACT_INIT       0x02    /* Initiating */
#define RWIP_ACT_CONN       0x03    /* Connection */

/*
 * RivieraWaves co_list header structure (from reversed code)
 */
typedef struct __attribute__((packed)) {
    uint32_t head;      /* 0x00: First element pointer */
    uint32_t tail;      /* 0x04: Last element pointer */
    uint32_t count;     /* 0x08: Current element count */
    uint32_t max_count; /* 0x0C: Maximum element count (watermark) */
} co_list_t;

/*
 * ESP32 BLE OSI (OS Interface) function table
 * Located at EM offset 0x2960 - provides FreeRTOS abstraction to BLE ROM
 * These are wrapper functions for the BLE stack to call FreeRTOS APIs
 */
typedef struct __attribute__((packed)) {
    uint32_t version;               /* 0x00: OSI version/magic */
    uint32_t set_isr;               /* 0x04: Register ISR handler */
    uint32_t ints_on;               /* 0x08: Enable interrupts */
    uint32_t ints_off;              /* 0x0C: Disable interrupts */
    uint32_t task_create;           /* 0x10: Create task */
    uint32_t task_delete;           /* 0x14: Delete task */
    uint32_t task_yield;            /* 0x18: Yield to scheduler */
    uint32_t task_yield_from_isr;   /* 0x1C: Yield from ISR */
    uint32_t semphr_create;         /* 0x20: Create semaphore */
    uint32_t semphr_delete;         /* 0x24: Delete semaphore */
    uint32_t semphr_take;           /* 0x28: Take semaphore */
    uint32_t semphr_give;           /* 0x2C: Give semaphore */
    uint32_t semphr_take_from_isr;  /* 0x30: Take from ISR */
    uint32_t semphr_give_from_isr;  /* 0x34: Give from ISR */
    uint32_t mutex_create;          /* 0x38: Create mutex */
    uint32_t mutex_delete;          /* 0x3C: Delete mutex */
    uint32_t mutex_lock;            /* 0x40: Lock mutex */
    uint32_t mutex_unlock;          /* 0x44: Unlock mutex */
    uint32_t queue_create;          /* 0x48: Create queue */
    uint32_t queue_delete;          /* 0x4C: Delete queue */
    uint32_t queue_send;            /* 0x50: Send to queue */
    uint32_t queue_send_from_isr;   /* 0x54: Send from ISR */
    uint32_t queue_recv;            /* 0x58: Receive from queue */
    uint32_t queue_recv_from_isr;   /* 0x5C: Receive from ISR */
    uint32_t timer_create;          /* 0x60: Create timer */
    uint32_t timer_delete;          /* 0x64: Delete timer */
    uint32_t timer_start;           /* 0x68: Start timer */
    uint32_t timer_stop;            /* 0x6C: Stop timer */
    /* ... more functions ... */
} ble_osi_funcs_t;

/*
 * BLE CS layout (corrected based on log analysis):
 * - 0x2800-0x28CB: Queue headers (multiple co_list_t structures)
 * - 0x28CC-0x295F: CS entries (variable structure, ~0x30 bytes each)
 * - 0x2960-0x2A5F: OSI function table (FreeRTOS wrappers for BLE ROM)
 * - 0x2A60+: More CS entries or data structures
 */
#define BLE_CS_QUEUE_BASE   0x2800  /* Queue headers start */
#define BLE_CS_FIRST_ENTRY  0x28CC  /* First actual CS entry (EM offset) */
#define BLE_CS_STRIDE       0x30    /* 48 bytes per CS entry */
#define BLE_CS_HDR_SIZE     0xCC    /* Queue headers before first CS */

/* OSI function table base (EM offset) - FreeRTOS wrappers */
#define BLE_OSI_FUNCS_BASE  0x2960  /* Start of OSI function pointer table */
#define BLE_OSI_FUNCS_END   0x2A60  /* End of OSI function pointer table (64 functions x 4 bytes) */

/* 
 * Known EM regions (offsets from 0x3FFB0000)
 * Total EM size is 32KB (0x8000 bytes)
 * 
 * Layout derived from ESP32 BT firmware analysis:
 * - BT Link Control blocks: 0x1DEE+ (stride 0x66 = 102 bytes per link)
 * - BT RX descriptors: 0x2382+ (stride 0x0E = 14 bytes, 4 entries)
 * - BT TX descriptors: 0x23BA+ (stride 0x14 = 20 bytes per link*2)
 * - FHS packet buffer: 0x262C
 */
#define EM_NVDS_OFFSET          0x0000  /* NVDS magic at start */
#define EM_BT_LINKCTRL_OFFSET   0x1D00  /* BT Link Control blocks (0x66 bytes each) */
#define EM_BT_RXDESC_OFFSET     0x2382  /* BT RX descriptors (4 x 14 bytes) */
#define EM_BT_TXDESC_OFFSET     0x23BA  /* BT TX descriptors (stride 0x14) */
#define EM_BT_FHS_OFFSET        0x262C  /* FHS packet buffer */
#define EM_BLE_CS_OFFSET        0x2800  /* BLE Control Structures */
#define EM_KEY_OFFSET           0x3000  /* Encryption keys */
#define EM_WHITELIST_OFFSET     0x3800  /* Whitelist entries */
#define EM_BT_ET_OFFSET         0x4000  /* BT Exchange Table (16 entries) */
#define EM_SEMA_OFFSET          0x5000  /* Semaphores/Mutexes */
#define EM_HEAP_OFFSET          0x6000  /* BTDM heap / misc structures */
#define EM_RTOS_OFFSET          0x7800  /* FreeRTOS TCB/stack area */
#define EM_END_OFFSET           0x8000  /* End of EM */

/* BT TX descriptor field offsets (from descriptor base) */
#define EM_BT_TXDESC_CTRL_OFF   0x00  /* 0x23BA: TX control (bit 15 = tx_ready) */
#define EM_BT_TXDESC_BTHDR_OFF  0x02  /* 0x23BC: BT header */
#define EM_BT_TXDESC_ACLHDR_OFF 0x04  /* 0x23BE: ACL header */
#define EM_BT_TXDESC_DATAPTR_OFF 0x06 /* 0x23C0: Data pointer */
#define EM_BT_TXDESC_LMPPTR_OFF 0x08  /* 0x23C2: LMP pointer */
#define EM_BT_TXDESC_STRIDE     0x14  /* 20 bytes per descriptor pair */

/* BT RX descriptor field offsets */
#define EM_BT_RXDESC_STRIDE     0x0E  /* 14 bytes per descriptor */

/* BT Link Control block offsets (from 0x1DEE base, stride 0x66) */
#define EM_BT_LC_STRIDE         0x66  /* 102 bytes per link */

/*
 * BLE CS field name lookup function
 * Handles: queue headers (0x2800-0x28CB), CS entries (0x28CC-0x295F), 
 * OSI function table (0x2960-0x2A5F), and post-OSI data (0x2A60+)
 */
static inline const char *ble_cs_field_name(uint32_t addr, int *out_cs_idx, int *out_is_osi)
{
    uint32_t off = addr;  /* Already EM offset */
    
    *out_cs_idx = -1;
    *out_is_osi = 0;
    
    /* Check if this is in the OSI function table region */
    if (off >= BLE_OSI_FUNCS_BASE && off < BLE_OSI_FUNCS_END) {
        *out_is_osi = 1;
        uint32_t osi_off = off - BLE_OSI_FUNCS_BASE;
        *out_cs_idx = osi_off / 4;  /* Function index */
        
        /* ESP-IDF BLE OSI function names (from osi_funcs_t) */
        static const char *osi_names[] = {
            "set_isr",              /* 0: set_isr_hlevel_wrapper */
            "ints_on",              /* 1: xt_ints_on */
            "int_disable",          /* 2: interrupt_hlevel_disable */
            "int_restore",          /* 3: interrupt_hlevel_restore */
            "task_yield",           /* 4: task_yield */
            "task_yield_isr",       /* 5: task_yield_from_isr */
            "semphr_create",        /* 6: semphr_create_wrapper */
            "semphr_delete",        /* 7: semphr_delete_wrapper */
            "semphr_take_isr",      /* 8: semphr_take_from_isr_wrapper */
            "semphr_give_isr",      /* 9: semphr_give_from_isr_wrapper */
            "semphr_take",          /* 10: semphr_take_wrapper */
            "semphr_give",          /* 11: semphr_give_wrapper */
            "mutex_create",         /* 12: mutex_create_wrapper */
            "mutex_delete",         /* 13: mutex_delete_wrapper */
            "mutex_lock",           /* 14: mutex_lock_wrapper */
            "mutex_unlock",         /* 15: mutex_unlock_wrapper */
            "queue_create",         /* 16: queue_create_hlevel_wrapper */
            "queue_delete",         /* 17: queue_delete_hlevel_wrapper */
            "queue_send",           /* 18: queue_send_hlevel_wrapper */
            "queue_send_isr",       /* 19: queue_send_from_isr_hlevel_wrapper */
            "queue_recv",           /* 20: queue_recv_hlevel_wrapper */
            "queue_recv_isr",       /* 21: queue_recv_from_isr_hlevel_wrapper */
            "task_create",          /* 22: task_create_wrapper */
            "task_delete",          /* 23: task_delete_wrapper */
            "is_in_isr",            /* 24: is_in_isr_wrapper */
            "cause_sw_intr",        /* 25: cause_sw_intr_to_core_wrapper */
            "malloc",               /* 26: valloc */
            "malloc_internal",      /* 27: malloc_internal_wrapper */
            "free",                 /* 28: free */
            "read_mac",             /* 29: read_mac_wrapper */
            "srand",                /* 30: srand_wrapper */
            "rand",                 /* 31: rand_wrapper */
            "lpcycles_2_us",        /* 32: btdm_lpcycles_2_us */
            "us_2_lpcycles",        /* 33: btdm_us_2_lpcycles */
            "sleep_check",          /* 34: btdm_sleep_check_duration */
            "sleep_enter_p1",       /* 35: btdm_sleep_enter_phase1_wrapper */
            "sleep_enter_p2",       /* 36: btdm_sleep_enter_phase2_wrapper */
            "reserved_37",          /* 37: (reserved/unused) */
            "reserved_38",          /* 38: (reserved/unused) */
            "sleep_exit_p3",        /* 39: btdm_sleep_exit_phase3_wrapper */
            "bt_wakeup_req",        /* 40: coex_bt_wakeup_request */
            "bt_wakeup_end",        /* 41: coex_bt_wakeup_request_end */
            "coex_bt_req",          /* 42: coex_bt_request_wrapper */
            "coex_bt_rel",          /* 43: coex_bt_release_wrapper */
            "coex_reg_bt_cb",       /* 44: coex_register_bt_cb_wrapper */
            "coex_bb_lock",         /* 45: coex_bb_reset_lock_wrapper */
            "coex_bb_unlock",       /* 46: coex_bb_reset_unlock_wrapper */
            "coex_schm_reg",        /* 47: coex_schm_register_btdm_callback_wrapper */
            "coex_stat_clr",        /* 48: coex_schm_status_bit_clear_wrapper */
            "coex_stat_set",        /* 49: coex_schm_status_bit_set_wrapper */
            "coex_intv_get",        /* 50: coex_schm_interval_get_wrapper */
            "coex_period_get",      /* 51: coex_schm_curr_period_get_wrapper */
            "coex_phase_get",       /* 52: coex_schm_curr_phase_get_wrapper */
            "wifi_chan_get",        /* 53: coex_wifi_channel_get_wrapper */
            "wifi_chan_cb_reg",     /* 54: coex_register_wifi_channel_change_callback_wrapper */
            "set_int_handler",      /* 55: xt_set_interrupt_handler */
            "int_l3_disable",       /* 56: interrupt_l3_disable */
            "int_l3_restore",       /* 57: interrupt_l3_restore */
            "cust_queue_create",    /* 58: customer_queue_create_hlevel_wrapper */
            "coex_version_get",     /* 59: coex_version_get_wrapper */
            "patch_apply",          /* 60: patch_apply */
        };
        
        if (*out_cs_idx < (int)(sizeof(osi_names)/sizeof(osi_names[0]))) {
            return osi_names[*out_cs_idx];
        }
        return NULL;
    }
    
    /* Check if this is in the queue header region (before first CS) */
    if (off >= EM_BLE_CS_OFFSET && off < BLE_CS_FIRST_ENTRY) {
        uint32_t hdr_off = off - EM_BLE_CS_OFFSET;  /* Relative to 0x2800 */
        *out_cs_idx = -1;  /* Queue header, not a CS entry */
        
        /* Multiple co_list_t structures (16 bytes each) */
        uint32_t list_idx = hdr_off / sizeof(co_list_t);
        uint32_t field_off = hdr_off % sizeof(co_list_t);
        
        static const char *list_fields[] = { "head", "tail", "count", "max" };
        static const char *list_names[] = {
            "ready", "alarm", "scan", "adv", "conn", "init",
            "prog", "wait", "defer", "pend", "done", "free"
        };
        
        if (list_idx < 12 && (field_off / 4) < 4) {
            /* Return combined name like "ready.head" or "alarm.tail" */
            static char buf[32];
            snprintf(buf, sizeof(buf), "%s.%s", list_names[list_idx], list_fields[field_off / 4]);
            return buf;
        }
        return NULL;
    }
    
    /* Check if this is in the CS entry region (between queue headers and OSI table) */
    if (off >= BLE_CS_FIRST_ENTRY && off < BLE_OSI_FUNCS_BASE) {
        uint32_t cs_rel = off - BLE_CS_FIRST_ENTRY;  /* Relative to first CS */
        *out_cs_idx = cs_rel / BLE_CS_STRIDE;
        uint32_t field_off = cs_rel % BLE_CS_STRIDE;
        
        /* rwip_cs_t structure (48 bytes = 0x30) */
        switch (field_off) {
            case 0x00: return "next";       /* Next element in list */
            case 0x04: return "prev";       /* Previous element */
            case 0x08: return "parent";     /* Parent activity */
            case 0x0C: return "cb";         /* Callback function */
            case 0x10: return "env";        /* Environment pointer */
            case 0x14: return "state";      /* Activity state */
            case 0x18: return "time";       /* Target time */
            case 0x1C: return "interval";   /* Event interval */
            case 0x20: return "rxdesc";     /* RX descriptor pointer */
            case 0x24: return "txdesc";     /* TX descriptor pointer */
            case 0x28: return "flags";      /* Flags */
            case 0x2C: return "type";       /* Activity type */
            default:
                return NULL;
        }
    }
    
    /* Post-OSI region (0x2A60+) - more CS entries or data */
    if (off >= BLE_OSI_FUNCS_END && off < EM_KEY_OFFSET) {
        uint32_t post_rel = off - BLE_OSI_FUNCS_END;
        *out_cs_idx = 8 + (post_rel / BLE_CS_STRIDE);  /* Continue CS numbering */
        uint32_t field_off = post_rel % BLE_CS_STRIDE;
        
        switch (field_off) {
            case 0x00: return "next";
            case 0x04: return "prev";
            case 0x08: return "parent";
            case 0x0C: return "cb";
            case 0x10: return "env";
            case 0x14: return "state";
            case 0x18: return "time";
            case 0x1C: return "interval";
            case 0x20: return "rxdesc";
            case 0x24: return "txdesc";
            case 0x28: return "flags";
            case 0x2C: return "type";
            default:
                return NULL;
        }
    }
    
    return NULL;
}

/* Debug helper macros */
#define EM_REGION_NAME(off) \
    ((off) < 0x1D00 ? "NVDS" : \
     (off) < 0x2382 ? "BT_LC" : \
     (off) < 0x23BA ? "BT_RXDESC" : \
     (off) < 0x2600 ? "BT_TXDESC" : \
     (off) < 0x2800 ? "BT_FHS" : \
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
#if 1  
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
#endif
    (void)desc; (void)addr;  /* Suppress unused warnings */
}

/* Debug print helper for TX descriptor */
static inline void em_bt_txdesc_print(const em_bt_txdesc_t *desc, uint32_t addr)
{
#if 1  /* Disabled - too verbose */
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
#endif
    (void)desc; (void)addr;  /* Suppress unused warnings */
}

#endif /* ESP32_BT_DBG_H */
