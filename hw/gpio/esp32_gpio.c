/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "ui/console-priv.h"
#include "hw/hw.h"
#include "ui/input.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"
#include "sysemu/runstate.h"

#define N_GPIOS 40
#define DIRECT_GPIO 2
#define CONSOLE_WIDTH 320
#define CONSOLE_HEIGHT 656

struct gpio_matrix_t {
    int         num;
    const char* in;
    const char* out;
    bool        iomux;
} gpio_matrix[] = { { 0, "SPICLK_in", "SPICLK_out", true },
                    { 1, "SPIQ_in", "SPIQ_out", true },
                    { 2, "SPID_in", "SPID_out", true },
                    { 3, "SPIHD_in", "SPIHD_out", true },
                    { 4, "SPIWP_in", "SPIWP_out", true },
                    { 5, "SPICS0_in", "SPICS0_out", true },
                    { 6, "SPICS1_in", "SPICS1_out", false },
                    { 7, "SPICS2_in", "SPICS2_out", false },
                    { 8, "HSPICLK_in", "HSPICLK_out", true },
                    { 9, "HSPIQ_in", "HSPIQ_out", true },
                    { 10, "HSPID_in", "HSPID_out", true },
                    { 11, "HSPICS0_in", "HSPICS0_out", true },
                    { 12, "HSPIHD_in", "HSPIHD_out", true },
                    { 13, "HSPIWP_in", "HSPIWP_out", true },
                    { 14, "U0RXD_in", "U0TXD_out", true },
                    { 15, "U0CTS_in", "U0RTS_out", true },
                    { 16, "U0DSR_in", "U0DTR_out", false },
                    { 17, "U1RXD_in", "U1TXD_out", true },
                    { 18, "U1CTS_in", "U1RTS_out", true },
                    { 19, "", "", false },
                    { 20, "", "", false },
                    { 21, "", "", false },
                    { 22, "", "", false },
                    { 23, "I2S0O_BCK_in", "I2S0O_BCK_out", false },
                    { 24, "I2S1O_BCK_in", "I2S1O_BCK_out", false },
                    { 25, "I2S0O_WS_in", "I2S0O_WS_out", false },
                    { 26, "I2S1O_WS_in", "I2S1O_WS_out", false },
                    { 27, "I2S0I_BCK_in", "I2S0I_BCK_out", false },
                    { 28, "I2S0I_WS_in", "I2S0I_WS_out", false },
                    { 29, "I2C0_SCL_in", "I2C0_SCL_out", false },
                    { 30, "I2C0_SDA_in", "I2C0_SDA_out", false },
                    { 31, "pwm0_sync0_in", "sdio_tohost_int_out", false },
                    { 32, "pwm0_sync1_in", "pwm0_out0a", false },
                    { 33, "pwm0_sync2_in", "pwm0_out0b", false },
                    { 34, "pwm0_f0_in", "pwm0_out1a", false },
                    { 35, "pwm0_f1_in", "pwm0_out1b", false },
                    { 36, "pwm0_f2_in", "pwm0_out2a", false },
                    { 37, "", "pwm0_out2b", false },
                    { 38, "", "", false },
                    { 39, "pcnt_sig_ch0_in0", "", false },
                    { 40, "pcnt_sig_ch1_in0", "", false },
                    { 41, "pcnt_ctrl_ch0_in0", "", false },
                    { 42, "pcnt_ctrl_ch1_in0", "", false },
                    { 43, "pcnt_sig_ch0_in1", "", false },
                    { 44, "pcnt_sig_ch1_in1", "", false },
                    { 45, "pcnt_ctrl_ch0_in1", "", false },
                    { 46, "pcnt_ctrl_ch1_in1", "", false },
                    { 47, "pcnt_sig_ch0_in2", "", false },
                    { 48, "pcnt_sig_ch1_in2", "", false },
                    { 49, "pcnt_ctrl_ch0_in2", "", false },
                    { 50, "pcnt_ctrl_ch1_in2", "", false },
                    { 51, "pcnt_sig_ch0_in3", "", false },
                    { 52, "pcnt_sig_ch1_in3", "", false },
                    { 53, "pcnt_ctrl_ch0_in3", "", false },
                    { 54, "pcnt_ctrl_ch1_in3", "", false },
                    { 55, "pcnt_sig_ch0_in4", "", false },
                    { 56, "pcnt_sig_ch1_in4", "", false },
                    { 57, "pcnt_ctrl_ch0_in4", "", false },
                    { 58, "pcnt_ctrl_ch1_in4", "", false },
                    { 59, "", "", false },
                    { 60, "", "", false },
                    { 61, "HSPICS1_in", "HSPICS1_out", false },
                    { 62, "HSPICS2_in", "HSPICS2_out", false },
                    { 63, "VSPICLK_in", "VSPICLK_out_mux", true },
                    { 64, "VSPIQ_in", "VSPIQ_out", true },
                    { 65, "VSPID_in", "VSPID_out", true },
                    { 66, "VSPIHD_in", "VSPIHD_out", true },
                    { 67, "VSPIWP_in", "VSPIWP_out", true },
                    { 68, "VSPICS0_in", "VSPICS0_out", true },
                    { 69, "VSPICS1_in", "VSPICS1_out", false },
                    { 70, "VSPICS2_in", "VSPICS2_out", false },
                    { 71, "pcnt_sig_ch0_in5", "ledc_hs_sig_out0", false },
                    { 72, "pcnt_sig_ch1_in5", "ledc_hs_sig_out1", false },
                    { 73, "pcnt_ctrl_ch0_in5", "ledc_hs_sig_out2", false },
                    { 74, "pcnt_ctrl_ch1_in5", "ledc_hs_sig_out3", false },
                    { 75, "pcnt_sig_ch0_in6", "ledc_hs_sig_out4", false },
                    { 76, "pcnt_sig_ch1_in6", "ledc_hs_sig_out5", false },
                    { 77, "pcnt_ctrl_ch0_in6", "ledc_hs_sig_out6", false },
                    { 78, "pcnt_ctrl_ch1_in6", "ledc_hs_sig_out7", false },
                    { 79, "pcnt_sig_ch0_in7", "ledc_ls_sig_out0", false },
                    { 80, "pcnt_sig_ch1_in7", "ledc_ls_sig_out1", false },
                    { 81, "pcnt_ctrl_ch0_in7", "ledc_ls_sig_out2", false },
                    { 82, "pcnt_ctrl_ch1_in7", "ledc_ls_sig_out3", false },
                    { 83, "rmt_sig_in0", "ledc_ls_sig_out4", false },
                    { 84, "rmt_sig_in1", "ledc_ls_sig_out5", false },
                    { 85, "rmt_sig_in2", "ledc_ls_sig_out6", false },
                    { 86, "rmt_sig_in3", "ledc_ls_sig_out7", false },
                    { 87, "rmt_sig_in4", "rmtt_sig_out0", false },
                    { 88, "rmt_sig_in5", "rmtt_sig_out1", false },
                    { 89, "rmt_sig_in6", "rmtt_sig_out2", false },
                    { 90, "rmt_sig_in7", "rmtt_sig_out3", false },
                    { 91, "", "rmtt_sig_out4", false },
                    { 92, "", "rmtt_sig_out5", false },
                    { 93, "", "rmtt_sig_out6", false },
                    { 94, "", "rmtt_sig_out7", false },
                    { 95, "I2C1_SCL_in", "I2C1_SCL_out", false },
                    { 96, "I2C1_SDA_in", "I2C1_SDA_out", false },
                    { 97, "host_card_detect_n_1", "host_ccmd_od_pullup_en_n", false },
                    { 98, "host_card_detect_n_2", "host_rst_n_1", false },
                    { 99, "host_card_write_prt_1", "host_rst_n_2", false },
                    { 100, "host_card_write_prt_2", "gpio_sd0_out", false },
                    { 101, "host_card_int_n_1", "gpio_sd1_out", false },
                    { 102, "host_card_int_n_2", "gpio_sd2_out", false },
                    { 103, "pwm1_sync0_in", "gpio_sd3_out", false },
                    { 104, "pwm1_sync1_in", "gpio_sd4_out", false },
                    { 105, "pwm1_sync2_in", "gpio_sd5_out", false },
                    { 106, "pwm1_f0_in", "gpio_sd6_out", false },
                    { 107, "pwm1_f1_in", "gpio_sd7_out", false },
                    { 108, "pwm1_f2_in", "pwm1_out0a", false },
                    { 109, "pwm0_cap0_in", "pwm1_out0b", false },
                    { 110, "pwm0_cap1_in", "pwm1_out1a", false },
                    { 111, "pwm0_cap2_in", "pwm1_out1b", false },
                    { 112, "pwm1_cap0_in", "pwm1_out2a", false },
                    { 113, "pwm1_cap1_in", "pwm1_out2b", false },
                    { 114, "pwm1_cap2_in", "", false },
                    { 115, "", "", false },
                    { 116, "", "", false },
                    { 117, "", "", false },
                    { 118, "", "", false },
                    { 119, "", "", false },
                    { 120, "", "", false },
                    { 121, "", "", false },
                    { 122, "", "", false },
                    { 123, "", "", false },
                    { 124, "", "", false },
                    { 125, "", "", false },
                    { 126, "", "", false },
                    { 127, "", "", false },
                    { 128, "", "", false },
                    { 129, "", "", false },
                    { 130, "", "", false },
                    { 131, "", "", false },
                    { 132, "", "", false },
                    { 133, "", "", false },
                    { 134, "", "", false },
                    { 135, "", "", false },
                    { 136, "", "", false },
                    { 137, "", "", false },
                    { 138, "", "", false },
                    { 139, "", "", false },
                    { 140, "I2S0I_DATA_in0", "I2S0O_DATA_out0", false },
                    { 141, "I2S0I_DATA_in1", "I2S0O_DATA_out1", false },
                    { 142, "I2S0I_DATA_in2", "I2S0O_DATA_out2", false },
                    { 143, "I2S0I_DATA_in3", "I2S0O_DATA_out3", false },
                    { 144, "I2S0I_DATA_in4", "I2S0O_DATA_out4", false },
                    { 145, "I2S0I_DATA_in5", "I2S0O_DATA_out5", false },
                    { 146, "I2S0I_DATA_in6", "I2S0O_DATA_out6", false },
                    { 147, "I2S0I_DATA_in7", "I2S0O_DATA_out7", false },
                    { 148, "I2S0I_DATA_in8", "I2S0O_DATA_out8", false },
                    { 149, "I2S0I_DATA_in9", "I2S0O_DATA_out9", false },
                    { 150, "I2S0I_DATA_in10", "I2S0O_DATA_out10", false },
                    { 151, "I2S0I_DATA_in11", "I2S0O_DATA_out11", false },
                    { 152, "I2S0I_DATA_in12", "I2S0O_DATA_out12", false },
                    { 153, "I2S0I_DATA_in13", "I2S0O_DATA_out13", false },
                    { 154, "I2S0I_DATA_in14", "I2S0O_DATA_out14", false },
                    { 155, "I2S0I_DATA_in15", "I2S0O_DATA_out15", false },
                    { 156, "", "I2S0O_DATA_out16", false },
                    { 157, "", "I2S0O_DATA_out17", false },
                    { 158, "", "I2S0O_DATA_out18", false },
                    { 159, "", "I2S0O_DATA_out19", false },
                    { 160, "", "I2S0O_DATA_out20", false },
                    { 161, "", "I2S0O_DATA_out21", false },
                    { 162, "", "I2S0O_DATA_out22", false },
                    { 163, "", "I2S0O_DATA_out23", false },
                    { 164, "I2S1I_BCK_in", "I2S1I_BCK_out", false },
                    { 165, "I2S1I_WS_in", "I2S1I_WS_out", false },
                    { 166, "I2S1I_DATA_in0", "I2S1O_DATA_out0", false },
                    { 167, "I2S1I_DATA_in1", "I2S1O_DATA_out1", false },
                    { 168, "I2S1I_DATA_in2", "I2S1O_DATA_out2", false },
                    { 169, "I2S1I_DATA_in3", "I2S1O_DATA_out3", false },
                    { 170, "I2S1I_DATA_in4", "I2S1O_DATA_out4", false },
                    { 171, "I2S1I_DATA_in5", "I2S1O_DATA_out5", false },
                    { 172, "I2S1I_DATA_in6", "I2S1O_DATA_out6", false },
                    { 173, "I2S1I_DATA_in7", "I2S1O_DATA_out7", false },
                    { 174, "I2S1I_DATA_in8", "I2S1O_DATA_out8", false },
                    { 175, "I2S1I_DATA_in9", "I2S1O_DATA_out9", false },
                    { 176, "I2S1I_DATA_in10", "I2S1O_DATA_out10", false },
                    { 177, "I2S1I_DATA_in11", "I2S1O_DATA_out11", false },
                    { 178, "I2S1I_DATA_in12", "I2S1O_DATA_out12", false },
                    { 179, "I2S1I_DATA_in13", "I2S1O_DATA_out13", false },
                    { 180, "I2S1I_DATA_in14", "I2S1O_DATA_out14", false },
                    { 181, "I2S1I_DATA_in15", "I2S1O_DATA_out15", false },
                    { 182, "", "I2S1O_DATA_out16", false },
                    { 183, "", "I2S1O_DATA_out17", false },
                    { 184, "", "I2S1O_DATA_out18", false },
                    { 185, "", "I2S1O_DATA_out19", false },
                    { 186, "", "I2S1O_DATA_out20", false },
                    { 187, "", "I2S1O_DATA_out21", false },
                    { 188, "", "I2S1O_DATA_out22", false },
                    { 189, "", "I2S1O_DATA_out23", false },
                    { 190, "I2S0I_H_SYNC", "", false },
                    { 191, "I2S0I_V_SYNC", "", false },
                    { 192, "I2S0I_H_ENABLE", "", false },
                    { 193, "I2S1I_H_SYNC", "", false },
                    { 194, "I2S1I_V_SYNC", "", false },
                    { 195, "I2S1I_H_ENABLE", "", false },
                    { 196, "", "", false },
                    { 197, "", "", false },
                    { 198, "U2RXD_in", "U2TXD_out", true },
                    { 199, "U2CTS_in", "U2RTS_out", true },
                    { 200, "emac_mdc_i", "emac_mdc_o", false },
                    { 201, "emac_mdi_i", "emac_mdo_o", false },
                    { 202, "emac_crs_i", "emac_crs_o", false },
                    { 203, "emac_col_i", "emac_col_o", false },
                    { 204, "pcmfsync_in", "bt_audio0_irq", false },
                    { 205, "pcmclk_in", "bt_audio1_irq", false },
                    { 206, "pcmdin", "bt_audio2_irq", false },
                    { 207, "", "ble_audio0_irq", false },
                    { 208, "", "ble_audio1_irq", false },
                    { 209, "", "ble_audio2_irq", false },
                    { 210, "", "pcmfsync_out", false },
                    { 211, "", "pcmclk_out", false },
                    { 212, "", "pcmdout", false },
                    { 213, "", "ble_audio_sync0_p", false },
                    { 214, "", "ble_audio_sync1_p", false },
                    { 215, "", "ble_audio_sync2_p", false },
                    { 224, "", "sig_in_func224", false },
                    { 225, "", "sig_in_func225", false },
                    { 226, "", "sig_in_func226", false },
                    { 227, "", "sig_in_func227", false },
                    { 228, "", "sig_in_func228", false },
                    { -1, "", "", false } };

static const char *int_types[8]={"","IRQ Rising","IRQ Falling","IRQ Any","IRQ Low","IRQ High","",""} ;

struct pin_mux {
    int         pinnum;
    int         reset;
    const char* pinname;
    const char* functions[6];
} io_mux_pins[] = {
    { 0, 3,"GPIO0", { "GPIO0", "CLK_OUT1", "GPIO0", "-", "-", "EMAC_TX_CLK" } },
    { 1, 3,"U0TXD", { "U0TXD", "CLK_OUT3", "GPIO1", "-", "-", "EMAC_RXD2" } },
    { 2, 2,"GPIO2", { "GPIO2", "HSPIWP", "GPIO2", "HS2_DATA0", "SD_DATA0", "-" } },
    { 3, 3,"U0RXD", { "U0RXD", "CLK_OUT2", "GPIO3", "-", "-", "-" } },
    { 4, 2,"GPIO4", { "GPIO4", "HSPIHD", "GPIO4", "HS2_DATA1", "SD_DATA1", "EMAC_TX_ER" } },
    { 5, 3,"GPIO5", { "GPIO5", "VSPICS0", "GPIO5", "HS1_DATA6", "-", "EMAC_RX_CLK" } },
    { 6, 3,"SD_CLK", { "SD_CLK", "SPICLK", "GPIO6", "HS1_CLK", "U1CTS", "-" } },
    { 7, 3,"SD_DATA_0", { "SD_DATA0", "SPIQ", "GPIO7", "HS1_DATA0", "U2RTS", "-" } },
    { 8, 3,"SD_DATA_1", { "SD_DATA1", "SPID", "GPIO8", "HS1_DATA1", "U2CTS", "-" } },
    { 9, 3,"SD_DATA_2", { "SD_DATA2", "SPIHD", "GPIO9", "HS1_DATA2", "U1RXD", "-" } },
    { 10, 3,"SD_DATA_3", { "SD_DATA3", "SPIWP", "GPIO10", "HS1_DATA3", "U1TXD", "-" } },
    { 11, 3,"SD_CMD", { "SD_CMD", "SPICS0", "GPIO11", "HS1_CMD", "U1RTS", "-" } },
    { 12, 2,"MTDI", { "MTDI", "HSPIQ", "GPIO12", "HS2_DATA2", "SD_DATA2", "EMAC_TXD3" } },
    { 13, 2,"MTCK", { "MTCK", "HSPID", "GPIO13", "HS2_DATA3", "SD_DATA3", "EMAC_RX_ER" } },
    { 14, 3,"MTMS", { "MTMS", "HSPICLK", "GPIO14", "HS2_CLK", "SD_CLK", "EMAC_TXD2" } },
    { 15, 3,"MTDO", { "MTDO", "HSPICS0", "GPIO15", "HS2_CMD", "SD_CMD", "EMAC_RXD3" } },
    { 16, 1,"GPIO16", { "GPIO16", "-", "GPIO16", "HS1_DATA4", "U2RXD", "EMAC_CLK_OUT1" } },
    { 17, 1,"GPIO17", { "GPIO17", "-", "GPIO17", "HS1_DATA5", "U2TXD", "EMAC_CLK_1801" } },
    { 18, 1,"GPIO18", { "GPIO18", "VSPICLK", "GPIO18", "HS1_DATA7", "-", "-" } },
    { 19, 1,"GPIO19", { "GPIO19", "VSPIQ", "GPIO19", "U0CTS", "-", "EMAC_TXD0" } },
    { 19, 1,"GPIO20", { "GPIO20", "-", "-", "-", "-", "-" } },
    { 21, 1,"GPIO21", { "GPIO21", "VSPIHD", "GPIO21", "-", "-", "EMAC_TX_EN" } },
    { 22, 1,"GPIO22", { "GPIO22", "VSPIWP", "GPIO22", "U0RTS", "-", "EMAC_TXD1" } },
    { 23, 1,"GPIO23", { "GPIO23", "VSPID", "GPIO23", "HS1_STROBE", "-", "-" } },
    { 24, 0,"GPIO24", { "GPIO24", "-", "-", "-", "-", "-" } },
    { 25, 0,"GPIO25", { "GPIO25", "-", "GPIO25", "-", "-", "EMAC_RXD0" } },
    { 26, 0,"GPIO26", { "GPIO26", "-", "GPIO26", "-", "-", "EMAC_RXD1" } },
    { 27, 0,"GPIO27", { "GPIO27", "-", "GPIO27", "-", "-", "EMAC_RX_DV" } },
    { 28, 0,"GPIO28", { "GPIO28", "-", "-", "-", "-", "-" } },
    { 29, 0,"GPIO29", { "GPIO29", "-", "-", "-", "-", "-" } },
    { 30, 0,"GPIO30", { "GPIO30", "-", "-", "-", "-", "-" } },
    { 31, 0,"GPIO31", { "GPIO31", "-", "-", "-", "-", "-" } },
    { 32, 0,"32K_XP", { "GPIO32", "-", "GPIO32", "-", "-", "-" } },
    { 33, 0,"32K_XN", { "GPIO33", "-", "GPIO33", "-", "-", "-" } },
    { 34, 0,"VDET_1", { "GPIO34", "-", "GPIO34", "-", "-", "-" } },
    { 35, 0,"VDET_2", { "GPIO35", "-", "GPIO35", "-", "-", "-" } },
    { 36, 0,"SENSOR_VP", { "GPIO36", "-", "GPIO36", "-", "-", "-" } },
    { 37, 0,"SENSOR_CAPP", { "GPIO37", "-", "GPIO37", "-", "-", "-" } },
    { 38, 0,"SENSOR_CAPN", { "GPIO38", "-", "GPIO38", "-", "-", "-" } },
    { 39, 0,"SENSOR_VN", { "GPIO39", "-", "GPIO39", "-", "-", "-" } },
    { -1, 0,"", { "" } },
};
static const uint8_t GPIO_PIN_MUX_REG_OFFSET[] = {
    0x44,0x88,0x40,0x84,0x48,0x6c,0x60,0x64,0x68,0x54,0x58,0x5c,0x34,0x38,0x30,0x3c,
    0x4c,0x50,0x70,0x74,0x78,0x7c,0x80,0x8c,0x00,0x24,0x28,0x2c,0x00,0x00,0x00,0x00,
    0x1c,0x20,0x14,0x18,0x04,0x08,0x0c,0x10,
};

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
        case A_GPIO_OUT:
            r = s->gpio_out;
            break;
        case A_GPIO_OUT1:
            r = s->gpio_out1;
            break;
        case A_GPIO_ENABLE:
            r = s->gpio_enable;
            break;
        case A_GPIO_ENABLE1:
            r = s->gpio_enable1;
            break;
        case A_GPIO_STRAP:
            r = s->strap_mode;
            break;
        case A_GPIO_IN:
            r = s->gpio_in;
            break;
        case A_GPIO_IN1:
            r = s->gpio_in1;
            break;
        case A_GPIO_STATUS:
            r = s->gpio_status;
            break;
        case A_GPIO_STATUS1:
            r = s->gpio_status1;
            break;
        case A_GPIO_ACPU_INT:
            r = s->gpio_acpu_int;
            break;
        case A_GPIO_PCPU_INT:
            r = s->gpio_pcpu_int;
            break;
        case A_GPIO_ACPU_INT1:
            r = s->gpio_acpu_int1;
            break;
        case A_GPIO_PCPU_INT1:
            r = s->gpio_pcpu_int1;
            break;
        case A_GPIO_PIN_BASE ... A_GPIO_FUNC_IN_SEL_CFG_BASE-4:
            r = s->gpio_pin[(addr - A_GPIO_PIN_BASE) / 4];
            break;
       	case A_GPIO_FUNC_IN_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE-4:
       	    r = s->gpio_in_sel[(addr - A_GPIO_FUNC_IN_SEL_CFG_BASE) / 4];
       	    break;
       	case A_GPIO_FUNC_OUT_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE+0x9c:
       	    r = s->gpio_out_sel[(addr - A_GPIO_FUNC_OUT_SEL_CFG_BASE) / 4];
        default:
            break;
    }
    return r;
}

static int get_triggering(int int_type, int oldval, int val) {
    switch (int_type) {
        case 1:
            return (val > oldval);
        case 2:
            return (val < oldval);
        case 3:
            return (val != oldval);
        case 4:
            return (val == 0);
        case 5:
            return (val == 1);
    }
    return 0;
}
static void set_gpio(void *opaque, int n, int val) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
//    printf("set_gpio %x %x\n",s->rtc_pad_cfg[1],s->rtc_ext_wakeup0);
    if(n==35 && val==0 /*&& (s->rtc_pad_cfg[1]&0x10000000)*/ && ((s->rtc_ext_wakeup0>>27)==5)) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
        qemu_irq_pulse(s->rtc_reset);
        s->rtc_ext_wakeup0=0;
        return;
    }
    if(n==35 && val==0 /*&& vm_get_suspended()*/) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_OTHER, NULL);
        //qemu_irq_pulse(s->rtc_reset);
    }
        
    if (n < 32) {
        int oldval = (s->gpio_in >> n) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in &= ~(1 << n);
        s->gpio_in |= (val << n);
        int irq = get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if (irq && (s->gpio_pin[n] & (1 << 15))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_pcpu_int |= (1 << n);
        }
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // app cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_acpu_int |= (1 << n);
        }
    } else {
        int n1 = n - 32;
        int oldval = (s->gpio_in1 >> n1) & 1;
        int int_type = (s->gpio_pin[n] >> 7) & 7;
        s->gpio_in1 &= ~(1 << n1);
        s->gpio_in1 |= (val << n1);
        int irq = get_triggering(int_type, oldval, val);
        // says bit 16 in the ref manual, is that wrong?
        if (irq && (s->gpio_pin[n] & (1 << 15))) {  // pro cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_pcpu_int1 |= (1 << n1);
        }
        if (irq && (s->gpio_pin[n] & (1 << 13))) {  // app cpu int enable
            qemu_set_irq(s->irq, 1);
            s->gpio_acpu_int1 |= (1 << n1);
        }
    }
    s->redraw=1;
    
}

static void draw_char_x_y(DisplaySurface *surface, int x, int y, unsigned char c, pixman_color_t fgcol) {
    if(x>320/8) return;
    static pixman_image_t *glyphs[256];
    static pixman_color_t bgcol=QEMU_PIXMAN_COLOR_BLACK;
    
    if (!glyphs[c])
            glyphs[c] = qemu_pixman_glyph_from_vgafont(FONT_HEIGHT, vgafont16, c);
    qemu_pixman_glyph_render(glyphs[c], surface->image,
                             &fgcol, &bgcol, x, y, FONT_WIDTH, FONT_HEIGHT);
}
static void draw_string_x_y(DisplaySurface *surface, int x, int y, char *s,  pixman_color_t fgcol) {
    while(*s) {
        draw_char_x_y(surface,x++,y,*s++,fgcol);
    }
}
static void addconnection(char connection[32],const char str[]) {
    if(strlen(connection)>1 && (strlen(str)+strlen(connection))<32) {
        strncat(connection,",",31);
    }
    strncat(connection,str,31);
}
static const pixman_color_t WHITE=QEMU_PIXMAN_COLOR(0xff, 0xff, 0xff);
static const pixman_color_t GREEN=QEMU_PIXMAN_COLOR(0x00, 0xff, 0x00);
static const pixman_color_t GREY=QEMU_PIXMAN_COLOR(0x80, 0x80, 0x80);
static const pixman_color_t YELLOW=QEMU_PIXMAN_COLOR(0xff, 0xff, 0x00);

static void text_console_update(void *obj) {
    Esp32GpioState *s = ESP32_GPIO(obj);
    if(!s->redraw || !qemu_console_is_visible(QEMU_CONSOLE(s->con))) return;
    s->redraw=0;
    char connections[N_GPIOS][32];
    
    DisplaySurface *surface = qemu_console_surface(QEMU_CONSOLE(s->con));
    /* clear screen */
    int bpp = (surface_bits_per_pixel(surface) + 7) >> 3;
    uint8_t *d1 = surface_data(surface);
    for (int y = 0; y < surface_height(surface); y++) {
        memset(d1, 0x00, surface_width(surface) * bpp);
        d1 += surface_stride(surface);
    }

    char str[128];
    draw_string_x_y(surface, 0,0,(char *)"No I O IEOE Connections",YELLOW);
    
    for(int i=0;i<N_GPIOS;i++) {
        connections[i][0]=0;
        snprintf(str,32,"%2d:",i);
        draw_string_x_y(surface, 0,i+1,str,WHITE);
        int op_en=((i<32)?s->gpio_enable>>i:s->gpio_enable1>>(i-32))&1;
        int in=((i<32)?s->gpio_in>>i:s->gpio_in1>>(i-32))&1;
        int out=((i<32)?s->gpio_out>>i:s->gpio_out1>>(i-32))&1;

        int io_mux=GPIO_PIN_MUX_REG_OFFSET[i]/4;
        uint32_t out_sel=FIELD_EX32(s->gpio_out_sel[i],GPIO_FUNC_OUT,SEL);
        uint32_t oen_sel=FIELD_EX32(s->gpio_out_sel[i],GPIO_FUNC_OUT,OEN_SEL);
        uint32_t mux_ie=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_IE);
        uint32_t mux_func=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,MCU_SEL);
        uint32_t pullup=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_WPU);
        uint32_t pulldown=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_WPD);
        uint32_t int_type=FIELD_EX32(s->gpio_pin[i],GPIO_PIN,INT_TYPE);
        uint32_t int_enable=FIELD_EX32(s->gpio_pin[i],GPIO_PIN,INT_ENABLE);

        draw_char_x_y(surface, 3,i+1,in?'1':'0',((oen_sel || mux_ie)&&!op_en)?GREEN:GREY);
        draw_char_x_y(surface, 5,i+1,out?'1':'0',((out_sel==0x100)&&op_en)?GREEN:GREY);
        draw_char_x_y(surface, 7,i+1,mux_ie?'1':'0',WHITE);
        char oe=out_sel?'1':'0';
        if(out_sel!=0x100 && !oen_sel) {
            oe='P';
        } else {
        //    pullup=0;
        //    pulldown=0;
        } 
        draw_char_x_y(surface, 9,i+1,oe,WHITE);

        if(mux_func==DIRECT_GPIO) {
            if(out_sel==0x100 && op_en) {
                snprintf(str,32,"GPIO%2d",i);
                addconnection(connections[i],str);
            } else {
                if(out_sel<229)
                    addconnection(connections[i],gpio_matrix[out_sel].out);
            }
        } else {
            if(mux_func<6 /*&& op_en*/)
                addconnection(connections[i],(char *)io_mux_pins[i].functions[mux_func]);
        }
    //    printf("%d: out_sel=%x iomux=%x pin=%x op_en=%d oen_sel=%d mux_func=%d mix_ie=%d\n",i,s->gpio_out_sel[i],s->iomux_regs[io_mux],s->gpio_pin[i],op_en,oen_sel,mux_func,mux_ie);

   //     if((oen_sel || mux_ie) && !op_en /*&& mux_func!=2*/) {
   //         addconnection(connections[i],(char *)io_mux_pins[i].functions[mux_func]);
   //     }
        if(pullup) addconnection(connections[i],"PU");
        if(pulldown) addconnection(connections[i],"PD");
        if(int_enable) addconnection(connections[i],int_types[int_type]);
    }
    
    int i=0;
    while(gpio_matrix[i].num>=0) {
        int n=gpio_matrix[i].num;
        int in_sel=FIELD_EX32(s->gpio_in_sel[n],GPIO_FUNC_IN,SEL);
        int sig_sel=FIELD_EX32(s->gpio_in_sel[n],GPIO_FUNC_IN,SIG_SEL);
        uint32_t io_mux=GPIO_PIN_MUX_REG_OFFSET[in_sel]/4;
        uint32_t mux_func=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,MCU_SEL);
        if(in_sel<N_GPIOS && mux_func==DIRECT_GPIO) {
      //      uint32_t oen_sel=FIELD_EX32(s->gpio_out_sel[in_sel],GPIO_FUNC_OUT,OEN_SEL);
      //      uint32_t io_mux=GPIO_PIN_MUX_REG_OFFSET[in_sel]/4;
      //      uint32_t mux_ie=FIELD_EX32(s->iomux_regs[io_mux],IO_MUX,FUN_IE);
      //      int op_en=((i<32)?s->gpio_enable>>i:s->gpio_enable1>>(i-32))&1;
            
            if(sig_sel/* && (!oen_sel ||(oen_sel && !op_en)) && mux_ie*/) {
                snprintf(str,32,"%s",gpio_matrix[i].in);
                addconnection(connections[in_sel],str);
            } 
        }
        i++;
    }
    for(i=0;i<N_GPIOS;i++) {
        str[0]=0;
        strcat(str,(char *)(connections[i]));
        draw_string_x_y(surface, 11, i+1, str,WHITE);
    }
    dpy_gfx_update_full(QEMU_CONSOLE(s->con));
}
static void text_console_invalidate(void *obj) {
    Esp32GpioState *s = ESP32_GPIO(obj);
    s->redraw = 1;
}

static const GraphicHwOps text_console_ops = {
    .invalidate  = text_console_invalidate,
    .gfx_update = text_console_update,
};

static uint64_t esp32_iomux_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int n=addr/4;
    if(n<N_GPIOS) {
//        printf("IOMUX read %lx %x %d\n",addr, s->iomux_regs[n], size);
        return s->iomux_regs[n];
    }
    return 0;
}

static void esp32_iomux_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int n=addr/4;
    if(n<N_GPIOS) {
        s->iomux_regs[n]=value;
    }
//    printf("IOMUX write %lx %lx %d\n",addr,value, size);
}


static uint64_t esp32_rtc_read(void *opaque, hwaddr addr, unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r=0;
    
    switch (addr) {
        case A_RTC_GPIO_OUT:
            r=s->rtc_gpio_out;
            break;
        case A_RTC_GPIO_IN:
            r=s->rtc_gpio_in;
            break;
        case A_RTC_GPIO_PIN ... A_RTC_GPIO_PIN+17*4:
            r=s->rtc_gpio_pin[(addr-A_RTC_GPIO_PIN)/4];
            break;
        case A_RTC_PAD_CFG ... A_RTC_PAD_CFG+15*4:
            r=s->rtc_pad_cfg[(addr-A_RTC_PAD_CFG)/4];
            break;
        case A_RTC_DIG_PAD_HOLD:
            r=s->rtc_dig_pad_hold;
            break;
        case A_RTC_EXT_WAKEUP0:
            r=s->rtc_ext_wakeup0;
            break;
    }
//    printf("RTC read %lx=%lx\n",addr,r);
    return r;
}

static void esp32_rtc_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
//    printf("RTC write %lx %lx\n",addr,value);
    switch (addr) {
        case A_RTC_GPIO_OUT:
            s->rtc_gpio_out = value;
            s->rtc_gpio_in = (s->rtc_gpio_in & ~s->rtc_gpio_enable) | (value & s->rtc_gpio_enable);
            break;
        case A_RTC_GPIO_OUT_W1TS:
            s->rtc_gpio_out |= value;
            s->rtc_gpio_in |= value;
            break;
        case A_RTC_GPIO_OUT_W1TC:
            s->rtc_gpio_out &= ~value;
            s->rtc_gpio_in &= ~value;
            break;
        case A_RTC_GPIO_ENABLE:
        	s->rtc_gpio_enable = value;
        	break;
        case A_RTC_GPIO_ENABLE_W1TS:
        	s->rtc_gpio_enable |= value;
        	break;
        case A_RTC_GPIO_ENABLE_W1TC:
        	s->rtc_gpio_enable &= ~value;
        	break;
        case A_RTC_GPIO_PIN ... A_RTC_GPIO_PIN+17*4:
            s->rtc_gpio_pin[(addr-A_RTC_GPIO_PIN)/4] = value;
            break;
        case A_RTC_PAD_CFG ... A_RTC_PAD_CFG+15*4:
            s->rtc_pad_cfg[(addr-A_RTC_PAD_CFG)/4] = value;
            break;
        case A_RTC_DIG_PAD_HOLD:
            s->rtc_dig_pad_hold = value;
            break;
        case A_RTC_EXT_WAKEUP0:
            s->rtc_ext_wakeup0 = value;
            break;
    }
   // s->rtcio_regs[addr/4]=value;
}

static void esp32_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                             unsigned int size) {
    Esp32GpioState *s = ESP32_GPIO(opaque);
    int clearirq;
    uint32_t oldvalue = s->gpio_out;
    uint32_t oldvalue1 = s->gpio_out1;
    switch (addr) {
        case A_GPIO_OUT:
            s->gpio_out = value;
            s->gpio_in = (s->gpio_in & ~s->gpio_enable) | (value & s->gpio_enable);
            break;
        case A_GPIO_OUT1:
            s->gpio_out1 = value;
            s->gpio_in1 = (s->gpio_in1 & ~s->gpio_enable1) | (value & s->gpio_enable1);
            break;
        case A_GPIO_OUT_W1TS:
            s->gpio_out |= value;
            s->gpio_in |= value;
            break;
        case A_GPIO_OUT_W1TC:
            s->gpio_out &= ~value;
            s->gpio_in &= ~value;
            break;
        case A_GPIO_OUT1_W1TS:
            s->gpio_out1 |= value;
            s->gpio_in1 |= value;
            break;
        case A_GPIO_OUT1_W1TC:
            s->gpio_out1 &= ~value;
            s->gpio_in1 &= ~value;
            break;
        case A_GPIO_ENABLE:
        	s->gpio_enable = value;
        	break;
        case A_GPIO_ENABLE1:
        	s->gpio_enable1 = value;
        	break;
        case A_GPIO_ENABLE_W1TS:
        	s->gpio_enable |= value;
        	break;
        case A_GPIO_ENABLE_W1TC:
        	s->gpio_enable &= ~value;
        	break;
        case A_GPIO_ENABLE1_W1TS:
        	s->gpio_enable1 |= value;
        	break;
        case A_GPIO_ENABLE1_W1TC:
        	s->gpio_enable1 &= ~value;
        	break;
        case A_GPIO_STRAP:
            s->strap_mode = value;
            break;
        case A_GPIO_STATUS:
            s->gpio_status = value;
            break;
        case A_GPIO_STATUS_W1TS:
            s->gpio_status |= value;
            break;
        case A_GPIO_STATUS_W1TC:
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status &= ~value;
                s->gpio_pcpu_int &= ~value;
                s->gpio_acpu_int &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
        case A_GPIO_STATUS1:
            s->gpio_status1 = value;
            break;
        case A_GPIO_STATUS1_W1TS:
            s->gpio_status1 |= value;
            break;
        case A_GPIO_STATUS1_W1TC:
            clearirq = 1;
            for (int i = 0; i < 32; i++) {
                if ((1 << i) & value) {
                    int int_type = (s->gpio_pin[i + 32] >> 7) & 7;
                    if ((int_type == 4 && !(s->gpio_in1 & (1 << i))) ||
                        (int_type == 5 && (s->gpio_in1 & (1 << i))))
                        clearirq = 0;
                }
            }
            if (clearirq) {
                s->gpio_status1 &= ~value;
                s->gpio_pcpu_int1 &= ~value;
                s->gpio_acpu_int1 &= ~value;
                qemu_set_irq(s->irq, 0);
            }
            break;
        case A_GPIO_PIN_BASE ... A_GPIO_FUNC_IN_SEL_CFG_BASE-4:
            s->gpio_pin[(addr - A_GPIO_PIN_BASE) / 4] = value;
            break;
        case A_GPIO_FUNC_IN_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE-4:
            s->gpio_in_sel[(addr - A_GPIO_FUNC_IN_SEL_CFG_BASE)/4] = value;
            break;
        case A_GPIO_FUNC_OUT_SEL_CFG_BASE ... A_GPIO_FUNC_OUT_SEL_CFG_BASE+0x9c:
            s->gpio_out_sel[(addr - A_GPIO_FUNC_OUT_SEL_CFG_BASE)/4] = value;
            break;
    }

    if (s->gpio_out != oldvalue) {
        uint32_t diff = (s->gpio_out ^ oldvalue);
        for (int i = 0; i < 32; i++) {
            if ((1 << i) & diff) {
                if(i!=16) {
                    s->redraw=1;
                }
                qemu_set_irq(s->gpios[i], (s->gpio_out & (1 << i)) ? 1 : 0);
            }
        }
    }
    if (s->gpio_out1 != oldvalue1) {
        uint32_t diff = (s->gpio_out1 ^ oldvalue1);
        for (int i = 0; i < 16; i++) {
            if ((1 << i) & diff) {
                qemu_set_irq(s->gpios[i+32], (s->gpio_out1 & (1 << i)) ? 1 : 0);
            }
        }
    }

    
}

static const MemoryRegionOps gpio_ops = {
    .read = esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps iomux_ops = {
    .read = esp32_iomux_read,
    .write = esp32_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static const MemoryRegionOps rtc_ops = {
    .read = esp32_rtc_read,
    .write = esp32_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(Object *dev, ResetType type) {
    Esp32GpioState *s = ESP32_GPIO(dev);
    for(int i=0;i<256;i++) {
        s->gpio_in_sel[i]=0x30;
    }
    for(int i=0;i<N_GPIOS;i++) {
        s->gpio_out_sel[i]=0;
        s->gpio_pin[i]=0;
        int reset_type=io_mux_pins[i].reset;
        int v=0x800;
        if(reset_type==1 || reset_type==2 || reset_type==3) {
            v=FIELD_DP32(v,IO_MUX,FUN_IE,1);
        }
        if(reset_type==2) v=FIELD_DP32(v,IO_MUX,FUN_WPD,1);
        if(reset_type==3 || reset_type==4) v=FIELD_DP32(v,IO_MUX,FUN_WPU,1);
        s->iomux_regs[GPIO_PIN_MUX_REG_OFFSET[i]/4]=v;
    }

    s->gpio_in = 0x1;
    s->gpio_in1 = 0x8;
    s->gpio_out = 0x0;
    s->gpio_out1 = 0x0;
    s->gpio_enable = 0x0;
    s->gpio_enable1 = 0x0;

 //   for(int i=0;i<64;i++) {
 //       s->rtcio_regs[i]=0;
 //   }
 //   s->rtcio_regs[35] = 0x84100010;
}

static void func_gpio(void *opaque, int n, int val) {
	Esp32GpioState *s = ESP32_GPIO(opaque);
	int func=(val>>1)&0x1ff;
	int v=val & 1;
	int param=((unsigned)val)>>10;
	for(int i=0;i<N_GPIOS;i++) {
		if((s->gpio_out_sel[i] & 0x1ff ) == func) {
			qemu_set_irq(s->gpios[i], v+(param<<1));
		}
	}
}




static void esp32_gpio_realize(DeviceState *dev, Error **errp) {   
    Esp32GpioState *s = ESP32_GPIO(dev);
    s->con=QEMU_CONSOLE(object_new(TYPE_QEMU_FIXED_TEXT_CONSOLE));
    s->con->hw_ops = &text_console_ops;
    s->con->hw = s;
    dpy_gfx_replace_surface(QEMU_CONSOLE(s->con), qemu_create_displaysurface(CONSOLE_WIDTH,CONSOLE_HEIGHT));
}


static void esp32_gpio_init(Object *obj) {
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(s);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    memory_region_init_io(&s->iomuxmem, obj, &iomux_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    memory_region_init_io(&s->iortcmem, obj, &rtc_ops, s,
                          TYPE_ESP32_GPIO, 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->iomuxmem);
    sysbus_init_mmio(sbd, &s->iortcmem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->irq, SYSBUS_DEVICE_GPIO_IRQ, 1);
    qdev_init_gpio_out_named(dev, s->gpios, ESP32_GPIOS, 32);
    qdev_init_gpio_out_named(dev, &s->rtc_reset, ESP32_RTCIO_RESET_GPIO, 1);
    qdev_init_gpio_in_named(dev, set_gpio, ESP32_GPIOS_IN, N_GPIOS);
    qdev_init_gpio_in_named(dev, func_gpio, ESP32_GPIOS_FUNC,1);
    s->gpio_in = 0x1;
    s->gpio_in1 = 0x8;
}



static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.enter = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void) {
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
