#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "ui/console.h"
#include "ui/console-priv.h"


#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

#define ESP32_RTCIO_RESET_GPIO   "rtcio-cpu-reset"

// GPIO Registers
REG32(GPIO_OUT, 0x04)
REG32(GPIO_OUT_W1TS, 0x08)
REG32(GPIO_OUT_W1TC, 0x0c)
REG32(GPIO_OUT1, 0x10)
REG32(GPIO_OUT1_W1TS, 0x14)
REG32(GPIO_OUT1_W1TC, 0x18)
REG32(GPIO_ENABLE, 0x020)
REG32(GPIO_ENABLE_W1TS, 0x024)
REG32(GPIO_ENABLE_W1TC, 0x028)
REG32(GPIO_ENABLE1, 0x02c)
REG32(GPIO_ENABLE1_W1TS, 0x030)
REG32(GPIO_ENABLE1_W1TC, 0x034)
REG32(GPIO_STRAP, 0x0038)
REG32(GPIO_IN, 0x003c)
REG32(GPIO_IN1, 0x0040)
REG32(GPIO_STATUS, 0x0044)
REG32(GPIO_STATUS_W1TS, 0x0048)
REG32(GPIO_STATUS_W1TC, 0x004c)
REG32(GPIO_STATUS1, 0x0050)
REG32(GPIO_STATUS1_W1TS, 0x0054)
REG32(GPIO_STATUS1_W1TC, 0x0058)
REG32(GPIO_ACPU_INT, 0x0060)
REG32(GPIO_PCPU_INT, 0x0068)
REG32(GPIO_ACPU_INT1, 0x0074)
REG32(GPIO_PCPU_INT1, 0x007c)
REG32(GPIO_PIN_BASE,0x88)
    FIELD(GPIO_PIN,INT_TYPE,7,3)
    FIELD(GPIO_PIN,INT_ENABLE,13,5)
REG32(GPIO_FUNC_IN_SEL_CFG_BASE,0x130)
    FIELD(GPIO_FUNC_IN,SEL,0,6)
    FIELD(GPIO_FUNC_IN,SIG_SEL,7,1)
REG32(GPIO_FUNC_OUT_SEL_CFG_BASE,0x530)
    FIELD(GPIO_FUNC_OUT,SEL,0,9)
    FIELD(GPIO_FUNC_OUT,OEN_SEL,10,1)

// IOMUX Registers
REG32(IO_MUX_BASE,0x4)
    FIELD(IO_MUX,FUN_WPD,7,1)
    FIELD(IO_MUX,FUN_WPU,8,1)
    FIELD(IO_MUX,FUN_IE,9,1)
    FIELD(IO_MUX,FUN_DRV,10,2)
    FIELD(IO_MUX,MCU_SEL,12,3)

// RTCIO Registers 
REG32(RTC_GPIO_OUT,0x00)
REG32(RTC_GPIO_OUT_W1TS,0x04)
REG32(RTC_GPIO_OUT_W1TC,0x08)
REG32(RTC_GPIO_ENABLE,0x0c)
REG32(RTC_GPIO_ENABLE_W1TS,0x10)
REG32(RTC_GPIO_ENABLE_W1TC,0x14)
REG32(RTC_GPIO_STATUS,0x18)
REG32(RTC_GPIO_STATUS_W1TS,0x1c)
REG32(RTC_GPIO_STATUS_W1TC,0x20)
REG32(RTC_GPIO_IN,0x24)
REG32(RTC_GPIO_PIN,0x28)
    FIELD(RTC_GPIO_PIN,INT_TYPE,7,3)
    FIELD(RTC_GPIO_PIN,WAKEUP_ENABLE,10,1)
REG32(RTC_PAD_CFG,0x7c)
REG32(RTC_DIG_PAD_HOLD,0x74)
REG32(RTC_EXT_WAKEUP0,0xbc)


#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion iomuxmem;
    MemoryRegion iortcmem;
    qemu_irq irq;
    uint32_t gpio_out;
    uint32_t gpio_out1;
    uint32_t strap_mode;
    uint32_t gpio_in;
    uint32_t gpio_in1;
    uint32_t gpio_status;
    uint32_t gpio_status1;
    uint32_t gpio_pcpu_int;
    uint32_t gpio_pcpu_int1;
    uint32_t gpio_acpu_int;
    uint32_t gpio_acpu_int1;
    uint32_t gpio_enable;
    uint32_t gpio_enable1;
    uint32_t gpio_pin[40];
    uint32_t gpio_in_sel[256];
    uint32_t gpio_out_sel[40];
    qemu_irq gpios[40];
    uint32_t iomux_regs[41];
   // uint32_t rtcio_regs[64];
    uint32_t rtc_gpio_out;
    uint32_t rtc_gpio_in;
    uint32_t rtc_gpio_status;
    uint32_t rtc_gpio_enable;
    uint32_t rtc_gpio_pin[18];
    uint32_t rtc_pad_cfg[16];
    uint32_t rtc_dig_pad_hold;
    uint32_t rtc_ext_wakeup0;
    QemuConsole *con;
    uint32_t *data;
    uint32_t redraw;
    qemu_irq rtc_reset;
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;

#define ESP32_GPIOS "esp32_gpios"
#define ESP32_GPIOS_IN "esp32_gpios_in"
#define ESP32_GPIOS_FUNC "esp32_gpios_func"

