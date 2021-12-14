#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "buzzer.h"
#include "control.h"
#include "debug.h"
#include "drv_adc.h"
#include "drv_fmc.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_rgb_led.h"
#include "drv_serial.h"
#include "drv_spi.h"
#include "drv_spi_soft.h"
#include "drv_time.h"
#include "filter.h"
#include "flash.h"
#include "gestures.h"
#include "imu.h"
#include "led.h"
#include "osd_render.h"
#include "pid.h"
#include "profile.h"
#include "project.h"
#include "rgb_led.h"
#include "rx.h"
#include "scheduler.h"
#include "sixaxis.h"
#include "util.h"
#include "vbat.h"
#include "vtx.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#endif

#ifdef STM32F4
#include "blackbox.h"
#include "drv_usb.h"
#include "usb_configurator.h"
#endif

extern profile_t profile;

void failloop(int val);

int random_seed = 0;

void memory_section_init() {
#ifdef USE_FAST_RAM
  extern uint8_t _fast_ram_start;
  extern uint8_t _fast_ram_end;
  extern uint8_t _fast_ram_data;
  memcpy(&_fast_ram_start, &_fast_ram_data, (size_t)(&_fast_ram_end - &_fast_ram_start));
#endif
}

int main() {
  // init timer so we can use delays etc
  time_init();

  // load default profile
  profile_set_defaults();

  // setup filters early
  filter_global_init();
  pid_init();

  // read pid identifier for values in file pid.c
  flash_hard_coded_pid_identifier();

  // load flash saved variables
  flash_load();
  time_delay_us(1000);

  //init some hardware things
  gpio_init();
  usb_init();
  ledon(255); //Turn on LED during boot so that if a delay is used as part of using programming pins for other functions, the FC does not appear inactive while programming times out
  spi_init();
  usart_invert();
#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_UNIFIED_SERIAL)
  rx_spektrum_bind();
#endif

  time_delay_us(100000);

  //init the firmware things
  motor_init();
  motor_set_all(0);

  if (!sixaxis_init()) {
    //gyro not found
    failloop(4);
  }

#ifdef ENABLE_OSD
  time_delay_us(300000);
  osd_init();
#endif

  adc_init();

  //set always on channel to on
  state.aux[AUX_CHANNEL_ON] = 1;
  state.aux[AUX_CHANNEL_OFF] = 0;
#ifdef GESTURE_AUX_START_ON
  state.aux[AUX_CHANNEL_GESTURE] = 1;
#endif

  vtx_init();

#ifdef SERIAL_RX
  // if our RX is a serial, only init if we have valid usart
  if (profile.serial.rx != USART_PORT_INVALID) {
    rx_init();
  }
#else
  // we have a spi RX
  rx_init();
#endif

  time_delay_us(1000);
  vbat_init();

#ifdef RX_BAYANG_BLE_APP
  // for randomising MAC adddress of ble app - this will make the int = raw float value
  random_seed = *(int *)&state.vbattfilt;
  random_seed = random_seed & 0xff;
#endif

  sixaxis_gyro_cal();
  rgb_init();

#ifdef ENABLE_BLACKBOX
  blackbox_init();
#endif

  imu_init();

#ifdef ENABLE_OSD
  osd_clear();
#endif

  extern int liberror;
  if (liberror) {
    failloop(7);
  }

  scheduler_init();

  while (1) {
    scheduler_update();
  }
}

// the error codes indicate a failure that prevents normal operation
// led flash codes - the quad will not fly / bind if flashing a code
// 2 - low battery at powerup - currently unused
// 3 - radio chip not found
// 4 - Gyro not found
// 5 - clock , intterrupts , systick , gcc bad code , bad memory access (code issues like bad pointers)- this should not come up
// 6 - loop time issue - if loop time exceeds 20mS
// 7 - spi error  - triggered by hardware spi driver only
// 8 - i2c error main loop  - triggered by depreciated hardware i2c driver only

const char *failloop_string(int val) {
  switch (val) {
  case 2:
    return "low battery at powerup - unused";
  case 3:
    return "radio chip not detected";
  case 4:
    return "Gyro not found";
  case 5:
    return "clock , intterrupts , systick";
  case 6:
    return "loop time issue";
  case 7:
    return "i2c error";
  case 8:
    return "i2c error main loop";
  default:
    return "unknown error";
  }
}

void failloop(int val) {
  for (int i = 0; i <= 3; i++) {
    motor_set(i, 0);
  }

  while (1) {
#if defined(STM32F4) && defined(DEBUG)
    quic_debugf("failloop %s (%d)", failloop_string(val), val);
    usb_detect();
#endif
    for (int i = 0; i < val; i++) {
      ledon(255);
      time_delay_us(500000);
      ledoff(255);
      time_delay_us(500000);
    }
    time_delay_us(800000);
  }
}

void handle_fault() {
#if defined(STM32F4) && defined(RESET_ON_FAULT)
  extern void systemResetToBootloader();
  systemResetToBootloader();
#endif

  failloop(5);
}

void HardFault_Handler() {
  handle_fault();
}
void MemManage_Handler() {
  handle_fault();
}
void BusFault_Handler() {
  handle_fault();
}
void UsageFault_Handler() {
  handle_fault();
}
