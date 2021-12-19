#include "tasks.h"

#include <stddef.h>

#include "drv_serial.h"
#include "drv_usb.h"

#include "buzzer.h"
#include "control.h"
#include "gestures.h"
#include "imu.h"
#include "led.h"
#include "osd_render.h"
#include "profile.h"
#include "project.h"
#include "rx.h"
#include "sixaxis.h"
#include "usb_configurator.h"
#include "vbat.h"
#include "vtx.h"

void task_main() {
  // read gyro and accelerometer data
  sixaxis_read();

  // all flight calculations and motors
  control();

  // attitude calculations for level mode
  imu_calc();

  // battery low logic
  vbat_calc();

  // check gestures
  if (flags.on_ground && !flags.gestures_disabled) {
    gestures();
  }

  // handle led commands
  led_update();

#if (RGB_LED_NUMBER > 0)
  // RGB led control
  rgb_led_lvc();
#ifdef RGB_LED_DMA
  rgb_dma_start();
#endif
#endif

#ifdef BUZZER_ENABLE
  buzzer();
#endif
}

bool task_usb_poll() {
  // TODO: this function should not modify state

  if (usb_detect()) {
    flags.usb_active = 1;
#ifndef ALLOW_USB_ARMING
    if (rx_aux_on(AUX_ARMING))
      flags.arm_safety = 1; //final safety check to disallow arming during USB operation
#endif
    return usb_serial_available();
  }

  flags.usb_active = 0;
  extern usb_motor_test_t usb_motor_test;
  usb_motor_test.active = 0;

  return false;
}

__weak bool rx_poll() {
  return true;
}

task_t tasks[TASK_MAX] = {
    [TASK_MAIN] = CREATE_TASK("MAIN", NULL, TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, 0, NULL, task_main),
    [TASK_RX] = CREATE_TASK("RX", NULL, TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, 4000, rx_poll, rx_check),

    [TASK_USB] = CREATE_TASK("USB", NULL, TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, 0, task_usb_poll, usb_configurator),
#ifdef ENABLE_BLACKBOX
    [TASK_BLACKBOX] = CREATE_TASK("BLACKBOX", NULL, TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, NULL, blackbox_update),
#endif
#ifdef ENABLE_OSD
    [TASK_OSD] = CREATE_TASK("OSD", NULL, TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, 0, NULL, osd_display),
#endif
    [TASK_VTX] = CREATE_TASK("VTX", NULL, TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, 0, NULL, vtx_update),
};