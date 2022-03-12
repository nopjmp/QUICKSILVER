#include "flight/sixaxis.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "drv_serial.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "flash.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/sixaxis.h"
#include "led.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

#define CAL_TIME 2e6
#define GLOW_TIME 62500

// gyro has +-2000 divided over 16bit.
#define GYRO_RANGE (1.f / (65536.f / 4000.f))

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

// temporary fix for compatibility between versions
#ifndef GYRO_ID_1
#define GYRO_ID_1 0x68
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x98
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif

#define SMITH_MAX_SAMPLES 6 * 8

typedef struct {
  uint8_t idx;

  float samples[SMITH_MAX_SAMPLES + 1];
  uint8_t sample_count;
} smith_predictor_t;

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

extern profile_t profile;
extern target_info_t target_info;

float gyrocal[3];

static float smith_predictor_strength = 0.5f;
static uint8_t smith_predictor_delay = 40;
static uint16_t smith_predictor_filter_hz = 5;

static smith_predictor_t smith_predictor[3];
static filter_lp_pt1 smith_predictor_filter;
static filter_state_t smith_predictor_filter_state[3];

static void smith_predictor_init() {
  memset(smith_predictor, 0, 3 * sizeof(smith_predictor_t));

  for (uint8_t i = 0; i < 3; i++) {
    smith_predictor[i].sample_count = min_uint32(smith_predictor_delay / (LOOPTIME / 100.0f), SMITH_MAX_SAMPLES);
  }

  filter_lp_pt1_init(&smith_predictor_filter, smith_predictor_filter_state, 3, smith_predictor_filter_hz);
}

static float smith_predictor_step(uint8_t axis, float sample) {
  if (smith_predictor[axis].sample_count <= 1) {
    return sample;
  }

  smith_predictor[axis].idx++;
  if (smith_predictor[axis].idx > smith_predictor[axis].sample_count) {
    smith_predictor[axis].idx = 0;
  }

  const float delayed_sample = smith_predictor[axis].samples[smith_predictor[axis].idx];
  const float delay_scaled_sample = smith_predictor_strength * (sample - delayed_sample);
  const float delay_compenstated_sample = filter_lp_pt1_step(&smith_predictor_filter, &smith_predictor_filter_state[axis], delay_scaled_sample);

  return sample + delay_compenstated_sample;
}

uint8_t sixaxis_init() {
  const uint8_t id = spi_gyro_init();

  target_info.gyro_id = id;

  smith_predictor_init();
  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.gyro[i].type, &filter[i], filter_state[i], 3, profile.filter.gyro[i].cutoff_freq);
  }

#ifndef DISABLE_GYRO_CHECK
  return (GYRO_ID_1 == id || GYRO_ID_2 == id || GYRO_ID_3 == id || GYRO_ID_4 == id);
#else
  return 1;
#endif
}

void sixaxis_read() {
  const gyro_data_t data = spi_gyro_read();

  state.accel_raw = data.accel;
  state.gyro_temp = data.temp;
  state.gyro_raw = data.gyro;

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    float temp = state.accel_raw.axis[1];
    state.accel_raw.axis[1] = state.accel_raw.axis[0];
    state.accel_raw.axis[0] = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    float temp = state.accel_raw.axis[0];
    state.accel_raw.axis[0] = (state.accel_raw.axis[0] * INVSQRT2 + state.accel_raw.axis[1] * INVSQRT2);
    state.accel_raw.axis[1] = -(temp * INVSQRT2 - state.accel_raw.axis[1] * INVSQRT2);
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    float temp = state.accel_raw.axis[1];
    state.accel_raw.axis[1] = (state.accel_raw.axis[1] * INVSQRT2 + state.accel_raw.axis[0] * INVSQRT2);
    state.accel_raw.axis[0] = -(temp * INVSQRT2 - state.accel_raw.axis[0] * INVSQRT2);
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    float temp = state.accel_raw.axis[1];
    state.accel_raw.axis[1] = -state.accel_raw.axis[0];
    state.accel_raw.axis[0] = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    state.accel_raw.axis[1] = -state.accel_raw.axis[1];
    state.accel_raw.axis[0] = -state.accel_raw.axis[0];
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    state.accel_raw.axis[2] = -state.accel_raw.axis[2];
    state.accel_raw.axis[0] = -state.accel_raw.axis[0];
  }

  // remove bias and reduce to state.accel_raw in G
  state.accel_raw.axis[0] = (state.accel_raw.axis[0] - flash_storage.accelcal[0]) * (1 / 2048.0f);
  state.accel_raw.axis[1] = (state.accel_raw.axis[1] - flash_storage.accelcal[1]) * (1 / 2048.0f);
  state.accel_raw.axis[2] = (state.accel_raw.axis[2] - flash_storage.accelcal[2]) * (1 / 2048.0f);

  state.gyro_raw.axis[0] = state.gyro_raw.axis[0] - gyrocal[0];
  state.gyro_raw.axis[1] = state.gyro_raw.axis[1] - gyrocal[1];
  state.gyro_raw.axis[2] = state.gyro_raw.axis[2] - gyrocal[2];

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    float temp = state.gyro_raw.axis[1];
    state.gyro_raw.axis[1] = -state.gyro_raw.axis[0];
    state.gyro_raw.axis[0] = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    float temp = state.gyro_raw.axis[1];
    state.gyro_raw.axis[1] = state.gyro_raw.axis[0] * INVSQRT2 + state.gyro_raw.axis[1] * INVSQRT2;
    state.gyro_raw.axis[0] = state.gyro_raw.axis[0] * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    float temp = state.gyro_raw.axis[0];
    state.gyro_raw.axis[0] = state.gyro_raw.axis[1] * INVSQRT2 + state.gyro_raw.axis[0] * INVSQRT2;
    state.gyro_raw.axis[1] = state.gyro_raw.axis[1] * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    float temp = state.gyro_raw.axis[1];
    state.gyro_raw.axis[1] = state.gyro_raw.axis[0];
    state.gyro_raw.axis[0] = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    state.gyro_raw.axis[1] = -state.gyro_raw.axis[1];
    state.gyro_raw.axis[0] = -state.gyro_raw.axis[0];
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    state.gyro_raw.axis[1] = -state.gyro_raw.axis[1];
    state.gyro_raw.axis[2] = -state.gyro_raw.axis[2];
  }

  state.gyro_raw.axis[0] = state.gyro_raw.axis[0] * GYRO_RANGE * DEGTORAD;
  state.gyro_raw.axis[1] = -state.gyro_raw.axis[1] * GYRO_RANGE * DEGTORAD;
  state.gyro_raw.axis[2] = -state.gyro_raw.axis[2] * GYRO_RANGE * DEGTORAD;

  filter_coeff(profile.filter.gyro[0].type, &filter[0], profile.filter.gyro[0].cutoff_freq);
  filter_coeff(profile.filter.gyro[1].type, &filter[1], profile.filter.gyro[1].cutoff_freq);
  filter_lp_pt1_coeff(&smith_predictor_filter, smith_predictor_filter_hz);

  for (int i = 0; i < 3; i++) {
    smith_predictor[i].sample_count = min_uint32(smith_predictor_delay / (state.looptime_us / 100.0f), SMITH_MAX_SAMPLES);

    state.gyro.axis[i] = state.gyro_raw.axis[i];

    state.gyro.axis[i] = filter_step(profile.filter.gyro[0].type, &filter[0], &filter_state[0][i], state.gyro.axis[i]);
    state.gyro.axis[i] = filter_step(profile.filter.gyro[1].type, &filter[1], &filter_state[1][i], state.gyro.axis[i]);
    state.gyro.axis[i] = smith_predictor_step(i, state.gyro.axis[i]);
  }
}

void sixaxis_gyro_cal() {
  float limit[3];
  uint32_t time = time_micros();
  uint32_t timestart = time;
  uint32_t timemax = time;
  uint32_t lastlooptime = time;

  for (int i = 0; i < 3; i++) {
    limit[i] = gyrocal[i];
  }

  // 2 and 15 seconds
  while (time - timestart < CAL_TIME && time - timemax < 15e6) {

    uint32_t looptime;
    looptime = time - lastlooptime;
    lastlooptime = time;
    if (looptime == 0)
      looptime = 1;

    const gyro_data_t data = spi_gyro_read();

    static int brightness = 0;
    led_pwm(brightness);
    if ((brightness & 1) ^ ((time - timestart) % GLOW_TIME > (GLOW_TIME >> 1))) {
      brightness++;
    }

    brightness &= 0xF;

    for (int i = 0; i < 3; i++) {

      if (data.gyro.axis[i] > limit[i])
        limit[i] += 0.1f; // 100 gyro bias / second change
      if (data.gyro.axis[i] < limit[i])
        limit[i] -= 0.1f;

      limitf(&limit[i], 800);

      if (fabsf(data.gyro.axis[i]) > 100 + fabsf(limit[i])) {
        timestart = time_micros();
        brightness = 1;
      } else {
        lpf(&gyrocal[i], data.gyro.axis[i], lpfcalc((float)looptime, 0.5 * 1e6));
      }
    }

    while ((time_micros() - time) < 1000)
      time_delay_us(10);
    time = time_micros();
  }

  if (time - timestart < CAL_TIME) {
    for (int i = 0; i < 3; i++) {
      gyrocal[i] = 0;
    }
  }
}

void sixaxis_acc_cal() {
  flash_storage.accelcal[2] = 2048;
  for (int y = 0; y < 500; y++) {
    sixaxis_read();
    for (int x = 0; x < 3; x++) {
      lpf(&flash_storage.accelcal[x], state.accel_raw.axis[x], 0.92);
    }
    time_micros(); // if it takes too long time will overflow so we call it here
  }
  flash_storage.accelcal[2] -= 2048;

  for (int x = 0; x < 3; x++) {
    limitf(&flash_storage.accelcal[x], 500);
  }
}
