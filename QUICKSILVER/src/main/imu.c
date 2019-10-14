#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "defines.h"
#include "drv_time.h"
#include "sixaxis.h"
#include "util.h"

#define ACC_1G 1.0f

// disable drift correction ( for testing)
#define DISABLE_ACC 0

// filter times in seconds
// time to correct gyro readings using the accelerometer
// 1-4 are generally good
#define FASTFILTER 0.05 //onground filter
#define PREFILTER 0.2   //in_air prefilter (this can be commented out)
#define FILTERTIME 1.0  //in_air fusion filter

// accel magnitude limits for drift correction
#define ACC_MIN 0.7f
#define ACC_MAX 1.3f

float GEstG[3] = {0, 0, ACC_1G};
float attitude[3];

extern float gyro[3];
extern float accel[3];
extern float accelcal[3];

void imu_init(void) {
  // init the gravity vector with accel values
  for (int xx = 0; xx < 100; xx++) {
    sixaxis_read();

    for (int x = 0; x < 3; x++) {
      lpf(&GEstG[x], accel[x] * (1 / 2048.0f), 0.85);
    }
    delay(1000);
  }
}

float calcmagnitude(float vector[3]) {
  float accmag = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    accmag += vector[axis] * vector[axis];
  }
  accmag = 1.0f / Q_rsqrt(accmag);
  return accmag;
}

void vectorcopy(float *vector1, float *vector2) {
  for (int axis = 0; axis < 3; axis++) {
    vector1[axis] = vector2[axis];
  }
}

extern float looptime;

void imu_calc(void) {
  // remove bias
  accel[0] = accel[0] - accelcal[0];
  accel[1] = accel[1] - accelcal[1];
  accel[2] = accel[2] - accelcal[2];

  // reduce to accel in G
  for (int i = 0; i < 3; i++) {
    accel[i] *= (1 / 2048.0f);
  }

  float deltaGyroAngle[3];

  for (int i = 0; i < 3; i++) {
    deltaGyroAngle[i] = (gyro[i]) * looptime;
  }

  GEstG[2] = GEstG[2] - (deltaGyroAngle[0]) * GEstG[0];
  GEstG[0] = (deltaGyroAngle[0]) * GEstG[2] + GEstG[0];

  GEstG[1] = GEstG[1] + (deltaGyroAngle[1]) * GEstG[2];
  GEstG[2] = -(deltaGyroAngle[1]) * GEstG[1] + GEstG[2];

  GEstG[0] = GEstG[0] - (deltaGyroAngle[2]) * GEstG[1];
  GEstG[1] = (deltaGyroAngle[2]) * GEstG[0] + GEstG[1];

  extern int onground;
  if (onground) { //happyhour bartender - quad is ON GROUND and disarmed
    // calc acc mag
    float accmag = calcmagnitude(&accel[0]);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        accel[axis] = accel[axis] * (ACC_1G / accmag);
      }

      float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FASTFILTER);
      for (int x = 0; x < 3; x++) {
        lpf(&GEstG[x], accel[x], filtcoeff);
      }
    }
  } else {

    //lateshift bartender - quad is IN AIR and things are getting wild
    // hit accel[3] with a sledgehammer
#ifdef PREFILTER
    float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)PREFILTER);
    for (int x = 0; x < 3; x++) {
      lpf(&accel[x], accel[x], filtcoeff);
    }
#endif

    // calc mag of filtered acc
    float accmag = calcmagnitude(&accel[0]);
    if ((accmag > ACC_MIN * ACC_1G) && (accmag < ACC_MAX * ACC_1G)) {
      // normalize acc
      for (int axis = 0; axis < 3; axis++) {
        accel[axis] = accel[axis] * (ACC_1G / accmag);
      }
      // filter accel on to GEstG
      float filtcoeff = lpfcalc_hz(looptime, 1.0f / (float)FILTERTIME);
      for (int x = 0; x < 3; x++) {
        lpf(&GEstG[x], accel[x], filtcoeff);
      }
      //heal the gravity vector after nudging it with accel
      float GEstGmag = calcmagnitude(&GEstG[0]);
      for (int axis = 0; axis < 3; axis++) {
        GEstG[axis] = GEstG[axis] * (ACC_1G / GEstGmag);
      }
    }
  }

  if (rx_aux_on(AUX_HORIZON)) {
    attitude[0] = atan2approx(GEstG[0], GEstG[2]);
    attitude[1] = atan2approx(GEstG[1], GEstG[2]);
  }
}
