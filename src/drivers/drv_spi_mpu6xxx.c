#include "drv_spi_mpu6xxx.h"

#include <stdio.h>

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

static volatile uint8_t buffer[32];
static volatile spi_bus_device_t bus = {
    .port = GYRO_SPI_PORT,
    .nss = GYRO_NSS,

    .buffer = buffer,
    .buffer_size = 32,

    .auto_continue = true,
};

static uint32_t mpu6xxx_slow_divider() {
  return spi_find_divder(MHZ_TO_HZ(0.5));
}

static uint32_t mpu6xxx_fast_divider() {
  switch (GYRO_TYPE) {
  case ICM20601:
  case ICM20608:
    return spi_find_divder(MHZ_TO_HZ(5.25));

  case ICM20602:
    return spi_find_divder(MHZ_TO_HZ(10.5));

  case MPU6XXX:
  default:
    return spi_find_divder(MHZ_TO_HZ(21));
  }
}

static void mpu6xxx_init() {

// Interrupt GPIO
#ifdef GYRO_INT
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, GYRO_INT);
#endif

  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, true, mpu6xxx_slow_divider());
}

uint8_t mpu6xxx_configure() {
  mpu6xxx_init();

  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); // reg 107 soft reset  MPU_BIT_H_RESET
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATHWAYS);
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX); // reg 107 set pll clock to 1 for x axis reference
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); // reg 106 to 16 enabling spi
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_PWR_MGMT_2, MPU_BITS_STDBY_MODE_OFF); // reg 108 disable standbye mode to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_SMPLRT_DIV, MPU_BIT_SMPLRT_DIVIDER_OFF); // reg 25 sample rate divider to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_256HZ); // reg 26 dlpf to 0 - 8khz
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_16G); // reg 28 accel scale to 16G
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS); // reg 27 gyro scale to 2000deg/s
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_INT_ENABLE, MPU_BIT_INT_STATUS_DATA); // reg 56 data ready enable interrupt to 1
  time_delay_us(1500);

  return mpu6xxx_read(MPU_RA_WHO_AM_I);
}

// blocking dma read of a single register
uint8_t mpu6xxx_read(uint8_t reg) {
  spi_bus_device_reconfigure(&bus, true, mpu6xxx_slow_divider());

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return buffer[1];
}

// blocking dma write of a single register
void mpu6xxx_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&bus, true, mpu6xxx_slow_divider());

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);
}

void mpu6xxx_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  spi_bus_device_reconfigure(&bus, true, mpu6xxx_fast_divider());

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg | 0x80);
  spi_txn_add_seg(txn, data, NULL, size);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);
}

static uint8_t dma_buf[14];

void mpu6xxx_read_start(spi_txn_done_fn_t done_fn) {
  // make sure last read is done.
  spi_txn_wait(&bus);

  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, mpu6xxx_fast_divider());

  spi_txn_t *txn = spi_txn_init(&bus, done_fn);
  spi_txn_add_seg_const(txn, MPU_RA_ACCEL_XOUT_H | 0x80);
  spi_txn_add_seg(txn, dma_buf, NULL, 14);
  spi_txn_submit(txn);

  spi_txn_continue(&bus);
}

void mpu6xxx_read_finish(gyro_data_t *data) {
  data->accel.axis[0] = -(int16_t)((dma_buf[0] << 8) + dma_buf[1]);
  data->accel.axis[1] = -(int16_t)((dma_buf[2] << 8) + dma_buf[3]);
  data->accel.axis[2] = (int16_t)((dma_buf[4] << 8) + dma_buf[5]);

  data->temp = (float)((int16_t)((dma_buf[6] << 8) + dma_buf[7])) / 333.87f + 21.f;

  data->gyro.axis[1] = (int16_t)((dma_buf[8] << 8) + dma_buf[9]);
  data->gyro.axis[0] = (int16_t)((dma_buf[10] << 8) + dma_buf[11]);
  data->gyro.axis[2] = (int16_t)((dma_buf[12] << 8) + dma_buf[13]);
}
