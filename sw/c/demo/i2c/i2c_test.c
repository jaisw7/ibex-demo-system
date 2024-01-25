
#include "log.h"
#include "print.h"
#include "check.h"

#include "dif_i2c.h"
#include "i2c_testutils.h"

#include "demo_system.h"
#include "timer.h"
#include "gpio.h"
#include "pwm.h"

#define RPI_ID 0
#define RPI_HAT 1

/*
enum {
  // 
  kDeviceAddr = ,

  // Registers
  kDeviceIdReg = 0x00,
  kThreshTapReg = 0x1D,
  kPowerCtrlReg = 0x2D,
  kIntSourceReg = 0x30,
  kDataX0Reg = 0x32,
  kDataX1Reg = 0x33,
  kDataY0Reg = 0x34,
  kDataY1Reg = 0x35,
  kDataZ0Reg = 0x36,
  kDataZ1Reg = 0x37,

  // Registers values
  kDeviceId = 0xE5,
  kMeasure = 0x08,

  // kIntSourceReg masks
  kIntSourceDataReadyBit = 0x01 << 7,

  // Timing specification.
  kDefaultTimeoutMicros = 5000,
  kTurnOnTimeMicros = 11000,
};
*/

// IS31FL3741 13x9 RGB LED Matrix
enum {
  kDeviceAddr = 0x30,

  kDeviceIdReg = 0xFC,

  // Registers values
  kDeviceId = 0x60,
//  kMeasure = 0x08,

  // Timing specification.
  kDefaultTimeoutMicros = 5000,
  kTurnOnTimeMicros = 11000,
};

//static dif_rv_core_ibex_t rv_core_ibex;
//static dif_pinmux_t pinmux;
static dif_i2c_t i2c;

#if 0
static status_t read_device_id(void) {
  uint8_t reg = kDeviceIdReg, data = 0;
  LOG_INFO("read_device_id");
  TRY(i2c_testutils_write(&i2c, kDeviceAddr, 1, &reg, true));
  TRY(i2c_testutils_read(&i2c, kDeviceAddr, 1, &data, kDefaultTimeoutMicros));
  TRY_CHECK(data == kDeviceId, "Unexpected value %x", data);
  return OK_STATUS();
}
#endif
 
#if RPI_ID
static status_t read_eeprom(void) {
  static uint8_t data[0x100];
  uint8_t addr[] = { 0, 0 };
// TODO: the EPROM is 0x1000 bytes, but we can't even read 256 here
uint8_t len = 0xf0u;
  // Read ID EEPROM from RPi HAT
  LOG_INFO("Writing address 0x%x\n", (addr[0] << 8) | addr[1]);
#if 1
  // Send two byte address (0x0000) and skip the STOP condition
  TRY(i2c_testutils_write(&i2c, 0x50, 2, addr, true));
  LOG_INFO("Reading 0x%x bytes of data\n", len);
  // Burst read data back
  TRY(i2c_testutils_read(&i2c, 0x50, len, data, kDefaultTimeoutMicros));
#else
  for (unsigned i = 0; i < 0xf0; ++i) {
    addr[1] = i;  // TODO: guess!
    TRY(i2c_testutils_write(&i2c, 0x50, 2, addr, false)); //));
    TRY(i2c_testutils_read(&i2c, 0x50, 1, &data[i], kDefaultTimeoutMicros));
  }
#endif
  LOG_INFO("Got data...\n");
#if 0
  for (unsigned i = 0; i < len; ++i) {
    LOG_INFO("%x: %x\n", i, data[i]);
  }
#else
  base_hexdump((char*)data, len);
#endif
  return OK_STATUS();
}
#endif

#if RPI_HAT
// Read  "WHO_AM_I" register in the IMU on the RPi Sense HAT
static status_t read_sense_imu(void) {
  uint8_t data[2];
  uint8_t addr[] = { 0x0f };
  LOG_INFO("Attempting to read from Sense IMU\n");
  while (1) {
    TRY(i2c_testutils_write(&i2c, 0x6a, 1, addr, true)); //));
//    LOG_INFO(" trying read\n");
    TRY(i2c_testutils_read(&i2c, 0x6a, 1, &data[0], kDefaultTimeoutMicros));
    LOG_INFO("Read 0x%x", data[0]);
    TRY(i2c_testutils_write(&i2c, 0x1c, 1, addr, true)); //));
    TRY(i2c_testutils_read(&i2c, 0x1c, 1, &data[1], kDefaultTimeoutMicros));
    LOG_INFO(" - Read 0x%x\n", data[1]);
    if (data[0] != 0x68 || data[1] != 0x3d) {
      LOG_INFO("*** READ FAILED ***\n");
      while (1) {
      }
    }
  }
}
#endif

extern void setup(void);

void i2c_read(uint8_t dev_addr, uint8_t *buf, uint8_t n) {
//  LOG_INFO("read %p %u\n", buf, n);
  CHECK_STATUS_OK(i2c_testutils_read(&i2c, dev_addr, n, buf, kDefaultTimeoutMicros));
}

void i2c_write(uint8_t dev_addr, const uint8_t *data, uint8_t n, bool skip_stop) {
  static int written = 0;
  CHECK(written + n <= 64);
  written += n;
  status_t res = i2c_testutils_write(&i2c, dev_addr, n, data, skip_stop);
  //if (!status_ok(res))
//  LOG_INFO("res %u (%p %u\n)", res, data, n);
  CHECK_STATUS_OK(res);
  if (written + n >= 48 && !skip_stop) {
    CHECK_STATUS_OK(i2c_testutils_wait_host_idle(&i2c));
    written = 0;
  }
//  LOG_INFO("i2c_write done\n");
}

bool main(void) {

  LOG_INFO("i2c_test started\n");

  mmio_region_t base_addr;
//      mmio_region_from_addr(TOP_EARLGREY_RV_CORE_IBEX_CFG_BASE_ADDR);

//  TRY(dif_rv_core_ibex_init(base_addr, &rv_core_ibex));

  // TODO: we've just mapped the I2C in place of USBDEV presently
  base_addr = mmio_region_from_addr(USBDEV_BASE);  //TOP_EARLGREY_I2C2_BASE_ADDR);
  CHECK_DIF_OK(dif_i2c_init(base_addr, &i2c));

//  base_addr = mmio_region_from_addr(TOP_EARLGREY_PINMUX_AON_BASE_ADDR);
//  TRY(dif_pinmux_init(base_addr, &pinmux));

//  TRY(i2c_testutils_select_pinmux(&pinmux, 2));

  CHECK_DIF_OK(dif_i2c_host_set_enabled(&i2c, kDifToggleEnabled));

  dif_i2c_speed_t speeds[] = {kDifI2cSpeedStandard, kDifI2cSpeedFast,
                              kDifI2cSpeedFastPlus};

//TODO:
//  for (size_t i = 0; i < ARRAYSIZE(speeds); ++i) {
  size_t i = 2;
    CHECK_STATUS_OK(i2c_testutils_set_speed(&i2c, speeds[i]));
//    while (1) {
//    CHECK_STATUS_OK(read_DEVICE_IDread());
//    }

#if RPI_ID
    read_eeprom();
#elif RPI_HAT
    read_sense_imu();
#else
    setup();
#endif
//  }

  while (1);

  return true;
}
