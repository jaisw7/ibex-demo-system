// Scrolling text example for the Adafruit IS31FL3741 13x9 PWM RGB LED
// Matrix Driver w/STEMMA QT / Qwiic connector. This is the simplest
// version and should fit on small microcontrollers like Arduino Uno.
// Tradeoff is that animation isn't always as smooth as seen in the
// buffered example. Each LED changes state immediately when accessed,
// there is no show() or display() function as with NeoPixels or some
// OLED screens.

#include "Adafruit_IS31FL3741/Adafruit_IS31FL3741.h"
#include "SparkFun_BMA400_Arduino_Library/src/SparkFun_BMA400_Arduino_Library.h"

extern "C" {
#include "log.h"
#include "timer.h"
};

#include "demo_system.h"
#include "dev_access.h"

// If colors appear wrong on matrix, try invoking constructor like so:
// Adafruit_IS31FL3741_QT ledmatrix(IS3741_RBG);

// Some boards have just one I2C interface, but some have more...
TwoWire *i2c = &Wire; // e.g. change this to &Wire1 for QT Py RP2040
TwoWire Wire;


void delayMicroseconds(uint32_t micros) {
// LOG_INFO("delay micros %u\n", micros);
  uint32_t ms = (micros + 999) / 1000;
  // Configure timer to trigger every 1 ms
  timer_enable(50000);
  uint32_t timeout = get_elapsed_time() + ms;
  while (get_elapsed_time() < timeout) {
    asm volatile("wfi");
  }
  timer_disable();
}

extern const uint8_t digs[] = {
  0x3f,
  0x6,
  0x5b,
  0x4f,
  0x66,
  0x6d,
  0x7d,
  0x7,
  0x7f,
  0x6f
};

static void hex_led(uint8_t dd) {
  static uint8_t nyb = 0;
  static uint8_t val;
//  LOG_INFO("hex %u\n", dd);
  val = ((dd / 10) << 4) | (dd % 10);
  DEV_WRITE(GPIO_OUT, ((!nyb << 7) | digs[(val >> (nyb * 4)) & 0xf]));
//  LOG_INFO("hex %x\n", ((!nyb << 7) | digs[(val >> (nyb * 4)) & 0xf]));
  nyb ^= 1;
}

static void hex_blank(void) {
  DEV_WRITE(GPIO_OUT, 0);
}

extern "C" {
void setup() {
  Adafruit_IS31FL3741_QT ledmatrix;
  // Create a new sensor object
  BMA400 accelerometer;
  bool matrix = true;

  timer_init();

  hex_blank();

  if (1) {
    if (! ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
      LOG_INFO("IS41 not found");
      matrix = false;
    }
    LOG_INFO("IS41 found!");
  } else {
    matrix = false;
  }

#if 0
    // I2C address selection
    uint8_t i2cAddress = BMA400_I2C_ADDRESS_DEFAULT; // 0x14
    //uint8_t i2cAddress = BMA400_I2C_ADDRESS_SECONDARY; // 0x15

    // Start serial
    LOG_INFO("BMA400 Example 1 - Basic Readings I2C");

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x14)
    while(accelerometer.beginI2C(i2cAddress) != BMA400_OK)
    {
        // Not connected, inform user
        LOG_INFO("Error: BMA400 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
//        delay(1000);
while (1) { }
    }

    LOG_INFO("BMA400 connected!");
#endif

#if 0
    // The sensor's resolution is based on the measurement range, which defaults
    // to 4g. We can reduce the range to the minimum of 2g to get the best
    // measurement resolution
//    accelerometer.setRange(BMA400_RANGE_2G);
  uint8_t buf = 0x0;

  while (1) {
    uint8_t acc[8];
    memcpy(acc, "ACBDEFGH", 8);

    float temp;
    accelerometer.getTemperature(&temp);

    accelerometer.setMode(BMA400_MODE_NORMAL);
    // Driver introduces a 10ms delay here, which is perhaps causing
    // problems if the device does employ auto low-power?
    uint8_t mode = 137;
    int8_t res = accelerometer.getMode(&mode);

    buf = 4;
    i2c_write(0x14, &buf, 1, true);
    i2c_read(0x14, acc, 8);
    LOG_INFO("%02x%02x : %02x%02x : %02x%02x : t %02x%02x : ", acc[1], acc[0], acc[3], acc[2], acc[5], acc[4], acc[7], acc[6]);
    LOG_INFO("mode was set to %x (res %d) temp %f\n", mode, res, (int)(temp * 100.0));

//while (1);
  }
//while (1);
#endif

  // By default the LED controller communicates over I2C at 400 KHz.
  // Arduino Uno can usually do 800 KHz, and 32-bit microcontrollers 1 MHz.
//  i2c->setClock(800000);

  if (matrix) {
    // Set brightness to max and bring controller out of shutdown state
#if 0
        for (int y=0; y<ledmatrix.height(); y++) {
          for (int x=0; x<ledmatrix.width(); x++) {
            ledmatrix.setLEDscaling(y*ledmatrix.width()+x, 0xff);
          }
        }
#else
      ledmatrix.setLEDscaling(0xFF);
#endif
    ledmatrix.setGlobalCurrent(0xC0);
    ledmatrix.enable(true); // bring out of shutdown
  }

  int16_t hue_offset = 0;
  int16_t hue_del = 2048;
  uint8_t sat = 0xC0;
  const int poll_init = 10; //500;
  int poll_cnt = poll_init;

  while (1) {
    static uint16_t temp = 0;

//  hex_blank();
    if (matrix) {
//    static const uint8_t val[] = {
//      4, 8, 12, 0x10, 0x20, 0x40, 0x60, 0xc0, 0xff,
//    };
      uint32_t di = 65536 / 117;
      uint32_t i = hue_offset;
      for (int y=0; y<ledmatrix.height(); y++) {
        for (int x=0; x<ledmatrix.width(); x++) {
          uint32_t color888 = ledmatrix.ColorHSV(i, sat, 0x20);
          uint16_t color565 = ledmatrix.color565(color888);
          ledmatrix.drawPixel(x, y, color565);
          i+=di;
        }
      }

      hue_offset += hue_del;
    }

//    ledmatrix.setGlobalCurrent(hue_offset / 256); // Demonstrate global current

    if (!--poll_cnt) {
#if 0
      // Get measurements from the sensor. This must be called before accessing
      // the acceleration data, otherwise it will never update
      accelerometer.getSensorData();
while (1);
      // Print acceleration data
      LOG_INFO("Acceleration in g's");
      LOG_INFO("\t");
      LOG_INFO("X: %x", accelerometer.data.accelX);
      LOG_INFO("\t");
      LOG_INFO("Y: %x", accelerometer.data.accelY);
      LOG_INFO("\t");
      LOG_INFO("Z: %x", accelerometer.data.accelZ);

      poll_cnt = 10;
#else
      uint8_t buf[2];
      buf[0] = 1;
      i2c_write(0x48, buf, 1, false);
      i2c_read(0x48, buf, 2);
      uint16_t config = (buf[0] << 8) | buf[1];
      buf[0] = 0;
      i2c_write(0x48, buf, 1, false);
      i2c_read(0x48, buf, 2);
      temp = (buf[0] << 8) | buf[1];

      LOG_INFO("Config 0x%04x Temp 0x%04x\n", config, temp);
      poll_cnt = poll_init;

#endif
    }
    hex_led((uint8_t)(temp >> 7));
  }
}

};

