/*!
 *  @file Adafruit_TLC59711.h
 *
 *  This is a library for the Adafruit 12-channel PWM/LED driver
 *
 *  Designed specifically to work with the Adafruit 12-channel PWM/LED driver
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/1455
 *
 *  Two SPI Pins are required to send data: clock and data pin.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef _ADAFRUIT_TLC59711_H
#define _ADAFRUIT_TLC59711_H
#include <spi_master.h>

#define TLC59711_NUM_CHANNELS 12 // Number of channels per driver

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          TLC59711 Senor
 */
class TLC59711 {
public:
  TLC59711(spi_device_handle_t spi);
  ~TLC59711() {
    if (pwmbuffer) {
      free(pwmbuffer);
      pwmbuffer = NULL;
    }
  }

  bool begin();

  void setPWM(uint16_t chan, uint16_t pwm);
  void setLED(uint8_t lednum, uint16_t r, uint16_t g, uint16_t b);
  void getLED(uint8_t lednum, uint16_t &r, uint16_t &g, uint16_t &b);
  void write();
  void setBrightness(uint8_t bcr, uint8_t bcg, uint8_t bcb);
  void simpleSetBrightness(uint8_t BC);

private:
  uint16_t *pwmbuffer = NULL;

  uint8_t BCr = 0, BCg = 0, BCb = 0;
  spi_device_handle_t _spi_dev;
};

#endif
