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
#include "driver/spi_master.h"

#define TLC59711_NUM_CHANNELS 12 // Number of channels per driver


typedef struct {
  uint16_t *pwmbuffer;
  uint8_t BCr;
  uint8_t BCg;
  uint8_t  BCb;
  spi_device_handle_t _spi_dev;
} TLC59711;


/*!
 *  @brief  Class that stores state and functions for interacting with
 *          TLC59711 Senor
 */
bool TLC59711_begin(TLC59711 *ledDriverParameter, spi_device_handle_t spi);

void TLC59711_setPWM(TLC59711 *ledDriverParameter, uint16_t chan, uint16_t pwm);
void TLC59711_setLED(TLC59711 *ledDriverParameter, uint8_t lednum, uint16_t r, uint16_t g, uint16_t b);
//void TLC59711_getLED(TLC59711 *ledDriverParameter, uint8_t lednum, uint16_t r, uint16_t g, uint16_t b);
void TLC59711_write(TLC59711 *ledDriverParameter);
void TLC59711_setBrightness(TLC59711 *ledDriverParameter, uint8_t bcr, uint8_t bcg, uint8_t bcb);
void TLC59711_simpleSetBrightness(TLC59711 *ledDriverParameter, uint8_t BC);



#endif
