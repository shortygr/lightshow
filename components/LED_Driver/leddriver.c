/*!
 * @file Adafruit_TLC59711.cpp
 *
 * @mainpage Adafruit TLC59711 PWM/LED driver
 *
 * @section intro_sec Introduction
 *
 * This is a library for our Adafruit 12-channel PWM/LED driver
 *
 * Pick one up today in the adafruit shop!
 * ------> http://www.adafruit.com/products/1455
 *
 * Two SPI Pins are required to send data: clock and data pin.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#include "leddriver.h"
#include <string.h>

/*!
 *  @brief  Instantiates a new Adafruit_TLC59711 class
 *  @param  n
 *          number of connected drivers
 *  @param  c
 *          clock pin
 *  @param  d
 *          data pin
 */
bool TLC59711_begin(struct TLC59711 *ledDriverParameter, spi_device_handle_t spi) {

  ledDriverParameter->BCr = 0x7f 
  ledDriverParameter->BCg = 0x7f;
  ledDriverParameter->BCb = 0x7f; // default 100%
  ledDriverParameter->pwmbuffer = (uint16_t *)calloc(2, 12);
  ledDriverParameter->_spi_dev = spi;
  if (!pwmbuffer)
    return false;
  else
    return true  
}


/*!
 *  @brief  Writes PWM buffer to board
 */
void TLC59711_write(struct TLC59711 *ledDriverParameter) {
    uint8_t bufferSize = TLC59711_NUM_CHANNELS * 2 + 4; // 12 channels * 2 bytes + 4 bytes for command
    uint8_t data_to_send[bufferSize];
    uint8_t data_received;
    uint32_t command;

    // Magic word for write
    command = 0x25;

    command <<= 5;
    // OUTTMG = 1, EXTGCK = 0, TMGRST = 1, DSPRPT = 1, BLANK = 0 -> 0x16
    command |= 0x16;

    command <<= 7;
    command |= BCr;

    command <<= 7;
    command |= BCg;

    command <<= 7;
    command |= BCb;

    spi_transaction_t spiTransaction;
    memset(&spiTransaction, 0, sizeof(spiTransaction));
    spiTransaction.length = bufferSize * 8;


    data_to_send[0] =(command >> 24);
    data_to_send[1] =(command >> 16);
    data_to_send[2] =(command >> 8);
    data_to_send[3] =command;

    // 12 channels per TLC59711
    for (int8_t c = 11; c >= 0; c--) {
      // 16 bits per channel, send MSB first
      data_to_send[bufferSize - (2 * (c+1))]=(pwmbuffer[c] >> 8);
      data_to_send[bufferSize - (2 * (c+1) + 1)]=pwmbuffer[c];
    }
  
    spiTransaction.tx_buffer = data_to_send;
    spi_device_transmit(_spi_dev, &spiTransaction); 
}

/*!
 *  @brief  Set PWM value on selected channel
 *  @param  chan
 *          one from 12 channel (per driver) so there is 12 * number of drivers
 *  @param  pwm
 *          pwm value
 */
void TLC59711_setPWM(struct TLC59711 *ledDriverParameter,uint16_t chan, uint16_t pwm) {
  if (chan > 12)
    return;
  ledDriverParameter->pwmbuffer[chan] = pwm;
}

/*!
 *  @brief  Set RGB value for selected LED
 *  @param  lednum
 *          selected LED number that for which value will be set
 *  @param  r
 *          red value
 *  @param g
 *          green value
 *  @param b
 *          blue value
 */
void TLC59711_setLED(struct TLC59711 *ledDriverParameter, uint8_t lednum, uint16_t r, uint16_t g,
                               uint16_t b) {
  setPWM(lednum * 3, r);
  setPWM(lednum * 3 + 1, g);
  setPWM(lednum * 3 + 2, b);
}

/*!
 *  @brief  Get RGB value for selected LED
 *  @param  lednum
 *          selected LED number that for which value will be set
 *  @param  r
 *          red value
 *  @param g
 *          green value
 *  @param b
 *          blue value
 */
void TLC59711_getLED(struct TLC59711 *ledDriverParameter, uint8_t lednum, uint16_t &r, uint16_t &g,
                               uint16_t &b) {
  r = ledDriverParameter->pwmbuffer[lednum * 3];
  g = ledDriverParameter->pwmbuffer[lednum * 3 + 1];
  b = ledDriverParameter->pwmbuffer[lednum * 3 + 2];
}

/*!
 *  @brief  Set the brightness of LED channels to same value
 *  @param  BC
 *          Brightness Control value
 */
void TLC59711_simpleSetBrightness(struct TLC59711 *ledDriverParameter, uint8_t BC) {
  if (BC > 127) {
    BC = 127; // maximum possible value since BC can only be 7 bit
  } else if (BC < 0) {
    BC = 0;
  }
  ledDriverParameter->BCr = ledDriverParameter->BCg = ledDriverParameter->BCb = BC;
}

/*!
 *  @brief  Set the brightness of LED channels to specific value
 *  @param  bcr
 *          Brightness Control Red value
 *  @param  bcg
 *          Brightness Control Green value
 *  @param  bcb
 *          Brightness Control Blue value
 */
void TLC59711_setBrightness(struct TLC59711 *ledDriverParameter, uint8_t bcr, uint8_t bcg, uint8_t bcb) {
  if (bcr > 127) {
    bcr = 127; // maximum possible value since BC can only be 7 bit
  } else if (bcr < 0) {
    bcr = 0;
  }

  ledDriverParameter->BCr = bcr;

  if (bcg > 127) {
    bcg = 127; // maximum possible value since BC can only be 7 bit
  } else if (bcg < 0) {
    bcg = 0;
  }

  ledDriverParameter->BCg = bcg;

  if (bcb > 127) {
    bcb = 127; // maximum possible value since BC can only be 7 bit
  } else if (bcb < 0) {
    bcb = 0;
  }

  ledDriverParameter->BCb = bcb;
}

