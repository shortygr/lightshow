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

#include <leddriver.hpp>
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
TLC59711::TLC59711(spi_device_handle_t spi) {


  BCr = BCg = BCb = 0x7F; // default 100% brigthness

  pwmbuffer = (uint16_t *)calloc(2, 12);

  _spi_dev = spi;
}


/*!
 *  @brief  Writes PWM buffer to board
 */
void TLC59711::write() {
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
void Adafruit_TLC59711::setPWM(uint16_t chan, uint16_t pwm) {
  if (chan > 12 * numdrivers)
    return;
  pwmbuffer[chan] = pwm;
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
void Adafruit_TLC59711::setLED(uint8_t lednum, uint16_t r, uint16_t g,
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
void Adafruit_TLC59711::getLED(uint8_t lednum, uint16_t &r, uint16_t &g,
                               uint16_t &b) {
  r = pwmbuffer[lednum * 3];
  g = pwmbuffer[lednum * 3 + 1];
  b = pwmbuffer[lednum * 3 + 2];
}

/*!
 *  @brief  Set the brightness of LED channels to same value
 *  @param  BC
 *          Brightness Control value
 */
void Adafruit_TLC59711::simpleSetBrightness(uint8_t BC) {
  if (BC > 127) {
    BC = 127; // maximum possible value since BC can only be 7 bit
  } else if (BC < 0) {
    BC = 0;
  }

  BCr = BCg = BCb = BC;
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
void Adafruit_TLC59711::setBrightness(uint8_t bcr, uint8_t bcg, uint8_t bcb) {
  if (bcr > 127) {
    bcr = 127; // maximum possible value since BC can only be 7 bit
  } else if (bcr < 0) {
    bcr = 0;
  }

  BCr = bcr;

  if (bcg > 127) {
    bcg = 127; // maximum possible value since BC can only be 7 bit
  } else if (bcg < 0) {
    bcg = 0;
  }

  BCg = bcg;

  if (bcb > 127) {
    bcb = 127; // maximum possible value since BC can only be 7 bit
  } else if (bcb < 0) {
    bcb = 0;
  }

  BCb = bcb;
}

/*!
 *  @brief  Begins SPI connection if there is not empty PWM buffer
 *  @return If successful returns true, otherwise false
 */
bool Adafruit_TLC59711::begin() {
  if (!pwmbuffer)
    return false;

  return _spi_dev->begin();
}
