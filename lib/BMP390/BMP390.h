/*!
 * @file BMP390.h
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"
#include "IBaro.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address
/*=========================================================================*/

/* Defines watermark level of frame count requested
 * As, only Pressure is enabled in this example,
 * Total byte count requested : FIFO_FRAME_COUNT * BMP3_LEN_P_OR_T_HEADER_DATA
 */
#define FIFO_FRAME_COUNT UINT8_C(50)

/* Maximum FIFO size */
#define FIFO_MAX_SIZE UINT16_C(512)

/* Iteration count to run example code */
#define ITERATION UINT8_C(10)
class BMP390 : public IBaro
{
public:
    BMP390();
    void init(int basis, unsigned long clockspeed, TwoWire *theWire = &Wire);
    float read();

private:
    struct bmp3_dev dev;
    struct bmp3_settings settings;
    struct bmp3_data data;
    struct bmp3_status status;

    float seaLevel;
    /// Temperature (Celsius) assigned after calling performReading()
    double temperature;
    /// Pressure (Pascals) assigned after calling performReading()
    double pressure;
    void configureDevice();
};

#endif
