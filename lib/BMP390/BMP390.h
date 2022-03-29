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

#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address
/*=========================================================================*/
#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

/** BMP390 Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */

class BMP390 : public IBaro
{
public:
    BMP390();
    void init(int basis, unsigned long clockspeed, TwoWire *theWire = &Wire);
    float read();

    bool setTemperatureOversampling(uint8_t os);
    bool setPressureOversampling(uint8_t os);
    bool setIIRFilterCoeff(uint8_t fs);
    bool setOutputDataRate(uint8_t odr);

private:
    Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
    Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

    bool _init(void);

    bool _filterEnabled, _tempOSEnabled, _presOSEnabled, _ODREnabled;
    uint8_t _i2caddr;
    int32_t _sensorID;
    int8_t _cs;
    unsigned long _meas_end;

    uint8_t spixfer(uint8_t x);

    struct bmp3_dev the_sensor;
    struct bmp3_settings settings;

    float seaLevel;

    /// Perform a reading in blocking mode
    bool performReading(void);

    /// Temperature (Celsius) assigned after calling performReading()
    double temperature;
    /// Pressure (Pascals) assigned after calling performReading()
    double pressure;
    uint8_t chipID(void);
    float readTemperature(void);
    float readPressure(void);
};

#endif
