#include "BMP390.h"
#include "Arduino.h"

I2Cdev *g_i2c = NULL; ///< Global I2C interface pointer

// Our hardware interface functions
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static void delay_usec(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);
/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates sensor
*/
/**************************************************************************/
BMP390::BMP390(void)
{
    data = {0};
    settings = {0};
    status = {{0}};
}

/**************************************************************************/

/**************************************************************************/
void BMP390::init(int basis, unsigned long clockspeed, TwoWire *wire)
{
    if (i2c)
        delete i2c;

    // TODO: use basis to calculate;
    seaLevel = 1013.25;
    clockSpeed = clockspeed;

    g_i2c = i2c = new I2Cdev(BMP3XX_DEFAULT_ADDRESS, wire);

    // verify i2c address was found
    if (!i2c->begin())
    {
        return;
    }

    configureDevice();

    int8_t rslt = BMP3_OK;

    /* Reset the sensor */
    rslt = bmp3_soft_reset(&dev);
    if (rslt != BMP3_OK)
    {
        Serial.print(F("error resetting bmp390"));
        return init(basis, clockspeed, wire);
    }

    rslt = bmp3_init(&dev);
    rslt = validate_trimming_param(&dev);

    if (rslt != BMP3_OK)
    {
        Serial.print(F("error initializing bmp390"));
        return init(basis, clockspeed, wire);
    }
    configureDevice();
}

void BMP390::configureDevice()
{
    uint16_t settings_sel = 0;

    dev.chip_id = i2c->address();
    dev.intf = BMP3_I2C_INTF;
    dev.read = &i2c_read;
    dev.write = &i2c_write;
    dev.intf_ptr = g_i2c;
    dev.dummy_byte = 0;
    dev.delay_us = delay_usec;

    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.odr = BMP3_ODR_200_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_ODR;
    settings.op_mode = BMP3_MODE_NORMAL;

    bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_set_op_mode(&settings, &dev);
}

/**************************************************************************/
/*!
    @brief Calculates the altitude (in meters).

    Reads the current atmostpheric pressure (in hPa) from the sensor and
   calculates via the provided sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @return Altitude in meters
*/
/**************************************************************************/
float BMP390::read()
{
    if (oneShot())
    {
        bmp3_data comp_data;
        int8_t getDataStatus = bmp3_get_sensor_data(BMP3_PRESS, &comp_data, &dev);

        if (getDataStatus != BMP3_OK)
        {
            Serial.print("could not get sensor data due to error code: ");
            Serial.println(getDataStatus);
        }
        float atmospheric = data.pressure / 100.00F;
        alt = 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
    }

    return alt;
}

/**************************************************************************/
/*!
    @brief  Reads 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                void *intf_ptr)
{
    // Serial.print("I2C read address 0x"); Serial.print(reg_addr, HEX);
    // Serial.print(" len "); Serial.println(len, HEX);

    if (!g_i2c->write_then_read(&reg_addr, 1, reg_data, len))
    {
        return 1;
    }

    return 0;
}

/**************************************************************************/
/*!
    @brief  Writes 8 bit values over I2C
*/
/**************************************************************************/
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                 void *intf_ptr)
{
    // Serial.print("I2C write address 0x"); Serial.print(reg_addr, HEX);
    // Serial.print(" len "); Serial.println(len, HEX);

    if (!g_i2c->write((uint8_t *)reg_data, len, true, &reg_addr, 1))
    {
        return 1;
    }

    return 0;
}

static void delay_usec(uint32_t us, void *intf_ptr) { delayMicroseconds(us); }

static int8_t validate_trimming_param(struct bmp3_dev *dev)
{
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;

    rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
    if (rslt == BMP3_OK)
    {
        for (i = 0; i < 21; i++)
        {
            crc = (uint8_t)cal_crc(crc, trim_param[i]);
        }

        crc = (crc ^ 0xFF);
        rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
        if (stored_crc != crc)
        {
            rslt = -1;
        }
    }

    return rslt;
}

/*
 * @brief function to calculate CRC for the trimming parameters
 * */
static int8_t cal_crc(uint8_t seed, uint8_t data)
{
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if ((seed & 0x80) ^ (data & 0x80))
        {
            var2 = 1;
        }
        else
        {
            var2 = 0;
        }

        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t)(poly * var2);
    }

    return (int8_t)seed;
}