#include "MPL3115A2.h"

#define ALTMODE ; // comment out for barometer mode; default is altitude mode

MPL3115A2::MPL3115A2()
{
}

float MPL3115A2::readBaro()
{
    // this function takes values from the read buffer and converts them to pressure units
    unsigned long m_altitude = buffer[0];
    unsigned long c_altitude = buffer[1];
    float l_altitude = (float)(buffer[2] >> 4) / 4;                    // dividing by 4, since two lowest bits are fractional value
    return ((float)(m_altitude << 10 | c_altitude << 2) + l_altitude); // shifting 2 to the left to make room for LSB
}

float MPL3115A2::readAlt()
{
    // Reads altitude data (if CTRL_REG1 is set to altitude mode)
    int m_altitude = buffer[0];
    int c_altitude = buffer[1];
    float l_altitude = (float)(buffer[2] >> 4) / 16;
    return ((float)((m_altitude << 8) | c_altitude) + l_altitude);
}

float MPL3115A2::readTemp()
{
    // reads registers from the sensor
    int m_temp = buffer[3];                        // temperature, degrees
    float l_temp = (float)(buffer[4] >> 4) / 16.0; // temperature, fraction of a degree
    return (float)(m_temp + l_temp);
}

float MPL3115A2::read()
{
    // One shot mode at 0b10101011 is slightly too fast, but better than wasting sensor cycles that increase precision
    // one reading seems to take 4ms (datasheet p.33);
    // oversampling at 32x=130ms interval between readings seems to be optimal for 10Hz
    if (oneShot())
    {
        i2c->writeByte(MPL3115A2_CTRL_REG1, 0b10111011); // bit 2 is one shot mode //0xBB = 0b10111001
        i2c->writeByte(MPL3115A2_CTRL_REG1, 0b10111001);
        i2c->readBytes(CTRL_REG_1, 5, buffer);
        // float temp = readTemp();
        float altbaro;
#ifdef ALTMODE
        altbaro = readAlt();
#else
        altbaro = readBaro();
#endif
        // baro.temp = temp;
        alt = altbaro;
        // exponential smoothing to get a smooth time series
        // baro.smooth = (baro.smooth * 3 + altbaro) / 4;
    }
    return alt;
}

void MPL3115A2::init(int basis, unsigned long clockspeed, TwoWire *wire = &Wire)
{
    clockSpeed = clockspeed;
    i2c = new I2Cdev(MPL3115A2_ADDRESS, wire);
    uint8_t whoAmI = i2c->getByte(MPL3115A2_WHOAMI);
    if (whoAmI != 196)
    {
        Serial.print(F("wrong \"who am i\" bit: "));
        Serial.println(whoAmI);
        delay(1000);
        init(basis, clockspeed, wire);
    }

    i2c->writeByte(OFF_H, 0); // write altitude offset=0 (because calculation below is based on offset=0)
    // calculate sea level pressure by averaging a few readings
    Serial.println("Pressure calibration...");
    float buff[4];
    for (byte i = 0; i < 4; i++)
    {
        i2c->writeByte(MPL3115A2_CTRL_REG1, 0b00111011); // bit 2 is one shot mode, bits 4-6 are 128x oversampling
        i2c->writeByte(MPL3115A2_CTRL_REG1, 0b00111001); // must clear oversampling (OST) bit, otherwise update will be once per second
        delay(550);                                      // wait for sensor to read pressure (512ms in datasheet)
        i2c->readBytes(CTRL_REG_1, 5, buffer);           // read sensor data
        buff[i] = readBaro();                            // read pressure
        Serial.println(buff[i]);
    }
    float currpress = (buff[0] + buff[1] + buff[2] + buff[3]) / 4; // average over two seconds

    Serial.print("Current pressure: ");
    Serial.print(currpress);
    Serial.println(" Pa");
    // calculate pressure at mean sea level based on a given altitude
    float seapress = currpress / pow(1 - basis * 0.0000225577, 5.255877);
    Serial.print("Sea level pressure: ");
    Serial.print(seapress);
    Serial.println(" Pa");
    Serial.print("Temperature: ");
    Serial.print(buffer[3] + (float)(buffer[4] >> 4) / 16);
    Serial.println(" C");

    // This configuration option calibrates the sensor according to
    // the sea level pressure for the measurement location (2 Pa per LSB)
    i2c->writeByte(0x14, (unsigned int)(seapress / 2) >> 8);   // i2c->Write(0x14, 0xC3); // BAR_IN_MSB (register 0x14):
    i2c->writeByte(0x15, (unsigned int)(seapress / 2) & 0xFF); // i2c->Write(0x15, 0xF3); // BAR_IN_LSB (register 0x15):

    // one reading seems to take 4ms (datasheet p.33);
    // oversampling 32x=130ms interval between readings seems to be best for 10Hz; slightly too slow
    // first bit is altitude mode (vs. barometer mode)

    // Altitude mode

    i2c->writeByte(MPL3115A2_CTRL_REG1, 0b10111011); // bit 2 is one shot mode //0xBB = 0b10111001
    i2c->writeByte(MPL3115A2_CTRL_REG1, 0b10111001); // must clear oversampling (OST) bit, otherwise update will be once per second
    delay(550);                                      // wait for measurement
    i2c->readBytes(CTRL_REG_1, 5, buffer);           //
    alt = readAlt();
    Serial.print("Altitude now: ");
    Serial.println(alt);
    Serial.println("Done.");
}