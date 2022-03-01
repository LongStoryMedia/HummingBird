#include "MPL3115A2.h"

#define ALTMODE ;  // comment out for barometer mode; default is altitude mode
#define ALTBASIS 0 // start altitude to calculate mean sea level pressure in meters
// this altitude must be known (or provided by GPS etc.)

// MPL3115A2::MPL3115A2(TwoWire *_wire = &Wire)
// {
//     i2c = new I2C(MPL3115A2_ADDRESS, 5, _wire);
//     _wire->begin();
//     wire = _wire;
// }

// I2C i2c(MPL3115A2_ADDRESS, 5, &Wire1);
byte buffer[5] = {0, 0, 0, 0, 0}; // buffer for sensor data
TwoWire *wire = &Wire1;

byte alt_read(byte regAddr)
{
    // This function reads one byte over I2C
    wire->beginTransmission(0x60);
    wire->write(regAddr);         // Address of CTRL_REG1
    wire->endTransmission(false); // Send data to I2C dev with option for a repeated start. Works in Arduino V1.0.1
    wire->requestFrom(0x60, 1);
    return wire->read();
}

void alt_readBytes()
{ // Read Altitude/Barometer and Temperature data (5 bytes)
    // This is faster than reading individual register, as the sensor automatically increments the register address,
    // so we just keep reading...
    byte i = 0;
    wire->beginTransmission(0x60);
    wire->write(0x01); // Address of CTRL_REG1
    wire->endTransmission(false);
    wire->requestFrom(0x60, 5); // read 5 bytes: 3 for altitude or pressure, 2 for temperature
    while (wire->available())
        buffer[i++] = wire->read();
}

void alt_write(byte regAddr, byte value)
{
    // This function writes one byto over I2C
    wire->beginTransmission(0x60);
    wire->write(regAddr);
    wire->write(value);
    wire->endTransmission(true);
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

MPL3115A2::_baro MPL3115A2::read()
{
    // This function reads the altitude (or barometer) and temperature registers, then prints their values
    // variables for the calculations
    int m_temp;
    float l_temp;
    float altbaro, temperature;

// One shot mode at 0b10101011 is slightly too fast, but better than wasting sensor cycles that increase precision
// one reading seems to take 4ms (datasheet p.33);
// oversampling at 32x=130ms interval between readings seems to be optimal for 10Hz
#ifdef ALTMODE                   // Altitude mode
    alt_write(0x26, 0b10111011); // bit 2 is one shot mode //0xB9 = 0b10111001
    alt_write(0x26, 0b10111001); // must clear oversampling (OST) bit, otherwise update will be once per second
#else                            // Barometer mode
    alt_write(0x26, 0b00111011); // bit 2 is one shot mode //0xB9 = 0b10111001
    alt_write(0x26, 0b00111001); // must clear oversampling (OST) bit, otherwise update will be once per second
#endif
    delay(100); // read with 10Hz; drop this if calling from an outer loop

    alt_readBytes();                         // reads registers from the sensor
    m_temp = buffer[3];                      // temperature, degrees
    l_temp = (float)(buffer[4] >> 4) / 16.0; // temperature, fraction of a degree
    temperature = (float)(m_temp + l_temp);

#ifdef ALTMODE // converts byte data into float; change function to Alt_Read() or Baro_Read()
    altbaro = readAlt();
#else
    altbaro = readBaro();
#endif
    //     // One shot mode at 0b10101011 is slightly too fast, but better than wasting sensor cycles that increase precision
    //     // one reading seems to take 4ms (datasheet p.33);
    //     // oversampling at 32x=130ms interval between readings seems to be optimal for 10Hz
    //     // bool newVal = oneShot();

    //     // if (lastUpdateTime + 100 <= millis())
    //     // {
    // #ifdef ALTMODE                       // Altitude mode
    //         alt_write(0x26, 0b10111011); // bit 2 is one shot mode //0xB9 = 0b10111001
    //         alt_write(0x26, 0b10111001); // must clear oversampling (OST) bit, otherwise update will be once per second
    // #else                                // Barometer mode
    //         alt_write(0x26, 0b00111011); // bit 2 is one shot mode //0xB9 = 0b10111001
    //         alt_write(0x26, 0b00111001); // must clear oversampling (OST) bit, otherwise update will be once per second
    // #endif
    //         lastUpdateTime = millis();
    //         alt_readBytes();
    //         float temp = readTemp();
    //         float altbaro;
    // #ifdef ALTMODE // converts byte data into float; change function to Alt_Read() or Baro_Read()
    //         altbaro = readAlt();
    // #else
    //         altbaro = readBaro();
    // #endif

    baro.temp = temperature;
    baro.raw = altbaro;
    // exponential smoothing to get a smooth time series
    baro.smooth = (baro.smooth * 3 + altbaro) / 4;
    // }
    return baro;
}

bool MPL3115A2::oneShot()
{
    if (lastUpdateTime + 100 <= millis())
    {
#ifdef ALTMODE                                      // Altitude mode
        alt_write(MPL3115A2_CTRL_REG1, 0b10111011); // bit 2 is one shot mode //0xB9 = 0b10111001
        alt_write(MPL3115A2_CTRL_REG1, 0b10111001); // must clear oversampling (OST) bit, otherwise update will be once per second
#else                                               // Barometer mode
        alt_write(MPL3115A2_CTRL_REG1, 0b00111011); // bit 2 is one shot mode //0xB9 = 0b10111001
        alt_write(MPL3115A2_CTRL_REG1, 0b00111001); // must clear oversampling (OST) bit, otherwise update will be once per second
#endif
        lastUpdateTime = millis();
        return true;
    }
    return false;
}

void MPL3115A2::init()
{
    wire->begin();
    byte whoAmI = alt_read(0x0C);
    if (whoAmI != 196)
    {
        Serial.print(F("wrong \"who am i\" bit: "));
        Serial.println(whoAmI);
        delay(1000);
        init();
    }

    alt_write(0x2D, 0); // write altitude offset=0 (because calculation below is based on offset=0)
    // calculate sea level pressure by averaging a few readings
    Serial.println("Pressure calibration...");
    float buff[4];
    for (byte i = 0; i < 4; i++)
    {
        alt_write(0x26, 0b00111011); // bit 2 is one shot mode, bits 4-6 are 128x oversampling
        alt_write(0x26, 0b00111001); // must clear oversampling (OST) bit, otherwise update will be once per second
        delay(550);                  // wait for sensor to read pressure (512ms in datasheet)
        alt_readBytes();             // read sensor data
        buff[i] = readBaro();        // read pressure
        Serial.println(buff[i]);
    }
    float currpress = (buff[0] + buff[1] + buff[2] + buff[3]) / 4; // average over two seconds

    Serial.print("Current pressure: ");
    Serial.print(currpress);
    Serial.println(" Pa");
    // calculate pressure at mean sea level based on a given altitude
    float seapress = currpress / pow(1 - ALTBASIS * 0.0000225577, 5.255877);
    Serial.print("Sea level pressure: ");
    Serial.print(seapress);
    Serial.println(" Pa");
    Serial.print("Temperature: ");
    Serial.print(buffer[3] + (float)(buffer[4] >> 4) / 16);
    Serial.println(" C");

    // This configuration option calibrates the sensor according to
    // the sea level pressure for the measurement location (2 Pa per LSB)
    alt_write(0x14, (unsigned int)(seapress / 2) >> 8);   // alt_Write(0x14, 0xC3); // BAR_IN_MSB (register 0x14):
    alt_write(0x15, (unsigned int)(seapress / 2) & 0xFF); // alt_Write(0x15, 0xF3); // BAR_IN_LSB (register 0x15):

    // one reading seems to take 4ms (datasheet p.33);
    // oversampling 32x=130ms interval between readings seems to be best for 10Hz; slightly too slow
    // first bit is altitude mode (vs. barometer mode)

    // Altitude mode

    alt_write(0x26, 0b10111011); // bit 2 is one shot mode //0xB9 = 0b10111001
    alt_write(0x26, 0b10111001); // must clear oversampling (OST) bit, otherwise update will be once per second
    delay(550);                  // wait for measurement
    alt_readBytes();             //
    baro.smooth = readAlt();
    Serial.print("Altitude now: ");
    Serial.println(baro.smooth);
    Serial.println("Done.");
}
