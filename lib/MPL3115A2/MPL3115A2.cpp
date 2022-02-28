/*
 MPL3115A2 Barometric Pressure Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 22nd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This library allows an Arduino to read from the MPL3115A2 low-cost high-precision pressure sensor.

 If you have feature suggestions or need support please use the github support page: https://github.com/sparkfun/MPL3115A2_Breakout

 Hardware Setup: The MPL3115A2 lives on the I2C bus. Attach the SDA pin to A4, SCL to A5. Use inline 10k resistors
 if you have a 5V board. If you are using the SparkFun breakout board you *do not* need 4.7k pull-up resistors
 on the bus (they are built-in).

 Link to the breakout board product:

 Software:
 .begin() Gets sensor on the I2C bus.
 .readAltitude() Returns float with meters above sealevel. Ex: 1638.94
 .readAltitudeFt() Returns float with feet above sealevel. Ex: 5376.68
 .readPressure() Returns float with barometric pressure in Pa. Ex: 83351.25
 .readTemp() Returns float with current temperature in Celsius. Ex: 23.37
 .readTempF() Returns float with current temperature in Fahrenheit. Ex: 73.96
 .setModeBarometer() Puts the sensor into Pascal measurement mode.
 .setModeAltimeter() Puts the sensor into altimetery mode.
 .setModeStandy() Puts the sensor into Standby mode. Required when changing CTRL1 register.
 .setModeActive() Start taking measurements!
 .setOversampleRate(byte) Sets the # of samples from 1 to 128. See datasheet.
 .enableEventFlags() Sets the fundamental event flags. Required during setup.

 */

#include <Wire.h>

#include "MPL3115A2.h"

// Returns the number of meters above sea level
// Returns -1 if no new data is available
float MPL3115A2::readAltitude()
{
    toggleOneShot(); // Toggle the OST bit causing the sensor to immediately take another reading

    // Wait for PDR bit, indicates we have new pressure data
    int counter = 0;
    while ((IIC_Read(STATUS) & (1 << 1)) == 0)
    {
        if (++counter > 600)
            return (-999); // Error out after max of 512ms for a read
        delay(1);
    }

    // Read pressure registers
    Wire1.beginTransmission(MPL3115A2_ADDRESS);
    Wire1.write(OUT_P_MSB);                  // Address of data to get
    Wire1.endTransmission(false);            // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
    Wire1.requestFrom(MPL3115A2_ADDRESS, 3); // Request three bytes

    // Wait for data to become available
    counter = 0;
    while (Wire1.available() < 3)
    {
        if (counter++ > 100)
            return (-999); // Error out
        delay(1);
    }

    byte msb, csb, lsb;
    msb = Wire1.read();
    csb = Wire1.read();
    lsb = Wire1.read();

    // The least significant bytes l_altitude and l_temp are 4-bit,
    // fractional values, so you must cast the calulation in (float),
    // shift the value over 4 spots to the right and divide by 16 (since
    // there are 16 values in 4-bits).
    float tempcsb = (lsb >> 4) / 16.0;

    float altitude = (float)((msb << 8) | csb) + tempcsb;

    return (altitude);
}

// Returns the number of feet above sea level
float MPL3115A2::readAltitudeFt()
{
    return (readAltitude() * 3.28084);
}

// Reads the current pressure in Pa
// Unit must be set in barometric pressure mode
// Returns -1 if no new data is available
float MPL3115A2::readPressure()
{
    // Check PDR bit, if it's not set then toggle OST
    if (IIC_Read(STATUS) & (1 << 2) == 0)
        toggleOneShot(); // Toggle the OST bit causing the sensor to immediately take another reading

    // Wait for PDR bit, indicates we have new pressure data
    int counter = 0;
    while (IIC_Read(STATUS) & (1 << 2) == 0)
    {
        if (++counter > 600)
            return (-999); // Error out after max of 512ms for a read
        delay(1);
    }

    // Read pressure registers
    Wire1.beginTransmission(MPL3115A2_ADDRESS);
    Wire1.write(OUT_P_MSB);                  // Address of data to get
    Wire1.endTransmission(false);            // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
    Wire1.requestFrom(MPL3115A2_ADDRESS, 3); // Request three bytes

    // Wait for data to become available
    counter = 0;
    while (Wire1.available() < 3)
    {
        if (counter++ > 100)
            return (-999); // Error out
        delay(1);
    }

    byte msb, csb, lsb;
    msb = Wire1.read();
    csb = Wire1.read();
    lsb = Wire1.read();

    toggleOneShot(); // Toggle the OST bit causing the sensor to immediately take another reading

    // Pressure comes back as a left shifted 20 bit number
    long pressure_whole = (long)msb << 16 | (long)csb << 8 | (long)lsb;
    pressure_whole >>= 6; // Pressure is an 18 bit number with 2 bits of decimal. Get rid of decimal portion.

    lsb &= 0b00110000;                         // Bits 5/4 represent the fractional component
    lsb >>= 4;                                 // Get it right aligned
    float pressure_decimal = (float)lsb / 4.0; // Turn it into fraction

    float pressure = (float)pressure_whole + pressure_decimal;

    return (pressure);
}

float MPL3115A2::readTemp()
{
    if (IIC_Read(STATUS) & (1 << 1) == 0)
        toggleOneShot(); // Toggle the OST bit causing the sensor to immediately take another reading

    // Wait for TDR bit, indicates we have new temp data
    int counter = 0;
    while ((IIC_Read(STATUS) & (1 << 1)) == 0)
    {
        if (++counter > 600)
            return (-999); // Error out after max of 512ms for a read
        delay(1);
    }

    // Read temperature registers
    Wire1.beginTransmission(MPL3115A2_ADDRESS);
    Wire1.write(OUT_T_MSB);                  // Address of data to get
    Wire1.endTransmission(false);            // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
    Wire1.requestFrom(MPL3115A2_ADDRESS, 2); // Request two bytes

    // Wait for data to become available
    counter = 0;
    while (Wire1.available() < 2)
    {
        if (counter++ > 100)
            return (-999); // Error out
        delay(1);
    }

    byte msb, lsb;
    msb = Wire1.read();
    lsb = Wire1.read();

    toggleOneShot(); // Toggle the OST bit causing the sensor to immediately take another reading

    // Negative temperature fix by D.D.G.
    word foo = 0;
    bool negSign = false;

    // Check for 2s compliment
    if (msb > 0x7F)
    {
        foo = ~((msb << 8) + lsb) + 1; // 2s complement
        msb = foo >> 8;
        lsb = foo & 0x00F0;
        negSign = true;
    }

    // The least significant bytes l_altitude and l_temp are 4-bit,
    // fractional values, so you must cast the calulation in (float),
    // shift the value over 4 spots to the right and divide by 16 (since
    // there are 16 values in 4-bits).
    float templsb = (lsb >> 4) / 16.0; // temp, fraction of a degree

    float temperature = (float)(msb + templsb);

    if (negSign)
        temperature = 0 - temperature;

    return (temperature);
}

// Give me temperature in fahrenheit!
float MPL3115A2::readTempF()
{
    return ((readTemp() * 9.0) / 5.0 + 32.0); // Convert celsius to fahrenheit
}

// Sets the mode to Barometer
// CTRL_REG1, ALT bit
void MPL3115A2::setModeBarometer()
{
    byte tempSetting = IIC_Read(CTRL_REG1); // Read current settings
    tempSetting &= ~(1 << 7);               // Clear ALT bit
    IIC_Write(CTRL_REG1, tempSetting);
}

// Sets the mode to Altimeter
// CTRL_REG1, ALT bit
void MPL3115A2::setModeAltimeter()
{
    byte tempSetting = IIC_Read(CTRL_REG1); // Read current settings
    tempSetting |= (1 << 7);                // Set ALT bit
    IIC_Write(CTRL_REG1, tempSetting);
}

// Puts the sensor in standby mode
// This is needed so that we can modify the major control registers
void MPL3115A2::setModeStandby()
{
    byte tempSetting = IIC_Read(CTRL_REG1); // Read current settings
    tempSetting &= ~(1 << 0);               // Clear SBYB bit for Standby mode
    IIC_Write(CTRL_REG1, tempSetting);
}

// Puts the sensor in active mode
// This is needed so that we can modify the major control registers
void MPL3115A2::setModeActive()
{
    byte tempSetting = IIC_Read(CTRL_REG1); // Read current settings
    tempSetting |= (1 << 0);                // Set SBYB bit for Active mode
    IIC_Write(CTRL_REG1, tempSetting);
}

// Call with a rate from 0 to 7. See page 33 for table of ratios.
// Sets the over sample rate. Datasheet calls for 128 but you can set it
// from 1 to 128 samples. The higher the oversample rate the greater
// the time between data samples.
void MPL3115A2::setOversampleRate(byte sampleRate)
{
    if (sampleRate > 7)
        sampleRate = 7; // OS cannot be larger than 0b.0111
    sampleRate <<= 3;   // Align it for the CTRL_REG1 register

    byte tempSetting = IIC_Read(CTRL_REG1); // Read current settings
    tempSetting &= 0b11000111;              // Clear out old OS bits
    tempSetting |= sampleRate;              // Mask in new OS bits
    IIC_Write(CTRL_REG1, tempSetting);
}

// Enables the pressure and temp measurement event flags so that we can
// test against them. This is recommended in datasheet during setup.
void MPL3115A2::enableEventFlags()
{
    IIC_Write(PT_DATA_CFG, 0x07); // Enable all three pressure and temp event flags
}

// Clears then sets the OST bit which causes the sensor to immediately take another reading
// Needed to sample faster than 1Hz
void MPL3115A2::toggleOneShot(void)
{
    byte tempSetting = IIC_Read(CTRL_REG1); // Read current settings
    tempSetting &= ~(1 << 1);               // Clear OST bit
    IIC_Write(CTRL_REG1, tempSetting);

    tempSetting = IIC_Read(CTRL_REG1); // Read current settings to be safe
    tempSetting |= (1 << 1);           // Set OST bit
    IIC_Write(CTRL_REG1, tempSetting);
}

// These are the two I2C functions in this sketch.
byte MPL3115A2::IIC_Read(byte regAddr)
{
    // This function reads one byte over IIC
    Wire1.beginTransmission(MPL3115A2_ADDRESS);
    Wire1.write(regAddr);                    // Address of CTRL_REG1
    Wire1.endTransmission(false);            // Send data to I2C dev with option for a repeated start. THIS IS NECESSARY and not supported before Arduino V1.0.1!
    Wire1.requestFrom(MPL3115A2_ADDRESS, 1); // Request the data...
    return Wire1.read();
}

void MPL3115A2::IIC_Write(byte regAddr, byte value)
{
    // This function writes one byto over IIC
    Wire1.beginTransmission(MPL3115A2_ADDRESS);
    Wire1.write(regAddr);
    Wire1.write(value);
    Wire1.endTransmission(true);
}

/*!
 *   @brief  Setups the HW (reads coefficients values, etc.)
 *   @param  twoWire
 *           Optional TwoWire I2C object
 *   @return true on successful startup, false otherwise
 */
boolean MPL3115A2::begin(TwoWire *twoWire)
{
    if (i2c_dev)
        delete i2c_dev;
    i2c_dev = new I2C(MPL3115A2_ADDRESS, twoWire);
    if (!i2c_dev->begin())
        return false;

    // sanity check
    uint8_t whoami = read8(MPL3115A2_WHOAMI);
    if (whoami != 0xC4)
    {
        return false;
    }

    i2c_dev->setSpeed(1000000);

    // software reset
    write8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
    while (read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST)
        delay(10);

    // set oversampling and altitude mode
    _ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
    write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

    // enable data ready events for pressure/altitude and temperature
    write8(MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
                                      MPL3115A2_PT_DATA_CFG_PDEFE |
                                      MPL3115A2_PT_DATA_CFG_DREM);

    return true;
}

/*!
 *  @brief  Set the local sea level pressure
 *  @param SLP sea level pressure in hPa
 */
void MPL3115A2::setSeaPressure(float SLP)
{
    // multiply by 100 to convert hPa to Pa
    // divide by 2 to convert to 2 Pa per LSB
    // convert to integer
    uint16_t bar = SLP * 50;

    // write result to register
    uint8_t buffer[3];
    buffer[0] = MPL3115A2_BAR_IN_MSB;
    buffer[1] = bar >> 8;
    buffer[2] = bar & 0xFF;
    i2c_dev->write(buffer, 3);
}

/*!
 *  @brief  read 1 byte of data at the specified address
 *  @param  a
 *          the address to read
 *  @return the read data byte
 */
uint8_t MPL3115A2::read8(uint8_t a)
{
    uint8_t buffer[1] = {a};
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
    return buffer[0];
}

/*!
 *  @brief  write a byte of data to the specified address
 *  @param  a
 *          the address to write to
 *  @param  d
 *          the byte to write
 */
void MPL3115A2::write8(uint8_t a, uint8_t d)
{
    uint8_t buffer[2] = {a, d};
    i2c_dev->write(buffer, 2);
}