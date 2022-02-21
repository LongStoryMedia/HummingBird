#ifndef MPL3115A2_H
#define MPL3115A2_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "Arduino.h"
#include "../I2C/I2C.h"

#define MPL3115A2_ADDRESS (0x60) ///< default I2C address 1100000

/** MPL3115A2 registers **/
enum
{
    MPL3115A2_REGISTER_STATUS = (0x00),

    MPL3115A2_REGISTER_PRESSURE_MSB = (0x01),
    MPL3115A2_REGISTER_PRESSURE_CSB = (0x02),
    MPL3115A2_REGISTER_PRESSURE_LSB = (0x03),

    MPL3115A2_REGISTER_TEMP_MSB = (0x04),
    MPL3115A2_REGISTER_TEMP_LSB = (0x05),

    MPL3115A2_REGISTER_DR_STATUS = (0x06),

    MPL3115A2_OUT_P_DELTA_MSB = (0x07),
    MPL3115A2_OUT_P_DELTA_CSB = (0x08),
    MPL3115A2_OUT_P_DELTA_LSB = (0x09),

    MPL3115A2_OUT_T_DELTA_MSB = (0x0A),
    MPL3115A2_OUT_T_DELTA_LSB = (0x0B),

    MPL3115A2_WHOAMI = (0x0C),

    MPL3115A2_BAR_IN_MSB = (0x14),
    MPL3115A2_BAR_IN_LSB = (0x15),
};

/** MPL3115A2 status register bits **/
enum
{
    MPL3115A2_REGISTER_STATUS_TDR = 0x02,
    MPL3115A2_REGISTER_STATUS_PDR = 0x04,
    MPL3115A2_REGISTER_STATUS_PTDR = 0x08,
};

/** MPL3115A2 PT DATA register bits **/
enum
{
    MPL3115A2_PT_DATA_CFG = 0x13,
    MPL3115A2_PT_DATA_CFG_TDEFE = 0x01,
    MPL3115A2_PT_DATA_CFG_PDEFE = 0x02,
    MPL3115A2_PT_DATA_CFG_DREM = 0x04,
};

/** MPL3115A2 control registers **/
enum
{

    MPL3115A2_CTRL_REG1 = (0x26),
    MPL3115A2_CTRL_REG2 = (0x27),
    MPL3115A2_CTRL_REG3 = (0x28),
    MPL3115A2_CTRL_REG4 = (0x29),
    MPL3115A2_CTRL_REG5 = (0x2A),
};

/** MPL3115A2 control register bits **/
enum
{
    MPL3115A2_CTRL_REG1_SBYB = 0x01,
    MPL3115A2_CTRL_REG1_OST = 0x02,
    MPL3115A2_CTRL_REG1_RST = 0x04,
    MPL3115A2_CTRL_REG1_RAW = 0x40,
    MPL3115A2_CTRL_REG1_ALT = 0x80,
    MPL3115A2_CTRL_REG1_BAR = 0x00,
};

/** MPL3115A2 oversample values **/
enum
{
    MPL3115A2_CTRL_REG1_OS1 = 0x00,
    MPL3115A2_CTRL_REG1_OS2 = 0x08,
    MPL3115A2_CTRL_REG1_OS4 = 0x10,
    MPL3115A2_CTRL_REG1_OS8 = 0x18,
    MPL3115A2_CTRL_REG1_OS16 = 0x20,
    MPL3115A2_CTRL_REG1_OS32 = 0x28,
    MPL3115A2_CTRL_REG1_OS64 = 0x30,
    MPL3115A2_CTRL_REG1_OS128 = 0x38,
};

#define MPL3115A2_REGISTER_STARTCONVERSION (0x12) ///< start conversion
#define MPL3115A2_ADDRESS 0x60                    // Unshifted 7-bit I2C address for sensor

#define STATUS 0x00
#define OUT_P_MSB 0x01
#define OUT_P_CSB 0x02
#define OUT_P_LSB 0x03
#define OUT_T_MSB 0x04
#define OUT_T_LSB 0x05
#define DR_STATUS 0x06
#define OUT_P_DELTA_MSB 0x07
#define OUT_P_DELTA_CSB 0x08
#define OUT_P_DELTA_LSB 0x09
#define OUT_T_DELTA_MSB 0x0A
#define OUT_T_DELTA_LSB 0x0B
#define WHO_AM_I 0x0C
#define F_STATUS 0x0D
#define F_DATA 0x0E
#define F_SETUP 0x0F
#define TIME_DLY 0x10
#define SYSMOD 0x11
#define INT_SOURCE 0x12
#define PT_DATA_CFG 0x13
#define BAR_IN_MSB 0x14
#define BAR_IN_LSB 0x15
#define P_TGT_MSB 0x16
#define P_TGT_LSB 0x17
#define T_TGT 0x18
#define P_WND_MSB 0x19
#define P_WND_LSB 0x1A
#define T_WND 0x1B
#define P_MIN_MSB 0x1C
#define P_MIN_CSB 0x1D
#define P_MIN_LSB 0x1E
#define T_MIN_MSB 0x1F
#define T_MIN_LSB 0x20
#define P_MAX_MSB 0x21
#define P_MAX_CSB 0x22
#define P_MAX_LSB 0x23
#define T_MAX_MSB 0x24
#define T_MAX_LSB 0x25
#define CTRL_REG1 0x26
#define CTRL_REG2 0x27
#define CTRL_REG3 0x28
#define CTRL_REG4 0x29
#define CTRL_REG5 0x2A
#define OFF_P 0x2B
#define OFF_T 0x2C
#define OFF_H 0x2D

class MPL3115A2
{

public:
    // Public Functions
    float readAltitude();         // Returns float with meters above sealevel. Ex: 1638.94
    float readAltitudeFt();       // Returns float with feet above sealevel. Ex: 5376.68
    float readPressure();         // Returns float with barometric pressure in Pa. Ex: 83351.25
    float readTemp();             // Returns float with current temperature in Celsius. Ex: 23.37
    float readTempF();            // Returns float with current temperature in Fahrenheit. Ex: 73.96
    void setModeBarometer();      // Puts the sensor into Pascal measurement mode.
    void setModeAltimeter();      // Puts the sensor into altimetery mode.
    void setModeStandby();        // Puts the sensor into Standby mode. Required when changing CTRL1 register.
    void setModeActive();         // Start taking measurements!
    void setOversampleRate(byte); // Sets the # of samples from 1 to 128. See datasheet.
    void enableEventFlags();      // Sets the fundamental event flags. Required during setup.
    boolean begin(TwoWire *twoWire = &Wire);
    void setSeaPressure(float SLP);
    void write8(uint8_t a, uint8_t d);

    // Public Variables

private:
    // Private Functions

    void toggleOneShot();
    byte IIC_Read(byte regAddr);
    void IIC_Write(byte regAddr, byte value);
    I2C *i2c_dev = NULL; ///< Pointer to I2C bus interface
    uint8_t read8(uint8_t a);
    uint8_t mode;

    typedef union
    {
        struct
        {
            uint8_t SBYB : 1;
            uint8_t OST : 1;
            uint8_t RST : 1;
            uint8_t OS : 3;
            uint8_t RAW : 1;
            uint8_t ALT : 1;
        } bit;
        uint8_t reg;
    } ctrl_reg1;
    ctrl_reg1 _ctrl_reg1;
    // Private Variables
};

#endif