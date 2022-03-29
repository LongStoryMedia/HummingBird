// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//      2015-10-30 - simondlevy : support i2c_t3 for Teensy3.1
//      2013-05-06 - add Francesco Ferrara's Fastwire v0.24 implementation with small modifications
//      2013-05-05 - fix issue with writing bit values to words (Sasquatch/Farzanegan)
//      2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                 - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//      2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//      2011-10-03 - added automatic Arduino version detection for ease of use
//      2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//      2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//      2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//      2011-08-02 - added support for 16-bit registers
//                 - fixed incorrect Doxygen comments on some methods
//                 - added timeout value for read operations (thanks mem @ Arduino forums)
//      2011-07-30 - changed read/write function structures to return success or byte counts
//                 - made all methods static for multi-device memory savings
//      2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

// -----------------------------------------------------------------------------
// I2C interface implementation setting
// -----------------------------------------------------------------------------
#ifndef I2CDEV_IMPLEMENTATION
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
//#define I2CDEV_IMPLEMENTATION       I2CDEV_BUILTIN_SBWIRE
//#define I2CDEV_IMPLEMENTATION       I2CDEV_BUILTIN_FASTWIRE
#endif // I2CDEV_IMPLEMENTATION

// comment this out if you are using a non-optimal IDE/implementation setting
// but want the compiler to shut up about it
#define I2CDEV_IMPLEMENTATION_WARNINGS

// -----------------------------------------------------------------------------
// I2C interface implementation options
// -----------------------------------------------------------------------------
#define I2CDEV_ARDUINO_WIRE 1      // Wire object from Arduino
#define I2CDEV_BUILTIN_NBWIRE 2    // Tweaked Wire object from Gene Knight's NBWire project
                                   // ^^^ NBWire implementation is still buggy w/some interrupts!
#define I2CDEV_BUILTIN_FASTWIRE 3  // FastWire object from Francesco Ferrara's project
#define I2CDEV_I2CMASTER_LIBRARY 4 // I2C object from DSSCircuits I2C-Master Library at https://github.com/DSSCircuits/I2C-Master-Library
#define I2CDEV_BUILTIN_SBWIRE 5    // I2C object from Shuning (Steve) Bian's SBWire Library at https://github.com/freespace/SBWire

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define I2CDEV_SERIAL_DEBUG

#ifdef ARDUINO
#if ARDUINO < 100
#include "WProgram.h"
#else
#include "Arduino.h"
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_I2CMASTER_LIBRARY
#include <I2C.h>
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE
#include "SBWire.h"
#endif
#endif

#ifdef SPARK
#include <spark_wiring_i2c.h>
#define ARDUINO 101
#endif

/**\name    UTILITY MACROS  */
#define SET_LOW_BYTE UINT16_C(0x00FF)
#define SET_HIGH_BYTE UINT16_C(0xFF00)
// 1000ms default read timeout (modify with "1000 = [ms];")
#define I2CDEV_DEFAULT_READ_TIMEOUT 1000

#define CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

#define SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~bitname) |              \
     ((data << bitname) & bitname))

/* Macro variant to handle the bitname position if it is zero */
#define SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~bitname) |                    \
     (data & bitname))

#define GET_BITS(reg_data, bitname) ((reg_data & (bitname##_MSK)) >> \
                                     (bitname##_POS))

/* Macro variant to handle the bitname position if it is zero */
#define GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

#define GET_LSB(var) (uint8_t)(var & BMP3_SET_LOW_BYTE)
#define GET_MSB(var) (uint8_t)((var & BMP3_SET_HIGH_BYTE) >> 8)

class I2Cdev
{
public:
    I2Cdev(uint8_t _addr, TwoWire *_wire = &Wire);
    uint8_t address(void);
    bool begin(bool addr_detect = true);
    void end(void);
    bool detected(void);

    bool read(uint8_t *buffer, size_t len, bool stop = true);
    bool write(const uint8_t *buffer, size_t len, bool stop = true,
               const uint8_t *prefix_buffer = NULL, size_t prefix_len = 0);
    bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                         uint8_t *read_buffer, size_t read_len,
                         bool stop = false);
    bool setSpeed(uint32_t desiredclk);

    /*!   @brief  How many bytes we can read in a transaction
     *    @return The size of the Wire receive/transmit buffer */
    size_t maxBufferSize() { return _maxBufferSize; }

    bool readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout = 1000);
    bool readBitW(uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout = 1000);
    bool readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout = 1000);
    bool readBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout = 1000);
    bool getByte(byte regAddr);
    bool readByte(uint8_t regAddr, uint8_t *data, uint16_t timeout = 1000);
    bool readWord(uint8_t regAddr, uint16_t *data, uint16_t timeout = 1000);
    bool readBytes(uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = 1000);
    bool readBytes(uint8_t length, uint8_t *data);
    bool readWords(uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout = 1000);

    bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    bool writeBitW(uint8_t regAddr, uint8_t bitNum, uint16_t data);
    bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    bool writeBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
    bool writeByte(uint8_t regAddr, uint8_t data);
    bool writeWord(uint8_t regAddr, uint16_t data);
    bool writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data);
    bool writeWords(uint8_t regAddr, uint8_t length, uint16_t *data);

    uint16_t readTimeout;

private:
    uint8_t _addr;
    TwoWire *_wire;
    bool _begun;
    size_t _maxBufferSize;
    bool _read(uint8_t *buffer, size_t len, bool stop);
};

#endif /* _I2CDEV_H_ */
