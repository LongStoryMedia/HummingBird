// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 2013-06-05 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
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

#include "I2Cdev.h"

/** Default constructor.
 */
I2Cdev::I2Cdev(uint8_t addr, TwoWire *wire)
{
    _addr = addr;
    _wire = wire;
    _begun = false;
    _maxBufferSize = 32;
}

bool I2Cdev::begin(bool addr_detect)
{
    // _wire->begin();
    _begun = true;

    if (addr_detect)
    {
        return detected();
    }
    return true;
}

void I2Cdev::end(void)
{
    _wire->end();
    _begun = false;
}

bool I2Cdev::detected(void)
{
    // Init I2C if not done yet
    if (!_begun && !begin(true))
    {
        return false;
    }

    // A basic scanner, see if it ACK's
    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() == 0)
    {
        return true;
    }

    return false;
}
/*!
 *    @brief  Returns the 7-bit _addr of this device
 *    @return The 7-bit _addr of this device
 */
uint8_t I2Cdev::address(void) { return _addr; }

/** Read a single bit from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout)
{
    uint8_t b;
    bool success = readByte(regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return success;
}

/** Read a single bit from a 16-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBitW(uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout)
{
    uint16_t b;
    uint8_t success = readWord(regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return success;
}

/** Read multiple bits from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t b;
    bool success;
    if ((success = readByte(regAddr, &b, timeout)) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return success;
}

/** Read multiple bits from a 16-bit device register.
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
bool I2Cdev::readBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout)
{
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    bool success;
    uint16_t w;
    if ((success = readWord(regAddr, &w, timeout)) != 0)
    {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return success;
}

bool I2Cdev::getByte(byte regAddr)
{
    // This function reads one byte over I2C
    _wire->beginTransmission(_addr);
    _wire->write(regAddr);         // Address of CTRL_REG1
    _wire->endTransmission(false); // Send data to I2C dev with option for a repeated start. Works in Arduino V1.0.1
    _wire->requestFrom(_addr, 1);
    return _wire->read();
}

/** Read single byte from an 8-bit device register.
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readByte(uint8_t regAddr, uint8_t *data, uint16_t timeout)
{
    return readBytes(regAddr, 1, data, timeout);
}

/** Read single word from a 16-bit device register.
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
bool I2Cdev::readWord(uint8_t regAddr, uint16_t *data, uint16_t timeout)
{
    return readWords(regAddr, 1, data, timeout);
}

/** Read multiple bytes from an 8-bit device register.
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
bool I2Cdev::readBytes(uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout)
{

    int8_t count = 0;
    uint32_t t1 = millis();

    // Arduino v1.0.1+, Wire library
    // Adds official support for repeated start condition, yay!

    // I2C/TWI subsystem uses internal buffer that breaks with large data requests
    // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
    // smaller chunks instead of all at once
    for (uint8_t k = 0; k < length; k += min((int)length, BUFFER_LENGTH))
    {
        _wire->beginTransmission(_addr);
        _wire->write(regAddr);
        _wire->endTransmission(false);
        _wire->beginTransmission(_addr);
        _wire->requestFrom(_addr, (uint8_t)min(length - k, BUFFER_LENGTH));

        while (_wire->available() && (timeout == 0 || millis() - t1 < timeout))
        {
            data[count++] = _wire->read();
        }
    }

    // check for timeout
    if (timeout > 0 && millis() - t1 >= timeout && count < length)
    {
        return false;
    }

    return count > 0;
}

/** Read multiple words from a 16-bit device register.
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
bool I2Cdev::readWords(uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout)
{

    int8_t count = 0;
    uint32_t t1 = millis();

    // Arduino v1.0.1+, Wire library
    // Adds official support for repeated start condition, yay!

    // I2C/TWI subsystem uses internal buffer that breaks with large data requests
    // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
    // smaller chunks instead of all at once
    for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH))
    {
        _wire->beginTransmission(_addr);
        _wire->write(regAddr);
        _wire->endTransmission();
        _wire->beginTransmission(_addr);
        _wire->requestFrom(_addr, (uint8_t)(length * 2)); // length=words, this wants bytes

        bool msb = true; // starts with MSB, then LSB
        while (_wire->available() && (timeout == 0 || millis() - t1 < timeout))
        {
            if (msb)
            {
                // first byte is bits 15-8 (MSb=15)
                data[count++] = _wire->read() << 8;
            }
            else
            {
                // second byte is bits 7-0 (LSb=0)
                data[count++] |= _wire->read();
            }
            msb = !msb;
        }

        _wire->endTransmission();
    }
    if (timeout > 0 && millis() - t1 >= timeout && count < length)
    {
        return false;
    }
    return count > 0;
}

/*!
 *    @brief  Read from I2C into a buffer from the I2C device.
 *    Cannot be more than maxBufferSize() bytes.
 *    @param  buffer Pointer to buffer of data to read into
 *    @param  len Number of bytes from buffer to read.
 *    @param  stop Whether to send an I2C STOP signal on read
 *    @return True if read was successful, otherwise false.
 */
bool I2Cdev::read(uint8_t *buffer, size_t len, bool stop)
{
    size_t pos = 0;
    while (pos < len)
    {
        size_t read_len =
            ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
        bool read_stop = (pos < (len - read_len)) ? false : stop;
        if (!_read(buffer + pos, read_len, read_stop))
            return false;
        pos += read_len;
    }
    return true;
}

bool I2Cdev::_read(uint8_t *buffer, size_t len, bool stop)
{
    size_t recv = _wire->requestFrom((uint8_t)_addr, (uint8_t)len, (uint8_t)stop);

    if (recv != len)
    {
        // Not enough data available to fulfill our obligation!
        Serial.print(F("\tI2CDevice did not receive enough data: "));
    }

    for (uint16_t i = 0; i < len; i++)
    {
        buffer[i] = _wire->read();
    }

    return true;
}

/*!
 *    @brief  Write 1 byte of data, then read some data from I2C into another buffer.
 *    Cannot be more than `BUFFER_LENGTH` bytes. The buffers can point to
 *    same/overlapping locations.
 *    @param  writeBuffer Buffer of data to write from
 *    @param  readBuffer Pointer to buffer of data to read into.
 *    @param  len Number of bytes from buffer to read.
 *    @return True if write & read was successful, otherwise false.
 */
bool I2Cdev::write_then_read(const uint8_t *write_buffer, size_t write_len, uint8_t *readBuffer, size_t read_len, bool stop = false)
{
    if (!write(write_buffer, write_len, stop))
    {
        return false;
    }

    return read(readBuffer, read_len);
}

/*!
 *    @brief  Write a buffer or two to the I2C device. Cannot be more than
 *            BUFFER_LENGTH bytes.
 *    @param  buffer Pointer to buffer of data to write. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  len Number of bytes from buffer to write
 *    @param  prefixBuffer Pointer to optional array of data to write before
 *            buffer. Cannot be more than BUFFER_LENGTH bytes. This is const to
 *            ensure the content of this buffer doesn't change.
 *    @param  prefixLen Number of bytes from prefix buffer to write
 *    @param  stop Whether to send an I2C STOP signal on write
 *    @return True if write was successful, otherwise false.
 */
bool I2Cdev::write(const uint8_t *buffer, size_t len, bool stop,
                   const uint8_t *prefixBuffer = (const uint8_t *)__null, size_t prefixLen = 0U)
{
    if ((len + prefixLen) > maxBufferSize())
    {
        Serial.println("Buffer too large");
        return false;
    }

    _wire->beginTransmission(_addr);

    // Write the prefix data (usually an _addr)
    if ((prefixLen != 0) && (prefixBuffer != NULL))
    {
        if (_wire->write(prefixBuffer, prefixLen) != prefixLen)
        {
            Serial.println(F("\tI2CDevice failed to write"));
            return false;
        }
    }

    // Write the data itself
    if (_wire->write(buffer, len) != len)
    {
        Serial.println(F("\tI2CDevice failed to write"));
        return false;
    }

    if (_wire->endTransmission(stop) == 0)
    {
        return true;
    }
    else
    {
        Serial.println("\tFailed to send!");
        return false;
    }
}

/** write a single bit in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    readByte(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t regAddr, uint8_t bitNum, uint16_t data)
{
    uint16_t w;
    readWord(regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(regAddr, &b) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        b &= ~(mask);                     // zero all important bits in existing byte
        b |= data;                        // combine data with existing byte
        return writeByte(regAddr, b);
    }
    else
    {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(regAddr, &w) != 0)
    {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        w &= ~(mask);                     // zero all important bits in existing word
        w |= data;                        // combine data with existing word
        return writeWord(regAddr, w);
    }
    else
    {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param regAddr Register _addr to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t regAddr, uint8_t data)
{
    uint8_t status = 0;
    _wire->beginTransmission(_addr);
    _wire->write((uint8_t)regAddr);
    _wire->write((uint8_t)data);
    status = _wire->endTransmission();
    return status == 0;
}

/** Write single word to a 16-bit device register.
 * @param regAddr Register _addr to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t regAddr, uint16_t data)
{
    return writeWords(regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param regAddr First register _addr to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data)
{
    uint8_t status = 0;
    _wire->beginTransmission(_addr);
    _wire->write((uint8_t)regAddr); // send _addr
    for (uint8_t i = 0; i < length; i++)
    {
        _wire->write((uint8_t)data[i]);
    }
    status = _wire->endTransmission();
    return status == 0;
}

/** Write multiple words to a 16-bit device register.
 * @param regAddr First register _addr to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t regAddr, uint8_t length, uint16_t *data)
{
    uint8_t status = 0;
    _wire->beginTransmission(_addr);
    _wire->write(regAddr);
    for (uint8_t i = 0; i < length; i++)
    {
        _wire->write((uint8_t)(data[i] >> 8)); // send MSB
        _wire->write((uint8_t)data[i]);        // send LSB
    }
    status = _wire->endTransmission();
    return status == 0;
}
