#include "BMP390.h"

#define ALTMODE ; // comment out for barometer mode; default is altitude mode

BMP390::BMP390()
{
}

float BMP390::readRawBaro()
{
    uint8_t buffer[6] = {0, 0, 0, 0, 0, 0};
    i2c->readBytes(BMP390_DATA_REG_0, 6, buffer);
    // Temporary variables to store the sensor data
    // Store the parsed register values for pressure data
    uint32_t dataXlsb = (uint32_t)buffer[0];
    uint32_t dataLsb = (uint32_t)buffer[1] << 8;
    uint32_t dataMsb = (uint32_t)buffer[2] << 16;
    return dataMsb | dataLsb | dataXlsb;
}

float BMP390::readBaro()
{
    uint32_t pressure = readRawBaro();

    double comp_press;
    double partial_data1;
    double partial_data2;
    double partial_data3;
    double partial_data4;
    double partial_out1;
    double partial_out2;

    partial_data1 = quantizedCalibrationData.p6 * quantizedCalibrationData.lin;
    partial_data2 = quantizedCalibrationData.p7 * pow(quantizedCalibrationData.lin, 2);
    partial_data3 = quantizedCalibrationData.p8 * pow(quantizedCalibrationData.lin, 3);
    partial_out1 = quantizedCalibrationData.p5 + partial_data1 + partial_data2 + partial_data3;
    partial_data1 = quantizedCalibrationData.p2 * quantizedCalibrationData.lin;
    partial_data2 = quantizedCalibrationData.p3 * pow(quantizedCalibrationData.lin, 2);
    partial_data3 = quantizedCalibrationData.p4 * pow(quantizedCalibrationData.lin, 3);
    partial_out2 = pressure *
                   (quantizedCalibrationData.p1 + partial_data1 + partial_data2 + partial_data3);
    partial_data1 = pow((double)pressure, 2);
    partial_data2 = quantizedCalibrationData.p9 + quantizedCalibrationData.p10 * quantizedCalibrationData.lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + pow((double)pressure, 3) * quantizedCalibrationData.p11;

    return partial_out1 + partial_out2 + partial_data4;
}

float BMP390::read()
{
    alt = 44330. * (1. - pow(readBaro() / seaLevelPressure, 0.1902949));
    return alt;
}

void BMP390::calibrate(const uint8_t *buffer)
{
    double temp_var;

    /* 1 / 2^8 */
    temp_var = 0.00390625f;
    registerCalibrationData.t1 = CONCAT_BYTES(buffer[1], buffer[0]);
    quantizedCalibrationData.t1 = ((double)registerCalibrationData.t1 / temp_var);
    registerCalibrationData.t2 = CONCAT_BYTES(buffer[3], buffer[2]);
    temp_var = 1073741824.0f;
    quantizedCalibrationData.t2 = ((double)registerCalibrationData.t2 / temp_var);
    registerCalibrationData.t3 = (int8_t)buffer[4];
    temp_var = 281474976710656.0f;
    quantizedCalibrationData.t3 = ((double)registerCalibrationData.t3 / temp_var);
    registerCalibrationData.p1 = (int16_t)CONCAT_BYTES(buffer[6], buffer[5]);
    temp_var = 1048576.0f;
    quantizedCalibrationData.p1 = ((double)(registerCalibrationData.p1 - (16384)) / temp_var);
    registerCalibrationData.p2 = (int16_t)CONCAT_BYTES(buffer[8], buffer[7]);
    temp_var = 536870912.0f;
    quantizedCalibrationData.p2 = ((double)(registerCalibrationData.p2 - (16384)) / temp_var);
    registerCalibrationData.p3 = (int8_t)buffer[9];
    temp_var = 4294967296.0f;
    quantizedCalibrationData.p3 = ((double)registerCalibrationData.p3 / temp_var);
    registerCalibrationData.p4 = (int8_t)buffer[10];
    temp_var = 137438953472.0f;
    quantizedCalibrationData.p4 = ((double)registerCalibrationData.p4 / temp_var);
    registerCalibrationData.p5 = CONCAT_BYTES(buffer[12], buffer[11]);

    /* 1 / 2^3 */
    temp_var = 0.125f;
    quantizedCalibrationData.p5 = ((double)registerCalibrationData.p5 / temp_var);
    registerCalibrationData.p6 = CONCAT_BYTES(buffer[14], buffer[13]);
    temp_var = 64.0f;
    quantizedCalibrationData.p6 = ((double)registerCalibrationData.p6 / temp_var);
    registerCalibrationData.p7 = (int8_t)buffer[15];
    temp_var = 256.0f;
    quantizedCalibrationData.p7 = ((double)registerCalibrationData.p7 / temp_var);
    registerCalibrationData.p8 = (int8_t)buffer[16];
    temp_var = 32768.0f;
    quantizedCalibrationData.p8 = ((double)registerCalibrationData.p8 / temp_var);
    registerCalibrationData.p9 = (int16_t)CONCAT_BYTES(buffer[18], buffer[17]);
    temp_var = 281474976710656.0f;
    quantizedCalibrationData.p9 = ((double)registerCalibrationData.p9 / temp_var);
    registerCalibrationData.p10 = (int8_t)buffer[19];
    temp_var = 281474976710656.0f;
    quantizedCalibrationData.p10 = ((double)registerCalibrationData.p10 / temp_var);
    registerCalibrationData.p11 = (int8_t)buffer[20];
    temp_var = 36893488147419103232.0f;
    quantizedCalibrationData.p11 = ((double)registerCalibrationData.p11 / temp_var);
}

void BMP390::init(int basis, TwoWire *wire = &Wire)
{
    i2c = new I2Cdev(BMP390_ADDRESS, wire);

    // configure interface
    i2c->writeByte(0x1A, 0b00000010);
    delay(10);

    uint8_t id = 0;
    uint8_t readyStatus;
    i2c->readByte(BMP390_STATUS_ADDRESS, &readyStatus);
    if (readyStatus & BMP390_CMD_READY)
    {
        // soft reset
        i2c->writeByte(BMP390_CMD_ADDRESS, BMP390_RESET_CMD);
        delay(2);

        // get data from sensor register address
        i2c->readByte(BMP390_CHIP_ID_ADDRESS, &id);
        Serial.print("chip id is");
        Serial.println(id);
        if (id == BMP390_CHIP_ID)
        {
            // Read the calibration data
            uint8_t buffer[21] = {0};
            i2c->readBytes(BMP3_CALIBRATION_DATA_ADDRESS, 21, buffer);
            calibrate(buffer);
            delay(10);

            float currentPressure = readBaro();
            Serial.print("Current pressure: ");
            Serial.println(currentPressure);
            seaLevelPressure = currentPressure / pow(1 - basis * 0.0000225577, 5.255877);
            Serial.print("Sea level pressure: ");
            Serial.println(seaLevelPressure);
        }
    }
}

int8_t BMP390::calCrc(uint8_t seed, uint8_t data)
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