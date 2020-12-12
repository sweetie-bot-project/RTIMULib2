////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#ifndef _RTIMULSM6DS33LIS3MDL_H
#define	_RTIMULSM6DS33LIS3MDL_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

class RTIMULSM6DS33LIS3MDL : public RTIMU
{
public:
    RTIMULSM6DS33LIS3MDL(RTIMUSettings *settings);
    ~RTIMULSM6DS33LIS3MDL();

    virtual const char *IMUName() { return "LSM6DS33 + LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM6DS33LIS3MDL; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual ssize_t read_fifo_status();
    bool read_fifo(unsigned int chunks);
    bool convert_chunk(unsigned int chunk);
    virtual bool IMURead();

private:
    bool setGyroSampleRate();
    bool setFIFO();
    bool setGyro();
    bool setAccel();
    bool setCompass();

    unsigned char m_gyroAccelSlaveAddr;                // I2C address of LSM6DS33
    unsigned char m_compassSlaveAddr;                  // I2C address of LIS3MDL

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;

    unsigned char data[8192] = {0};
    unsigned int bytes_readed = 0;

    const unsigned char chunk_size = 12;
    const unsigned char samples_in_chunk = 6;

    unsigned int total_droped_chunks = 0;
    unsigned int total_droped_samples = 0;
    unsigned int total_droped_bytes = 0;

    unsigned int first_valid_chunk_addr = 0;

    unsigned int n_samples = 0;
    unsigned int n_chunks = 0;
    unsigned int n_bytes = 0;

    unsigned int total_blocks_readed = 2;
    unsigned int total_chunks_readed = 10;
    unsigned int samples_to_read_next_round = 0;

    bool fifo_full    = false;
    bool fifo_overrun = false;
    bool fifo_empty   = false;
    unsigned int fifo_pattern = 0;

};

#endif // _RTIMULSM6DS33LIS3MDL_H
