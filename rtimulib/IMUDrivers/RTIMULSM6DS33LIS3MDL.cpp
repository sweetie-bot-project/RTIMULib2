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


#include "RTIMULSM6DS33LIS3MDL.h"
#include "RTIMUSettings.h"
#include <iostream>
#include <bitset>

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMULSM6DS33LIS3MDL::RTIMULSM6DS33LIS3MDL(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMULSM6DS33LIS3MDL::~RTIMULSM6DS33LIS3MDL()
{
}

bool RTIMULSM6DS33LIS3MDL::IMUInit()
{
    unsigned char result;

#ifdef LSM6DS33LIS3MDL_CACHE_MODE
    m_firstTime = true;
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif
    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_gyroAccelSlaveAddr = m_settings->m_I2CSlaveAddress;

    // work out gyro/accel address

    if (!m_settings->HALRead(LSM6DS33_ADDRESS0, LSM6DS33_WHO_AM_I, 1, &result, "Reading WHO AM I on Addr0")) 
    {
        if (!m_settings->HALRead(LSM6DS33_ADDRESS1, LSM6DS33_WHO_AM_I, 1, &result, "Reading WHO AM I on Addr1"))
        {
            return false;
        } 
        else 
        {
            if (result == LSM6DS33_ID) 
            {
                m_gyroAccelSlaveAddr = LSM6DS33_ADDRESS1;
                //std::cout << "Addr1 " << LSM6DS33_ADDRESS1 << std::endl;
            }
        }
    } 
    else 
    {
        if (result == LSM6DS33_ID) 
        {
            m_gyroAccelSlaveAddr = LSM6DS33_ADDRESS0;
            //std::cout << "Addr0 " << LSM6DS33_ADDRESS0 << std::endl;
        }
    }

    
    // work out compass address
    if (!m_settings->HALRead(LIS3MDL_ADDRESS0, LIS3MDL_WHO_AM_I, 1, &result, "Reading LIS3MDL WHO AM I on Addr0")) 
    {
        if (!m_settings->HALRead(LIS3MDL_ADDRESS1, LIS3MDL_WHO_AM_I, 1, &result, "Reading LIS3MDL WHO AM I on Addr1"))
        {
            return false;
        } 
        else 
        {
            if (result == LIS3MDL_ID) 
            {
                m_compassSlaveAddr = LIS3MDL_ADDRESS1;
                //std::cout << "Addr1 " << LIS3MDL_ADDRESS1 << std::endl;
            }
        }
    } 
    else 
    {
        if (result == LIS3MDL_ID) 
        {
            m_compassSlaveAddr = LIS3MDL_ADDRESS0;
            //std::cout << "Addr0 " << LIS3MDL_ADDRESS0 << std::endl;
        }
    }



    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  Set up the gyro/accel

    // IF_INC = 1 (automatically increment address register)
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL3_C, 0x04, "Failed to set LSM6DS33 automatic register address increment"))
    {
        std::cout << "Error setting LSM6DS33 gyroscope" << std::endl;
        return false;
    } 
    else 
    {
        std::cout << "Set LSM6DS33 automatic register address increment" << std::endl;
    } 

    if (!setGyro())
        return false;
    
    if (!setAccel())
    {
        std::cout << "Error setting LSM6DS33 accelerometer" << std::endl;
        return false;
    }
    else
    {
        std::cout << "LSM6DS33 accelerometer set" << std::endl;
    }

    //  Set up the compass
    //std::cout << "Trying to read from compassSlaveAddr: " << static_cast<unsigned>(m_compassSlaveAddr) << std::endl;
    if (!m_settings->HALRead(m_compassSlaveAddr, LIS3MDL_WHO_AM_I, 1, &result, "Failed to read LIS3MDL id"))
        return false;

    if (result != LIS3MDL_ID) {
        HAL_ERROR1("Incorrect LIS3MDL id %d\n", result);
        return false;
    }

    if (!setCompass())
    {
        std::cout << "Error setting LIS3MDL compass" << std::endl;
        return false;
    }
    else
    {
        std::cout << "LIS3MDL compass set" << std::endl;
    }

/*
#ifdef LSM6DS33LIS3MDL_CACHE_MODE

    //  turn on gyro fifo

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL, 0x3f, "Failed to set LSM6DS33 FIFO mode"))
        return false;
#endif
*/
    //std::cout << "Calculating gyro bias" << std::endl;
    gyroBiasInit();

    HAL_INFO("LSM6DS33LIS3MDL init complete\n");
    return true;
}

bool RTIMULSM6DS33LIS3MDL::setGyro()
{
    unsigned char ctrl2_g;
    unsigned char lowOdr = 0;     // high speed enabled lowOdr = 0
    unsigned char powerMode = 0;  // high speed enabled -> powerMode = 0, low power and normal mode -> powerMode = 1

    switch (m_settings->m_LSM6DS33LIS3MDLGyroSampleRate) {
    case LSM6DS33_SAMPLERATE_0:
        ctrl2_g = 0x00;
        powerMode = 1;
        m_sampleRate = 0;
        break;

    case LSM6DS33_SAMPLERATE_13:
        ctrl2_g = 0x10;
        powerMode = 1;
        m_sampleRate = 13;
        break;

    case LSM6DS33_SAMPLERATE_26:
        ctrl2_g = 0x20;
        powerMode = 1;
        m_sampleRate = 26;
        break;

    case LSM6DS33_SAMPLERATE_52:
        ctrl2_g = 0x30;
        powerMode = 1;
        m_sampleRate = 52;
        break;

    case LSM6DS33_SAMPLERATE_104:
        ctrl2_g = 0x40;
        m_sampleRate = 104;
        break;

    case LSM6DS33_SAMPLERATE_208:
        ctrl2_g = 0x50;
        m_sampleRate = 208;
        break;

    case LSM6DS33_SAMPLERATE_416:
        ctrl2_g = 0x60;
        m_sampleRate = 416;
        break;

    case LSM6DS33_SAMPLERATE_833:
        ctrl2_g = 0x70;
        m_sampleRate = 833;
        break;

    case LSM6DS33_SAMPLERATE_1660:
        ctrl2_g = 0x80;
        m_sampleRate = 1660;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;


    switch (m_settings->m_LSM6DS33LIS3MDLGyroFsr) {
    case LSM6DS33_FSR_125:
        ctrl2_g |= 0x02;
        m_gyroScale = (RTFLOAT)0.004375 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_245:
        ctrl2_g |= 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_500:
        ctrl2_g |= 0x04;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_1000:
        ctrl2_g |= 0x08;
        m_gyroScale = (RTFLOAT)0.035 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_2000:
        ctrl2_g |= 0x0c;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroFsr);
        return false;
    }

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL2_G, ctrl2_g, "Failed to set LSM6DS33 CTRL2_G"))
	return false;
    
    //std::cout << "Writing CTRL2_G " << static_cast<unsigned>(ctrl2_g) << std::endl;
    
    unsigned char regData;
    m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_CTRL2_G, 1, &regData, "Failed to read LSM6DS33 CTRL2_G");

    if ((m_settings->m_GD20HM303DGyroHpf < LSM6DS33_HPF_0) || (m_settings->m_LSM6DS33LIS3MDLGyroHpf > LSM6DS33_HPF_3))
    {
        HAL_ERROR1("Illegal LSM6DS33 high pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroHpf);
        return false;
    }

    unsigned char hpgEn = 1;

    unsigned char gyroHpf = m_settings->m_LSM6DS33LIS3MDLGyroHpf;

    // G_HM_MODE, HP_G_EN, HPCF_G1, HPCF_G0, HP_G_RST, ROUNDING_STATUS, 0, 0
    unsigned char ctrl7_g = (powerMode<<7) | (hpgEn<<6) | (gyroHpf<<4) | 0x00;    
 
    //std::cout << "Writing CTRL7_G " << static_cast<unsigned>(ctrl7_g) << std::endl;
    
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL7_G, ctrl7_g, "Failed to set LSM6DS33 CTRL7_G"))
        return false;


    std::cout << "LSM6DS33 Gyro set" << std::endl;
    return true;
}

/*
bool RTIMULSM6DS33LIS3MDL::setGyroCTRL5()
{
    unsigned char ctrl5;

    //  Turn on hpf

    ctrl5 = 0x10;

#ifdef GD20HM303D_CACHE_MODE
    //  turn on fifo

    ctrl5 |= 0x40;
#endif

    return m_settings->HALWrite(m_gyroAccelSlaveAddr,  LSM6DS33_CTRL5, ctrl5, "Failed to set LSM6DS33 CTRL5");
}
*/


bool RTIMULSM6DS33LIS3MDL::setAccel()
{
    unsigned char ctrl1_xl;
    unsigned char powerMode = 0;  // high speed enabled -> powerMode = 0, low power and normal mode -> powerMode = 1


    if ((m_settings->m_GD20HM303DAccelSampleRate < 0) || (m_settings->m_GD20HM303DAccelSampleRate > 10)) 
    {
        HAL_ERROR1("Illegal LSM303D accel sample rate code %d\n", m_settings->m_GD20HM303DAccelSampleRate);
        return false;
    }

    switch (m_settings->m_LSM6DS33LIS3MDLAccelSampleRate) {
    case LSM6DS33_ACCEL_SAMPLERATE_0:
        ctrl1_xl = 0x00;
        powerMode = 1;
        m_sampleRate = 0;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_13:
        ctrl1_xl = 0x10;
        powerMode = 1;
        m_sampleRate = 13;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_26:
        ctrl1_xl = 0x20;
        powerMode = 1;
        m_sampleRate = 26;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_52:
        ctrl1_xl = 0x30;
        powerMode = 1;
        m_sampleRate = 52;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_104:
        ctrl1_xl = 0x40;
        m_sampleRate = 104;
        break;

    case LSM6DS33_SAMPLERATE_208:
        ctrl1_xl = 0x50;
        m_sampleRate = 208;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_416:
        ctrl1_xl = 0x60;
        m_sampleRate = 416;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_833:
        ctrl1_xl = 0x70;
        m_sampleRate = 833;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_1660:
        ctrl1_xl = 0x80;
        m_sampleRate = 1660;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_3330:
        ctrl1_xl = 0x90;
        m_sampleRate = 3330;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_6660:
        ctrl1_xl = 0xa0;
        m_sampleRate = 6660;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;


    switch (m_settings->m_LSM6DS33LIS3MDLAccelFsr) {
    case LSM6DS33_ACCEL_FSR_2:
        ctrl1_xl |= (0x00<<2);
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM6DS33_ACCEL_FSR_16:
        ctrl1_xl |= (0x01<<2);
        m_accelScale = (RTFLOAT)0.000488;
        break;

    case LSM6DS33_ACCEL_FSR_4:
        ctrl1_xl |= (0x10<<2);
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM6DS33_ACCEL_FSR_8:
        ctrl1_xl |= (0x11<<2);
        m_accelScale = (RTFLOAT)0.000244;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelFsr);
        return false;
    }


    switch (m_settings->m_LSM6DS33LIS3MDLAccelLpf) {
    case LSM6DS33_ACCEL_LPF_400:
        ctrl1_xl |= 0x00;
        break;

    case LSM6DS33_ACCEL_LPF_200:
        ctrl1_xl |= 0x01;
        break;

    case LSM6DS33_ACCEL_LPF_100:
        ctrl1_xl |= 0x02;
        break;

    case LSM6DS33_ACCEL_LPF_50:
        ctrl1_xl |= 0x03;
        break;

    }

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL1_XL, ctrl1_xl, "Failed to set LSM6DS33 CTRL1_XL"))
	return false;

    //std::cout << "Writing LSM6DS33 CTRL1_XL: " << static_cast<unsigned>(ctrl1_xl) << std::endl;

    if ((m_settings->m_LSM6DS33LIS3MDLAccelHpf < LSM6DS33_ACCEL_HPF_0) || (m_settings->m_LSM6DS33LIS3MDLAccelHpf > LSM6DS33_ACCEL_HPF_3)) 
    {
        HAL_ERROR1("Illegal LSM6DS33 high pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelHpf);
        return false;
    }

    unsigned char LPF2_XL_EN = 0;
    unsigned char HP_SLOPE_XL_EN = 0;
    unsigned char LOW_PASS_ON_6D = 0;

    unsigned char accelHpf = m_settings->m_LSM6DS33LIS3MDLAccelHpf;

    // LPF2_XL_EN, HPCF_XL1, HPCF_XL0, 0, 0, HP_SLOPE_XL_EN, 0, LOW_PASS_ON_6D
    unsigned char ctrl8_xl = (LPF2_XL_EN<<7) | (accelHpf<<5) | (HP_SLOPE_XL_EN<<2) | (HP_SLOPE_XL_EN<<0) | 0x00;

    //std::cout << "Writing LSM6DS33 CTRL8_XL: " << static_cast<unsigned>(ctrl8_xl) << std::endl;

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL8_XL, ctrl8_xl, "Failed to set LSM6DS33 CTRL8_XL"))
        return false;

    std::cout << "LSM6DS33 Accel set" << std::endl;
    return true;

}

bool RTIMULSM6DS33LIS3MDL::setCompass() 
{

    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    //write_reg(LIS3MDL_CTRL_REG1, 0b01110000);
    //if (!m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL1, 0b01110000, "Failed to set LIS3MDL CTRL1"))
    //    return false;


    unsigned char ctrl1;

    unsigned char OM = 0b01100000;
    ctrl1 = OM;

    if ((m_settings->m_LSM6DS33LIS3MDLCompassSampleRate < 0) || (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate > 7)) {
        HAL_ERROR1("Illegal LIS3MDL compass sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassSampleRate);
        return false;
    }

    ctrl1 |= (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate << 2);

#ifdef LSM6DS33LIS3MDL_CACHE_MODE
    //  enable fifo

    //ctrl1 |= 0x40; incorrect register, check datasheet if requried
#endif

    if (!m_settings->HALWrite(m_compassSlaveAddr,  LIS3MDL_CTRL1, ctrl1, "Failed to set LIS3MDL CTRL1"))
    {
        return false;
    }


    // FS = 00 (+/- 4 gauss full scale)
    //write_reg(LIS3MDL_CTRL2, 0b00000000);
    //if (!m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL2, 0b00000000, "Failed to set LIS3MDL CTRL2"))
    //    return false;

    unsigned char ctrl2;

    //  convert FSR to uT

    switch (m_settings->m_LSM6DS33LIS3MDLCompassFsr) {
    case LIS3MDL_COMPASS_FSR_4:
        ctrl2 = 0x00;
        m_compassScale = (RTFLOAT)0.01462;
        break;

    case LIS3MDL_COMPASS_FSR_8:
        ctrl2 = 0x20;
        m_compassScale = (RTFLOAT)0.02923;
        break;

    case LIS3MDL_COMPASS_FSR_12:
        ctrl2 = 0x40;
        m_compassScale = (RTFLOAT)0.04384;
        break;

    case LIS3MDL_COMPASS_FSR_16:
        ctrl2 = 0x60;
        m_compassScale = (RTFLOAT)0.05845;
        break;

    default:
        HAL_ERROR1("Illegal LIS3MDL compass FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassFsr);
        return false;
    }

    if (!m_settings->HALWrite(m_compassSlaveAddr,  LIS3MDL_CTRL2, ctrl2, "Failed to set LIS3MDL CTRL2"))
    {
        return false;
    }

    // MD = 00 (continuous-conversion mode)
    //write_reg(LIS3MDL_CTRL3, 0b00000000);
    if (!m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL3, 0x00, "Failed to set LIS3MDL CTRL3"))
        return false;

    // OMZ = 11 (ultra-high-performance mode for Z)
    //write_reg(LIS3MDL_CTRL4, 0b00001100);
    //if (!m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL4, 0b00001100, "Failed to set LIS3MDL CTRL4"))
    if (!m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL4, 0b00001100, "Failed to set LIS3MDL CTRL4"))
        return false;
    
    std::cout << "LIS3MDL Compass set" << std::endl; 
    return true;
}

/*
bool RTIMULSM6DS33LIS3MDL::setCompassCTRL5()
{
    unsigned char ctrl5;

    if ((m_settings->m_LSM6DS33LIS3MDLCompassSampleRate < 0) || (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate > 5)) {
        HAL_ERROR1("Illegal LIS3MDL compass sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassSampleRate);
        return false;
    }

    ctrl5 = (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate << 2);

#ifdef LSM6DS33LIS3MDL_CACHE_MODE
    //  enable fifo

    ctrl5 |= 0x40;
#endif

    return m_settings->HALWrite(m_compassSlaveAddr,  LIS3MDL_CTRL5, ctrl5, "Failed to set LIS3MDL CTRL5");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL6()
{
    unsigned char ctrl6;

    //  convert FSR to uT

    switch (m_settings->m_LSM6DS33LIS3MDLCompassFsr) {
    case LIS3MDL_COMPASS_FSR_2:
        ctrl6 = 0;
        m_compassScale = (RTFLOAT)0.008;
        break;

    case LIS3MDL_COMPASS_FSR_4:
        ctrl6 = 0x20;
        m_compassScale = (RTFLOAT)0.016;
        break;

    case LIS3MDL_COMPASS_FSR_8:
        ctrl6 = 0x40;
        m_compassScale = (RTFLOAT)0.032;
        break;

    case LIS3MDL_COMPASS_FSR_12:
        ctrl6 = 0x60;
        m_compassScale = (RTFLOAT)0.0479;
        break;

    default:
        HAL_ERROR1("Illegal LIS3MDL compass FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassFsr);
        return false;
    }

    return m_settings->HALWrite(m_compassSlaveAddr,  LIS3MDL_CTRL6, ctrl6, "Failed to set LIS3MDL CTRL6");
}

bool RTIMULSM6DS33LIS3MDL::setCompassCTRL7()
{
     return m_settings->HALWrite(m_compassSlaveAddr,  LIS3MDL_CTRL7, 0x60, "Failed to set LIS3MDL CTRL7");
}
*/

int RTIMULSM6DS33LIS3MDL::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMULSM6DS33LIS3MDL::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];


#ifdef LSM6DS33LIS3MDL_CACHE_MODE
/*
    int count;

    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_SRC, 1, &status, "Failed to read LSM6DS33 fifo status"))
        return false;

    if ((status & 0x40) != 0) {
        HAL_INFO("LSM6DS33 fifo overrun\n");
        if (!m_settings->HALWrite(m_gyroSlaveAddr, LSM6DS33_CTRL5, 0x10, "Failed to set LSM6DS33 CTRL5"))
            return false;

        if (!m_settings->HALWrite(m_gyroSlaveAddr, LSM6DS33_FIFO_CTRL, 0x0, "Failed to set LSM6DS33 FIFO mode"))
            return false;

        if (!m_settings->HALWrite(m_gyroSlaveAddr, LSM6DS33_FIFO_CTRL, 0x3f, "Failed to set LSM6DS33 FIFO mode"))
            return false;

        if (!setGyroCTRL5())
            return false;

        m_imuData.timestamp += m_sampleInterval * 32;
        return false;
    }

    // get count of samples in fifo
    count = status & 0x1f;

    if ((m_cacheCount == 0) && (count > 0) && (count < LSM6DS33LIS3MDL_FIFO_THRESH)) {
        // special case of a small fifo and nothing cached - just handle as simple read

        if (!m_settings->HALRead(m_gyroAccelSlaveAddr, 0x80 | LSM6DS33_OUT_X_L, 6, gyroData, "Failed to read LSM6DS33 data"))
            return false;

        if (!m_settings->HALRead(m_gyroAccelSlaveAddr, 0x80 | LSM6DS33_OUT_X_L_A, 6, accelData, "Failed to read LSM6DS33 accel data"))
            return false;

        if (!m_settings->HALRead(m_compassSlaveAddr, 0x80 | LIS3MDL_OUT_X_L_M, 6, compassData, "Failed to read LIS3MDL compass data"))
            return false;

        if (m_firstTime)
            m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        else
            m_imuData.timestamp += m_sampleInterval;

        m_firstTime = false;
    } else {
        if (count >=  LSM6DS33LIS3MDL_FIFO_THRESH) {
            // need to create a cache block

            if (m_cacheCount == LSM6DS33LIS3MDL_CACHE_BLOCK_COUNT) {
                // all cache blocks are full - discard oldest and update timestamp to account for lost samples
                m_imuData.timestamp += m_sampleInterval * m_cache[m_cacheOut].count;
                if (++m_cacheOut == LSM6DS33LIS3MDL_CACHE_BLOCK_COUNT)
                    m_cacheOut = 0;
                m_cacheCount--;
            }

            if (!m_settings->HALRead(m_gyroAccelSlaveAddr, 0x80 | LSM6DS33_OUT_X_L, LSM6DS33LIS3MDL_FIFO_CHUNK_SIZE * LSM6DS33LIS3MDL_FIFO_THRESH,
                         m_cache[m_cacheIn].data, "Failed to read LSM6DS33 fifo data"))
                return false;

            if (!m_settings->HALRead(m_gyroAccelSlaveAddr, 0x80 | LSM6DS33_OUT_X_L_A, 6,
                         m_cache[m_cacheIn].accel, "Failed to read LSM6DS33 accel data"))
                return false;

            if (!m_settings->HALRead(m_compassSlaveAddr, 0x80 | LIS3MDL_OUT_X_L_M, 6,
                         m_cache[m_cacheIn].compass, "Failed to read LIS3MDL compass data"))
                return false;

            m_cache[m_cacheIn].count = LSM6DS33LIS3MDL_FIFO_THRESH;
            m_cache[m_cacheIn].index = 0;

            m_cacheCount++;
            if (++m_cacheIn == LSM6DS33LIS3MDL_CACHE_BLOCK_COUNT)
                m_cacheIn = 0;

        }

        //  now fifo has been read if necessary, get something to process

        if (m_cacheCount == 0)
            return false;

        memcpy(gyroData, m_cache[m_cacheOut].data + m_cache[m_cacheOut].index, LSM6DS33LIS3MDL_FIFO_CHUNK_SIZE);
        memcpy(accelData, m_cache[m_cacheOut].accel, 6);
        memcpy(compassData, m_cache[m_cacheOut].compass, 6);

        m_cache[m_cacheOut].index += LSM6DS33LIS3MDL_FIFO_CHUNK_SIZE;

        if (--m_cache[m_cacheOut].count == 0) {
            //  this cache block is now empty

            if (++m_cacheOut == GD20HM303D_CACHE_BLOCK_COUNT)
                m_cacheOut = 0;
            m_cacheCount--;
        }
        if (m_firstTime)
            m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        else
            m_imuData.timestamp += m_sampleInterval;

        m_firstTime = false;
    }
*/
#else

    // STATUS_REG
    // -, -, -, -, EV_BOOT, TDA, GDA, XLDA
    // EV_BOOT: Boot running flag signal. Default value: 0
    //          0: no boot running; 1: boot running
    // TDA:     Temperature new data available. Default value: 0
    //          0: no set of data is available at temperature sensor output;
    //          1: a new set of data is available at temperature sensor output
    // GDA:     Gyroscope new data available. Default value: 0
    //          0: no set of data available at gyroscope output
    //          1: a new set of data is available at gyroscope sensor output
    // XLDA:    Accelerometer new data available. Default value: 0
    //          0: no set of data available at accelerometer output
    //          1: a new set of data is available at accelerometer output
    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_STATUS_REG, 1, &status, "Failed to read LSM6DS33 status"))
        return false;
    /*
    if ((status & 0x8) == 0)
    {
        std::cout << "STATUS ERROR -> RETURN" << std::endl;
        return false;
    }
    */

    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_OUTX_L_G, 6, gyroData, "Failed to read LSM6DS33 data"))
        return false;

    /*
    std::cout << "Gyro Data: "
    << "  X " << static_cast<int16_t>(gyroData[0] | gyroData[1] << 8) * m_gyroScale 
    << "; Y " << static_cast<int16_t>(gyroData[2] | gyroData[3] << 8) * m_gyroScale
    << "; Z " << static_cast<int16_t>(gyroData[4] | gyroData[5] << 8) * m_gyroScale << std::endl;
    */

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_OUTX_L_XL, 6, accelData, "Failed to read LSM6DS33 accel data"))
        return false;
    
    /*
    std::cout << "Accel Data: "
    << "  X " << static_cast<int16_t>(accelData[0] | accelData[1] << 8) 
    << "; Y " << static_cast<int16_t>(accelData[2] | accelData[3] << 8)
    << "; Z " << static_cast<int16_t>(accelData[4] | accelData[5] << 8) << std::endl;
    */

    if (!m_settings->HALRead(m_compassSlaveAddr, 0x80 | LIS3MDL_OUT_X_L, 6, compassData, "Failed to read LIS3MDL compass data"))
        return false;
 
    /* 
    std::cout << "Compass Data: "
    << "  X " << static_cast<int16_t>(compassData[0] | compassData[1] << 8) 
    << "; Y " << static_cast<int16_t>(compassData[2] | compassData[3] << 8)
    << "; Z " << static_cast<int16_t>(compassData[4] | compassData[5] << 8) << std::endl;
    */


#endif

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    //  sort out gyro axes
    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(m_imuData.gyro.y());
    m_imuData.gyro.setZ(m_imuData.gyro.z());

    //  sort out accel data;
    m_imuData.accel.setX(m_imuData.accel.x());
    m_imuData.accel.setY(-m_imuData.accel.y());
    m_imuData.accel.setZ(-m_imuData.accel.z());

    //  sort out compass axes
    m_imuData.compass.setX(m_imuData.compass.x());
    m_imuData.compass.setY(-m_imuData.compass.y());
    m_imuData.compass.setZ(-m_imuData.compass.z());

    //  now do standard processing
    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter
    updateFusion();

    return true;
}
