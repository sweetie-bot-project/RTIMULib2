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
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL3_C, LSM6DS33_CTRL3_C_IF_INC, "Failed to set LSM6DS33 automatic register address increment"))
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

    if(m_settings->m_LSM6DS33LIS3MDLFIFOMode)
      if(!setFIFO())
        return false;

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

    //std::cout << "Calculating gyro bias" << std::endl;
    gyroBiasInit();

    HAL_INFO("LSM6DS33LIS3MDL init complete\n");
    return true;
}

bool RTIMULSM6DS33LIS3MDL::setGyro()
{
    unsigned char ctrl2_g;

    //
    // gyro sample rate and scale factor
    //

    // sample rate
    switch (m_settings->m_LSM6DS33LIS3MDLGyroSampleRate) {
    case LSM6DS33_SAMPLERATE_0:
        ctrl2_g = 0x00;
        m_sampleRate = 0;
        break;

    case LSM6DS33_SAMPLERATE_13:
        ctrl2_g = 0x10;
        m_sampleRate = 13;
        break;

    case LSM6DS33_SAMPLERATE_26:
        ctrl2_g = 0x20;
        m_sampleRate = 26;
        break;

    case LSM6DS33_SAMPLERATE_52:
        ctrl2_g = 0x30;
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

    // gyro scale factor
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

    //std::cout << "Writing CTRL2_G " << static_cast<unsigned>(ctrl2_g) << std::endl;
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL2_G, ctrl2_g, "Failed to set LSM6DS33 CTRL2_G"))
        return false;

    // setup HPF configuration
    unsigned char hpgEn;
    unsigned char gyroHpf;
    if (m_settings->m_LSM6DS33LIS3MDLGyroHpf == -1) {
        // HPF filter is disabled
        hpgEn = 0;
        gyroHpf = 0;
    }
    else if ((m_settings->m_LSM6DS33LIS3MDLGyroHpf >= LSM6DS33_HPF_0) || (m_settings->m_LSM6DS33LIS3MDLGyroHpf <= LSM6DS33_HPF_3)) {
        // HPF filter is enabled
        hpgEn = 1;
        gyroHpf = m_settings->m_LSM6DS33LIS3MDLGyroHpf;
    }
    else {
        HAL_ERROR1("Illegal LSM6DS33 high pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroHpf);
        return false;
    }

    // G_HM_MODE, HP_G_EN, HPCF_G1, HPCF_G0, HP_G_RST, ROUNDING_STATUS, 0, 0
    unsigned char ctrl7_g = (hpgEn<<6) | (gyroHpf<<4);
 
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

    //
    // set sample rate, analog LPF and scale factor
    //

    // sample rate
    switch (m_settings->m_LSM6DS33LIS3MDLAccelSampleRate) {
    case LSM6DS33_ACCEL_SAMPLERATE_0:
        ctrl1_xl = 0x00;
        m_sampleRate = 0;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_13:
        ctrl1_xl = 0x10;
        m_sampleRate = 13;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_26:
        ctrl1_xl = 0x20;
        m_sampleRate = 26;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_52:
        ctrl1_xl = 0x30;
        m_sampleRate = 52;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_104:
        ctrl1_xl = 0x40;
        m_sampleRate = 104;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_208:
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

    // scale factor
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

    // analog LPF configuration
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

    //std::cout << "Writing LSM6DS33 CTRL1_XL: " << static_cast<unsigned>(ctrl1_xl) << std::endl;
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL1_XL, ctrl1_xl, "Failed to set LSM6DS33 CTRL1_XL"))
        return false;

    //
    // disable analog low-pass filter bandwidth scaling depending on sample rate settings.
    // bandwidth should be determined by LPF setting above
    //
 
    // XL_BW_SCAL_ODR, SLEEP_G, INT2_ON_INT1, FIFO_TEMP_EN, DRDY_MASK, I2C_DISABLE, 0, STOP_ON_FTH
    unsigned char ctrl4_c = 0x80;
    //std::cout << "Writing LSM6DS33 CTRL4_C: " << static_cast<unsigned>(ctrl4_c) << std::endl;
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL4_C, ctrl4_c, "Failed to set LSM6DS33 CTRL4_C"))
        return false;

    std::cout << "LSM6DS33 Accel set" << std::endl;
    return true;

}

bool RTIMULSM6DS33LIS3MDL::setFIFO()
{
  // To reset FIFO set it to bypass at first
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL5, LSM6DS33_FIFO_CTRL5_MODE_BYPASS, "Failed to set LSM6DS33 FIFO_CTRL5"))
      return false;

  // LSM6DS33 datasheet, page 46-54
  // TODO del
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL1_XL,  LSM6DS33_CTRL1_ODR_XL_104HZ | LSM6DS33_CTRL1_FS_XL_2G | LSM6DS33_CTRL1_BW_XL_400HZ, "Failed to set LSM6DS33 CTRL1_XL"))
      return false;

  // 104 Hz (high performance), 245 dps (degrees per second), full scale disabled
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL2_G,   LSM6DS33_CTRL2_ODR_G_104HZ | LSM6DS33_CTRL2_FS_G_125DPS, "Failed to set LSM6DS33 CTRL2_G"))
    return false;

  // When the FIFO is used, the IF_INC bit of the CTRL3_C register must be equal to 1.
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL3_C,   LSM6DS33_CTRL3_C_IF_INC, "Failed to set LSM6DS33 CTRL3_C"))
    return false;

  /* When FIFO is active and the DRDY_MASK bit is set to 1, accelerometer invalid samples
stored in FIFO are always equal to 7FFFh; gyroscope invalid samples stored in FIFO can be
equal to 7FFFh, 7FFEh or 7FFDh. In this way, a tag is applied to the invalid samples stored
in the FIFO buffer, so that they can be easily identified and discarded during data post-
processing.
  */
  // TODO del
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL4_C,   LSM6DS33_CTRL4_C_DRDY_MASK, "Failed to set LSM6DS33 CTRL4_C"))
    return false;
  // rounding
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL5_C,   LSM6DS33_CTRL5_C_BOTH_ROUNDING, "Failed to set LSM6DS33 CTRL5_C"))
    return false;
  // Angular rate sensor control register 6
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL6_C,   0x00, "Failed to set LSM6DS33 CTRL6_C")) // LSM6DS33_CTRL6_C_XL_HM_MODE);
    return false;
  // Angular rate sensor control register 7
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL7_G,   0x00, "Failed to set LSM6DS33 CTRL7_G")) // LSM6DS33_CTRL6_C_XL_HM_MODE);
    return false;
  // Linear acceleration sensor control register 8
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL8_XL,  0x00, "Failed to set LSM6DS33 CTRL8_XL"))
    return false;
  // Linear acceleration sensor control register 9
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL9_XL,  LSM6DS33_CTRL9_XL_ZEN_G | LSM6DS33_CTRL9_XL_YEN_G | LSM6DS33_CTRL9_XL_XEN_G, "Failed to set LSM6DS33 CTRL9_XL"))
    return false;
  // Control register 10
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL10_C,  LSM6DS33_CTRL10_C_ZEN_G | LSM6DS33_CTRL10_C_YEN_G | LSM6DS33_CTRL10_C_XEN_G, "Failed to set LSM6DS33 CTRL10_C"))
    return false;
  // FIFO control register 1
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL1, LSM6DS33_FIFO_CTRL1_FTH_OFF, "Failed to set LSM6DS33 FIFO_CTRL1")) // FIFO threshold level off
    return false;
  // FIFO control register 2
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL2, LSM6DS33_FIFO_CTRL2_FTH_OFF, "Failed to set LSM6DS33 FIFO_CTRL2")) // FIFO threshold level off
    return false;
  // No decimation (downsampling) for gyro and acc
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL3, LSM6DS33_FIFO_CTRL3_GYRO_NO_DECIMATION | LSM6DS33_FIFO_CTRL3_XL_NO_DECIMATION, "Failed to set LSM6DS33 FIFO_CTRL3"))
    return false;
  // Disable timer
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL4, LSM6DS33_FIFO_CTRL4_PEDO_NOT_IN_FIFO, "Failed to set LSM6DS33 FIFO_CTRL4"))
    return false;
  // FIFO ODR 833Hz, FIFO continous mode
  if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_CTRL5, LSM6DS33_FIFO_CTRL5_MODE_CONTINOUS | LSM6DS33_FIFO_CTRL5_ODR_FIFO_104HZ, "Failed to set LSM6DS33 FIFO_CTRL5"))
    return false;

  return true;
}

bool RTIMULSM6DS33LIS3MDL::setCompass()
{
    //
    // sample rate and performance mode for X and Z axes
    //
    unsigned char ctrl1 = 0x00;

    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    ctrl1 |= 0b01100000;

    // sample rate
    if ((m_settings->m_LSM6DS33LIS3MDLCompassSampleRate < 0) || (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate > 7)) {
        HAL_ERROR1("Illegal LIS3MDL compass sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLCompassSampleRate);
        return false;
    }
    ctrl1 |= (m_settings->m_LSM6DS33LIS3MDLCompassSampleRate << 2);

    if (!m_settings->HALWrite(m_compassSlaveAddr,  LIS3MDL_CTRL1, ctrl1, "Failed to set LIS3MDL CTRL1"))
        return false;

    //
    // sensor scale factor
    //
    unsigned char ctrl2 = 0;

    // scale factor
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
        return false;

    // MD = 00 (continuous-conversion mode)
    if (!m_settings->HALWrite(m_compassSlaveAddr, LIS3MDL_CTRL3, 0x00, "Failed to set LIS3MDL CTRL3"))
        return false;

    // OMZ = 11 (ultra-high-performance mode for Z)
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
    int interval =  1000 / m_sampleRate;
    if (interval < 1) interval = 1;
    return interval;
}

ssize_t RTIMULSM6DS33LIS3MDL::read_fifo_status()
{
  if(!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_STATUS1, 4, data, "Failed to read LSM6DS33 data"))
    return -1;
  fifo_empty   = data[1] & LSM6DS33_FIFO_EMPTY;
  fifo_full    = data[1] & LSM6DS33_FIFO_FULL;
  fifo_overrun = data[1] & LSM6DS33_FIFO_OVER_RUN;
  fifo_pattern = (data[3] & LSM6DS33_FIFO_STATUS4_FIFO_PATTERN)*256 + data[2];

  if(fifo_pattern>samples_in_chunk)
    return -2;

  if(fifo_pattern>0){
    unsigned int bad_samples_in_buffer = (samples_in_chunk-fifo_pattern);
    total_droped_chunks++;
    total_droped_samples += bad_samples_in_buffer;
    total_droped_bytes += bad_samples_in_buffer*2;
    first_valid_chunk_addr = (samples_in_chunk-fifo_pattern)*2;;
  }else
    first_valid_chunk_addr = 0;

  n_samples = (data[1] & LSM6DS33_FIFO_STATUS2_DIFF_MASK)*256 + data[0];
  n_chunks  = n_samples/6;
  n_bytes   = n_samples*2;

  return n_chunks;
}

bool RTIMULSM6DS33LIS3MDL::read_fifo(unsigned int chunks)
{
  if(chunks<1) return false;
  if(!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_FIFO_DATA_OUT_L, (chunks*12) , data, "Failed to read LSM6DS33 FIFO data"))
    return false;
  return true;
}

bool RTIMULSM6DS33LIS3MDL::convert_chunk(unsigned int chunk)
{
  if(chunk>n_chunks)
    return false;

  unsigned int gyro_addr = first_valid_chunk_addr + (chunk * 12);
  unsigned int accel_addr = gyro_addr+6;
  unsigned int sample;

  m_imuData.gyroValid = false;
  for(unsigned char i=0; i<3; i++)
  {
    sample = ((data[gyro_addr+1+(i*2)]*256)  + data[gyro_addr+(i*2)]);
    if( !m_imuData.gyroValid )
     m_imuData.gyroValid = ( ( (sample-1) ^ LSM6DS33_FIFO_G_INVALID_VALUE_MASK ) > 0b11); // not 7FFFh, 7FFEh or 7FFDh

    m_imuData.gyro.setData( i, sample*m_gyroScale );
  }

  m_imuData.accelValid = false;
  for(unsigned char i=0; i<3; i++){
    sample = ((data[accel_addr+1+(i*2)]*256) + data[accel_addr+(i*2)]);
    if( !m_imuData.accelValid )
      m_imuData.accelValid = ( sample != LSM6DS33_FIFO_XL_INVALID_VALUE ); // not 7FFFh

    m_imuData.accel.setData( i, sample*m_accelScale );
  }
  return m_imuData.gyroValid & m_imuData.accelValid;
}

bool RTIMULSM6DS33LIS3MDL::IMURead()
{
    if(m_settings->m_LSM6DS33LIS3MDLFIFOMode)
    {
      ssize_t chunks_in_fifo = read_fifo_status();
      switch(chunks_in_fifo)
      {
        case -1: printf("Read failed\n");
          return -1;
        case -2: printf("Wrong fifo pattern %d\n", fifo_pattern);
          return -2;
        case -3: printf("Readed too many chunks\n");
          return -3;
      }

      printf("n_chunks=%d ", n_chunks);
      if(n_chunks < 2)
        return true;

      bool ret = read_fifo((n_chunks-1));
      if(!ret)
        return false;

      m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

      unsigned int chunks_readed = (n_chunks-1);

      int count = 0;
      for(unsigned int chunk=0; chunk < chunks_readed; chunk++)
      {
        count++;

        ret = convert_chunk(chunk);
      }
      //  now do standard processing
      handleGyroBias();
      calibrateAverageCompass();
      calibrateAccel();

      //  now update the filter
      updateFusion();
    }
    else
    {
      //unsigned char status;
      unsigned char imuData[12];
      unsigned char compassData[6];

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
        /*
        if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_STATUS_REG, 1, &status, "Failed to read LSM6DS33 status"))
            return false;
        if ((status & 0x8) == 0)
        {
            std::cout << "STATUS ERROR -> RETURN" << std::endl;
            return false;
        }
        */

      // bulk read: gyro and accel data
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_OUTX_L_G, 12, imuData, "Failed to read LSM6DS33 data"))
            return false;

        /*
        std::cout << "Gyro Data: "
        << "  X " << static_cast<int16_t>(imuData[0] | imuData[1] << 8) * m_imuScale
        << "; Y " << static_cast<int16_t>(imuData[2] | imuData[3] << 8) * m_imuScale
        << "; Z " << static_cast<int16_t>(imuData[4] | imuData[5] << 8) * m_imuScale << std::endl;
        */

        /*
        std::cout << "Accel Data: "
        << "  X " << static_cast<int16_t>(imuData[6] | imuData[7] << 8)
        << "; Y " << static_cast<int16_t>(imuData[8] | imuData[9] << 8)
        << "; Z " << static_cast<int16_t>(imuData[10] | imuData[11] << 8) << std::endl;
        */

        if (!m_settings->HALRead(m_compassSlaveAddr, 0x80 | LIS3MDL_OUT_X_L, 6, compassData, "Failed to read LIS3MDL compass data"))
            return false;

        /*
        std::cout << "Compass Data: "
        << "  X " << static_cast<int16_t>(compassData[0] | compassData[1] << 8)
        << "; Y " << static_cast<int16_t>(compassData[2] | compassData[3] << 8)
        << "; Z " << static_cast<int16_t>(compassData[4] | compassData[5] << 8) << std::endl;
        */
        RTMath::convertToVector(imuData, m_imuData.gyro, m_gyroScale, false);
        RTMath::convertToVector(imuData + 6, m_imuData.accel, m_accelScale, false);
        RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);
        //  now do standard processing
        handleGyroBias();
        calibrateAverageCompass();
        calibrateAccel();

        //  now update the filter
        updateFusion();
    }
    return true;
}
