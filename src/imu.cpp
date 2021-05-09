
/**
 * @file    imu.cpp
 * @author  Jose Miguel Rios Rubio <jrios.github@gmail.com>
 * @date    09-05-2021
 * @version 1.0.0
 *
 * @section DESCRIPTION
 *
 * IMU ease control class.
 *
 * @section LICENSE
 *
 * Copyright (c) 2021 Jose Miguel Rios Rubio. All right reserved.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*****************************************************************************/

/* Libraries */

// Header Interface
#include "imu.h"

// Device/Framework Libraries
#if defined(ARDUINO)
    #include <Arduino.h>
    //#include <Wire.h>
#else
    #error "IMU Library currently just support Arduino Framework"
#endif

// Third-Party Libraries
//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/*****************************************************************************/

/* ISR */

volatile bool mpuInterrupt = false;

void ICACHE_RAM_ATTR isr_imu_int(void)
{
    mpuInterrupt = true;
}

/*****************************************************************************/

/* Constructor */

/**
  * @brief  Constructor. Initialize attributtes.
  */
IMU::IMU(const uint8_t i2c_addr)
{
    imu = new MPU6050(i2c_addr);
    gpio_int = IMU_DEFAULT_INTERRUPT_PIN;
    imu_int_status = 0;
    imu_dmp_packet_size = 0;
    num_bytes_in_buffer = 0;
    memset(imu_buffer, 0, IMU_FIFO_LENGTH);
}

/*****************************************************************************/

/* Public Methods */

/**
  * @brief  Initialize communication with the IMU.
  * @param  int_pin Interrupt GPIO Pin to detect IMU new data.
  * @return Initialization result (true or false).
  */
bool IMU::init(const uint8_t int_pin)
{
    gpio_int = int_pin;

    pinMode(int_pin, INPUT_PULLDOWN);

    if (this->init_communication() == false)
    {
        Serial.println("Failed to start communication with IMU");
        return false;
    }

    if (this->init_dmp() == false)
    {
        Serial.println("Error: IMU DMP Initialization fail");
        return false;
    }

    Serial.println("IMU initialized");

    return true;
}

/**
  * @brief  Check if there is any new measure available.
  * @return If any new measure is available to be read.
  */
bool IMU::available()
{
    MPU6050* _imu = (MPU6050*) imu;

    // Check for Interrupt
    if (mpuInterrupt == false)
        return false;
    mpuInterrupt = false;
    imu_int_status = _imu->getIntStatus();

    // Check for FIFO overflow
    if (imu_int_status & 0x10)
    {
        _imu->resetFIFO();
        Serial.println("IMU FIFO overflow");
        return false;
    }

    // Check for data available
    else if (imu_int_status & 0x02)
    {
        // Check if still not enough bytes in FIFO to read
        if (_imu->getFIFOCount() < imu_dmp_packet_size)
            return false;

        // Read a packet from FIFO
        _imu->getFIFOBytes(imu_buffer, imu_dmp_packet_size);

        return true;
    }

    return false;
}

/**
  * @brief  Get IMU last measure data.
  * @param  imu_data IMU data readed.
  */
void IMU::get_measure(imu_data_t* imu_data)
{
    MPU6050* _imu = (MPU6050*) imu;
    Quaternion q;
    VectorInt16 a;
    VectorInt16 a_no_gravity;
    VectorInt16 a_world;
    VectorFloat gravity;
    float rotation[3];

    _imu->dmpGetQuaternion(&q, imu_buffer);
    _imu->dmpGetAccel(&a, imu_buffer);
    _imu->dmpGetGravity(&gravity, &q);
    _imu->dmpGetLinearAccel(&a_no_gravity, &a, &gravity);
    _imu->dmpGetLinearAccelInWorld(&a_world, &a_no_gravity, &q);
    _imu->dmpGetYawPitchRoll(rotation, &q, &gravity);

    imu_data->quaternion.x = q.x;
    imu_data->quaternion.y = q.y;
    imu_data->quaternion.z = q.z;
    imu_data->quaternion.w = q.w;
    
    imu_data->acceleration.x = a_world.x;
    imu_data->acceleration.y = a_world.y;
    imu_data->acceleration.z = a_world.z;

    imu_data->gravity.x = gravity.x;
    imu_data->gravity.y = gravity.y;
    imu_data->gravity.z = gravity.z;

    imu_data->rotation.yaw = rotation[0];
    imu_data->rotation.pitch = rotation[1];
    imu_data->rotation.roll = rotation[2];
}

/*****************************************************************************/

/* Private Methods */

/**
  * @brief  Initialize communication with the IMU.
  * @return Initialization result (true or false).
  */
bool IMU::init_communication()
{
    MPU6050* _imu = (MPU6050*) imu;

    // Initialize I2C (commented to expect do it by user from outside)
    //#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //    Wire.begin();
    //    Wire.setClock(400000);
    //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //    Fastwire::setup(400, true);
    //#endif

    _imu->initialize();
    return _imu->testConnection();
}

/**
  * @brief  Initialize IMU DMP.
  * @return Initialization result (true or false).
  */
bool IMU::init_dmp()
{
    MPU6050* _imu = (MPU6050*) imu;

    // Initialize DMP
    if (_imu->dmpInitialize() != 0)
        return false;

    // Set calibration offset
    _imu->setXGyroOffset(220);
    _imu->setYGyroOffset(76);
    _imu->setZGyroOffset(-85);
    _imu->setZAccelOffset(1688);

    // Enable DMP and GPIO interrupt
    _imu->setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(gpio_int), isr_imu_int, RISING);
    imu_int_status = _imu->getIntStatus();
    num_bytes_in_buffer = _imu->getFIFOCount();
    imu_dmp_packet_size = _imu->dmpGetFIFOPacketSize();

    return true;
}
