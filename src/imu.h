
/**
 * @file    rgbled.h
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

/* Include Guard */

#ifndef IMU_H_
#define IMU_H_

/*****************************************************************************/

/* Libraries */

// Standard C/C++ libraries
#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************/

/* Constants */

const uint8_t IMU_DEFAULT_I2C_ADDRESS = 0x68;
const uint8_t IMU_DEFAULT_INTERRUPT_PIN = 2;
const uint8_t IMU_FIFO_LENGTH = 64;

/*****************************************************************************/

/* Data Types */

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} u16_axis_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} float_axis_t;

typedef struct
{
    float x;
    float y;
    float z;
    float w;
} quaternion_t;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} rotations_t;

typedef struct
{
    quaternion_t quaternion;
    u16_axis_t acceleration;
    rotations_t rotation;
    float_axis_t gravity;
} imu_data_t;

/*****************************************************************************/

/* Class Interface */

class IMU
{
    public:
        IMU(const uint8_t i2c_addr=IMU_DEFAULT_I2C_ADDRESS);

        bool init(const uint8_t int_pin=IMU_DEFAULT_INTERRUPT_PIN);
        bool available();
        void get_measure(imu_data_t* imu_data);

    private:
        void* imu;
        uint8_t gpio_int;
        uint8_t imu_int_status;
        uint16_t imu_dmp_packet_size;
        uint16_t num_bytes_in_buffer;
        uint8_t imu_buffer[IMU_FIFO_LENGTH];

        bool init_communication();
        bool init_dmp();
};

/*****************************************************************************/

#endif /* IMU_H_ */
