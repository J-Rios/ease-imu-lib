
/* Libraries */

// Framework Libraries
#include <Arduino.h>
#include <Wire.h>

#include "imu.h"

/*****************************************************************************/

/* Constants */

#define IMU_I2C_ADDRESS 0x68
#define GPIO_INT_IMU 18

/*****************************************************************************/

/* Functions Prototypes */

// System Functions
static void system_reboot(void);


/*****************************************************************************/

/* Goobal Elements */

IMU Imu(IMU_I2C_ADDRESS);

/*****************************************************************************/

/* Setup & Loop Functions */

void setup(void)
{
    Serial.begin(115200);
    Serial.println("System Started");
    delay(3000);

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);

    // Initialize and configure IMU
    if (Imu.init(GPIO_INT_IMU) == false)
        system_reboot();

    Serial.println("");
}

void loop()
{
    imu_data_t imu_data;

    if(Imu.available() == false)
    {
        delay(10);
        return;
    }

    Imu.get_measure(&imu_data);

    Serial.print("(AX, AY, AZ, Yaw, Pitch, Roll, Gx, Gy, Gz): (");
    Serial.print(imu_data.acceleration.x);
    Serial.print(", ");
    Serial.print(imu_data.acceleration.y);
    Serial.print(", ");
    Serial.print(imu_data.acceleration.z);
    Serial.print(", ");
    Serial.print(imu_data.rotation.yaw);
    Serial.print(", ");
    Serial.print(imu_data.rotation.pitch);
    Serial.print(", ");
    Serial.print(imu_data.rotation.roll);
    Serial.print(", ");
    Serial.print(imu_data.gravity.x);
    Serial.print(", ");
    Serial.print(imu_data.gravity.y);
    Serial.print(", ");
    Serial.print(imu_data.gravity.z);
    Serial.println(")");

    delay(10);
}

/*****************************************************************************/

/* Auxiliar Functions */

static void system_reboot(void)
{
    Serial.println("Rebooting the system...");
    Serial.println("");
    while(1);
}
