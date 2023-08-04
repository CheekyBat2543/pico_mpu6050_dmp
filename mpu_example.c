#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "inv_mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MPU_SDA_PIN 12
#define MPU_SCL_PIN 13
#define MPU_INT_PIN 16
#define GRAVITY_ACCEL 9.80665f

int main() {
    stdio_init_all();

    struct int_param_s mpu6050_param = {
        .int_pin = MPU_INT_PIN,
        .int_event = GPIO_IRQ_LEVEL_LOW,
        .cb = NULL
    };

    mpu_data_f  mpu_data= {
        .accel_x_f = 0,
        .accel_y_f = 0,
        .accel_z_f = 0,
        .gyro_x_f = 0,
        .gyro_y_f = 0,
        .gyro_z_f = 0,
        .quat_w_f = 0,
        .quat_x_f = 0,
        .quat_y_f = 0,
        .quat_z_f = 0,
        .pitch = 0,
        .roll = 0,
        .yaw = 0
    };

    /* Initiliaze I2C. To use i2c1, "#define USE_I2C1" must be added to inv_mpu6050.c */
    short data[3] = {0, 0, 0};
    i2c_init(i2c_default, 400 * 1000);
    gpio_init(MPU_SDA_PIN);
    gpio_init(MPU_SCL_PIN);
    gpio_set_function(MPU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MPU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU_SDA_PIN);
    gpio_pull_up(MPU_SCL_PIN);
    sleep_ms(200);
    /* Initiliaze MPU6050 */
    while(mpu_init(&mpu6050_param)) {
        printf("Could not initiliaze MPU6050\n");
        sleep_ms(1000);
    }
    printf("Mpu is initiliazed.\n");
    sleep_ms(100);
    /* To get the best performance from dmp quaternions, Accel = -+2G and Gyro = -+2000DPS is recommended */
    mpu_set_accel_fsr(2);
    sleep_ms(100);
    mpu_set_gyro_fsr(2000);
    sleep_ms(100);
    /* Initiliaze low pass filter and high pass filter */
    mpu_set_lpf(42);
    sleep_ms(100);
    mpu_set_hpf(MPU6050_DHPF_1_25HZ);
    sleep_ms(100);
    /* RP2020 can easily handle 1khz reading speed from MPU6050*/
    mpu_set_sample_rate(1000);
    sleep_ms(100);
    /* Configure which sensors are pushed to the FIFO */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    sleep_ms(100);
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    sleep_ms(100);
    /* Get the accelerometer and gyroscope conversion factor to convert hardware units to G or DPS. 
    Formula is: (Hardware_Units / Sensor_Sensitivity) */
    unsigned short accel_sens = 0.0f;
    float gyro_sens = 0.0f;
    mpu_get_accel_sens(&accel_sens);
    mpu_get_gyro_sens(&gyro_sens);

    /* Load the firmware of DMP.*/
    if(dmp_load_motion_driver_firmware()) {
        printf("DMP could not be initiliazed.\n");
        sleep_ms(1000);
        return 0;
    } else {
        printf("DMP is initiliazed.\n");
        sleep_ms(100);
        /* Set FIFO rate of DMP to 200 to get the best performance for quaternion calculations */
        dmp_set_fifo_rate(200U);
        sleep_ms(100);
        /* Enable DMP */
        mpu_set_dmp_state(1);
        sleep_ms(100);
        /* Enable which features are pushed to the fifo. 
        If DMP_FEATURE_GYRO_CAL is active, the sensor automatically calibrates the gyro if there is no motion for 8 seconds */
        dmp_enable_feature(DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_6X_LP_QUAT);
        sleep_ms(100);
        /* Calculate the accelerometer and gyroscope by pushing bias values to the registers */
        long gyro_bias[] = {0, 0, 0};
        long accel_bias[] = {0, 0, 0};
        mpu_find_gyro_calibration_biases(gyro_bias);
        mpu_find_accel_calibration_biases_pid(accel_bias);
        mpu_set_gyro_bias_reg(gyro_bias);
        mpu_set_accel_bias_6050_reg(accel_bias);
        dmp_set_gyro_bias(gyro_bias);
        dmp_set_accel_bias(accel_bias);
    }
    sleep_ms(1000);
    while (1) {
        int16_t gyro[3] = {0, 0, 0};
        int16_t accel[3] = {0, 0, 0};
        long quat[4]   = {0, 0, 0, 0};
        unsigned long timestamp = 0;
        /* Sensor mask to choose which values are read from the FIFO */
        short sensors = INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_WXYZ_QUAT;
        unsigned char more = 0;
        dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
        if(sensors & (INV_XYZ_ACCEL | INV_XYZ_GYRO | INV_WXYZ_QUAT)) {

            mpu_data.gyro_x_f = (float)(gyro[0] / gyro_sens);
            mpu_data.gyro_y_f = (float)(gyro[1] / gyro_sens);
            mpu_data.gyro_z_f = (float)(gyro[2] / gyro_sens);
            printf("\nGyro          ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.gyro_x_f, mpu_data.gyro_y_f, mpu_data.gyro_z_f);

            mpu_data.accel_x_f = (float)(GRAVITY_ACCEL*accel[0]) / accel_sens ;
            mpu_data.accel_y_f = (float)(GRAVITY_ACCEL*accel[1]) / accel_sens ;
            mpu_data.accel_z_f = (float)(GRAVITY_ACCEL*accel[2]) / accel_sens ;
            printf("Accelerometer ==> x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.accel_x_f, mpu_data.accel_y_f, mpu_data.accel_z_f);
            /* Normalize the quaternion values by converting them to floats from Q30 format */
            mpu_data.quat_w_f = qToFloat(quat[0], 30);
            mpu_data.quat_x_f = qToFloat(quat[1], 30);
            mpu_data.quat_y_f = qToFloat(quat[2], 30);
            mpu_data.quat_z_f = qToFloat(quat[3], 30);
            printf("Quaternions   ==> w: %6.2f, x: %6.2f, y: %6.2f, z: %6.2f\n", mpu_data.quat_w_f, mpu_data.quat_x_f, mpu_data.quat_y_f, mpu_data.quat_z_f);
            // Roll (x-axis)
            float sinr_cosp = +2.0 * (mpu_data.quat_w_f * mpu_data.quat_x_f + mpu_data.quat_y_f * mpu_data.quat_z_f);
            float cosr_cosp = +1.0 - 2.0 * (mpu_data.quat_x_f * mpu_data.quat_x_f + mpu_data.quat_y_f * mpu_data.quat_y_f);
            mpu_data.roll = atan2f(sinr_cosp, cosr_cosp);
            // Pitch (y-axis)
            float sinp = sqrtf((float)(1.0 + 2.0 * (mpu_data.quat_w_f * mpu_data.quat_y_f - mpu_data.quat_x_f * mpu_data.quat_z_f)));
            float cosp = sqrtf((float)(1.0 - 2.0 * (mpu_data.quat_w_f * mpu_data.quat_y_f - mpu_data.quat_x_f * mpu_data.quat_z_f)));
            mpu_data.pitch = (float)(2.0 * atan2f(sinp, cosp) - (float)3.14f / 2.0);            
            // Yawn (z-axis)
            float siny_cosp = 2.0 * (mpu_data.quat_w_f * mpu_data.quat_z_f + mpu_data.quat_x_f * mpu_data.quat_y_f);
            float cosy_cosp = 1.0 - 2.0 * (mpu_data.quat_y_f * mpu_data.quat_y_f + mpu_data.quat_z_f * mpu_data.quat_z_f);
            mpu_data.yaw = atan2f(siny_cosp, cosy_cosp);

            // Radians to angles
            mpu_data.roll *= 57.2958f;
            mpu_data.yaw *= 57.2958f;
            mpu_data.pitch *= 57.2958f;
            printf("Angles        ==> Roll: %5.1f, Pitch: %5.1f, Yaw: %5.1f\n", mpu_data.roll, mpu_data.pitch, mpu_data.yaw);
        } else {
            printf("No new data from fifo\n");
        }
        sleep_ms(5);
    }
    return 0;
}