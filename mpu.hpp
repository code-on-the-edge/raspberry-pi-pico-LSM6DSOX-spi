#ifndef MPU_H
#define MPU_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "math.h"

#define GRAVITY 9.80665f
#define ONE_HUNDRED_CM 9.80665f
#define SAMPLE_INTERVAL 0.004

class Mpu {
    public:

        typedef struct Orientation {
            float roll_rads;
            float pitch_rads;
            float yaw_rads;
        } Orientation;

        static bool isStationary(float ax, float ay, float az, float threshold);
        static void print_raw_buffer_data_for_motion_cal(int16_t *accel_raw_buffer, int16_t *gyro_raw_buffer, int16_t *mag_raw_buffer);
        
        static float get_normalized(const float *data, float nominator);
        static float get_p_roll_phi(float *accel);
        static float get_q_pitch_theta(float *accel);
        static float get_r_yaw_psi(float *mag);
        static Orientation *get_roll_pitch_yaw_angle_rads(float &ax, float &ay, float &az, float &mx, float &my, float &mz);
        static float get_deg_to_rads(float data);
        static float get_rads_to_deg(float data);
        static bool is_normalized(float x_norm, float y_norm, float z_norm);
        static float get_vertical_velocity(const float ax, const float ay, const float az, const float mx, const float my, const float mz);
        static float get_accel_inertial_az(const float ax, const float ay, const float az, const float mx, const float my, const float mz);
    private:
        static const float pi;
        static float vertical_velocity;

        static float phi;
        static float theta;
        static float psi;
        static float roll_pitch_yaw_angles[3];
};
#endif