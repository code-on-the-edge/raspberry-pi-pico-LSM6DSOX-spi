#include "mpu.hpp"

const float Mpu::pi = 3.142;
float Mpu::vertical_velocity = 0;
float Mpu::phi=0;
float Mpu::theta=0;
float Mpu::psi=0;
float Mpu::roll_pitch_yaw_angles[3] = {0};


void Mpu::print_raw_buffer_data_for_motion_cal(int16_t *accel_raw_buffer, int16_t *gyro_raw_buffer, int16_t *mag_raw_buffer) {
    for(int i = 0; i < 3; i++) {
        accel_raw_buffer[i] = accel_raw_buffer[i] * (8192/9.8);
        gyro_raw_buffer[i] = gyro_raw_buffer[i] * (180/3.14159*16);
        mag_raw_buffer[i] = mag_raw_buffer[i] * 10;
    }
    printf("Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",  int(accel_raw_buffer[0]),
                                                int(accel_raw_buffer[1]),
                                                int(accel_raw_buffer[2]),
                                                int(gyro_raw_buffer[0]),
                                                int(gyro_raw_buffer[1]),
                                                int(gyro_raw_buffer[2]),
                                                int(mag_raw_buffer[0]),
                                                int(mag_raw_buffer[1]),
                                                int(mag_raw_buffer[2]));
}

float Mpu::get_normalized(const float *data, float nominator) {
    float sqrt_xyz = sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
    return nominator / sqrt_xyz;
}


bool Mpu::is_normalized(float x_norm, float y_norm, float z_norm) {
    float check = x_norm * x_norm + y_norm * y_norm + z_norm * z_norm;
    if (std::abs(check - 1.0f) > 0.01f) { // Allow small tolerance
        printf("Not normalized : check : %f\n", check);
        return false;
    }
    return true;  
}


float Mpu::get_p_roll_phi(float *accel) {
    float ay_norm = get_normalized(accel, accel[1]);
    float az_norm = get_normalized(accel, accel[2]);
    phi = atan2f(ay_norm, az_norm);
    return phi;
}

float Mpu::get_q_pitch_theta(float *accel) {
    float norm = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float ax_norm = accel[0] / norm;
    float ay_norm = accel[1] / norm;
    float az_norm = accel[2] / norm;
    is_normalized(ax_norm, ay_norm, az_norm);
    theta = atan2f(ax_norm, sqrt(ay_norm * ay_norm + az_norm * az_norm));
    return theta;
}


float Mpu::get_r_yaw_psi(float *mag) {
    float mx_norm = get_normalized(mag, mag[0]);
    float my_norm = get_normalized(mag, mag[1]);
    float mz_norm = get_normalized(mag, mag[2]);
    is_normalized(mx_norm, my_norm, mz_norm);
    float bz2 = my_norm * sin(phi) + mz_norm * cos(phi);
    float by2 = mz_norm * sin(phi) - my_norm  * cos(theta);
    float bx3 = mx_norm * cos(theta) + bz2 * sin(theta);
    psi = atan2f(by2, bx3);
    return psi;
}

Mpu::Orientation *Mpu::get_roll_pitch_yaw_angle_rads(float &ax, float &ay, float &az, float &mx, float &my, float &mz) {
    static Orientation orientation_i;
    float accel[3] = {ax,ay,az};
    float mag[3] = {mz,my,mz};
    orientation_i.roll_rads = get_p_roll_phi(accel);
    orientation_i.pitch_rads = -get_q_pitch_theta(accel);
    orientation_i.yaw_rads = get_r_yaw_psi(mag);
    return &orientation_i;    
}

float Mpu::get_deg_to_rads(float data) {
    return data * (M_PI / 180.0f);
}

float Mpu::get_rads_to_deg(float data) {
    return data * (180.0f / M_1_PI);
}

float Mpu::get_accel_inertial_az(const float ax, const float ay, const float az,
                                 const float mx, const float my, const float mz) {
    // float accel[3] = {ax,ay,az};
    // float mag[3] = {mx,my,mz};
    // float pitch_angle = get_roll_pitch_yaw_angle_rads(accel, mag)[1];
    // float roll_angle = get_roll_pitch_yaw_angle_rads(accel, mag)[0];
    // float accel_inertial_az_m_ss, A, B, C;
	// A = ax * sin(pitch_angle * (Mpu::pi/180.0));
    // B = ay * sin(roll_angle * (Mpu::pi/180.0)) * cos(pitch_angle * (Mpu::pi/180.0));
    // C = az * cos(roll_angle * (Mpu::pi/180.0)) * cos(pitch_angle * (Mpu::pi/180.0));
    // accel_inertial_az_m_ss = A + B + C;
	// return (accel_inertial_az_m_ss-1)*GRAVITY;


    return 0.0; // temp -- delete me later
}

float Mpu::get_vertical_velocity(const float ax, const float ay, const float az,
                                 const float mx, const float my, const float mz) {
    float accel_inertial_az_m_ss = get_accel_inertial_az(ax, ay, az, mx, my, mz);
    vertical_velocity = vertical_velocity + (accel_inertial_az_m_ss * SAMPLE_INTERVAL);
    return vertical_velocity;
}