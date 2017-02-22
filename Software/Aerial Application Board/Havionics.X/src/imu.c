#define IMU_H_IMPORT

#include "imu.h"
#include "math.h"

bool IMU_i2cOpen = false;

// in radians
float IMU_roll = 0.0;
float IMU_pitch = 0.0;
float IMU_yaw = 0.0;

static float angular_rate[3] = {0};

volatile float IMU_bias_upper_limit_int = 1;
volatile float IMU_bias_lower_limit_int = -1;

// attitude estimation
float IMU_q[4] = {0,1,0,0};

void IMU_openI2C(void){
    if (!IMU_i2cOpen){
        // Use I2C1 & set baud rate to 400kHz
        OpenI2C1( I2C_EN | I2C_SLW_EN , I2C1_BAUD );
        IMU_i2cOpen = true;
    }

    IMU_bias_upper_limit_int = IMU_BIAS_UPPER_LIMIT*fabs(IMU_OMEGA_TI/IMU_OMEGA_KP);
    IMU_bias_lower_limit_int = IMU_BIAS_LOWER_LIMIT*fabs(IMU_OMEGA_TI/IMU_OMEGA_KP);
}

bool IMU_tryConfig(uchar reg, uchar data, uchar ADDRESS){
    uchar reg_data = 0, counter = 0;

    while(counter++ < IMU_TRYCONFIG){
        reg_data =  IMU_writeI2C1Read(reg, data, ADDRESS);
        if (reg_data == data)
            return true;
    }

    return false;
}

uchar IMU_writeI2C1Read(uchar reg, uchar data, uchar ADDRESS){
    IMU_writeI2C1(reg, data, ADDRESS);
    return IMU_readI2C1(reg, ADDRESS);
}

void IMU_writeI2C1(uchar reg, uchar data, uchar ADDRESS){
    unsigned char WRITE_ADDRESS;
    WRITE_ADDRESS = (ADDRESS << 1);

    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(WRITE_ADDRESS);
    IdleI2C1();

    MasterWriteI2C1(reg);
    IdleI2C1();

    MasterWriteI2C1(data);
    IdleI2C1();

    StopI2C1();
    IdleI2C1();
}

uchar IMU_readI2C1(uchar reg, uchar ADDRESS){
    uchar WRITE_ADDRESS, READ_ADDRESS, data;
    WRITE_ADDRESS = (ADDRESS << 1);
    READ_ADDRESS = ((ADDRESS << 1) | 0x01);

    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(WRITE_ADDRESS);
    IdleI2C1();

    MasterWriteI2C1(reg);
    IdleI2C1();

    RestartI2C1();
    IdleI2C1();

    MasterWriteI2C1(READ_ADDRESS);
    IdleI2C1();

    data = MasterReadI2C1();
    IdleI2C1();

    NotAckI2C1();
    IdleI2C1();

    StopI2C1();
    IdleI2C1();

    return data;
}

void IMU_propagateState(float dt){
    static int normalize_counter = 0;
    static float qx_bias_input[3] = {0}, qx_bias_output[3] = {0};
    static float qy_bias_input[3] = {0}, qy_bias_output[3] = {0};
    static float qz_bias_input[3] = {0}, qz_bias_output[3] = {0};
    float q_next[4] = {0};
    float q_err[4] = {0};

    // determine rate correction factor
    // If recent valid data had come through comms do est
    if (RN131_ekfStable() && !RN131_getTimeout()){
        float IMU_q_conj[4] = {IMU_q[0],-IMU_q[1],-IMU_q[2],-IMU_q[3]};
        float Camera_quaternion[4] = {RN131_getQuaternion_q0(), RN131_getQuaternion_q1(), RN131_getQuaternion_q2(), RN131_getQuaternion_q3()};
        // get quaternion error
        IMU_quaternionMultiply(IMU_q_conj,&Camera_quaternion[0],&q_err[0]);
        IMU_normalizeVector(q_err,4);
    }

    // If there is a timeout q_err is zero an has no effect on offsets
    float omega_offset_x = IO_PI_with_limits(IMU_OMEGA_KP, IMU_OMEGA_TI, dt, q_err[1],
        &qx_bias_input[0], &qx_bias_output[0], IMU_bias_lower_limit_int, IMU_bias_upper_limit_int);
    float omega_offset_y = IO_PI_with_limits(IMU_OMEGA_KP, IMU_OMEGA_TI, dt, q_err[2],
        &qy_bias_input[0], &qy_bias_output[0], IMU_bias_lower_limit_int, IMU_bias_upper_limit_int);
    float omega_offset_z = IO_PI_with_limits(IMU_OMEGA_KP, IMU_OMEGA_TI, dt, q_err[3],
        &qz_bias_input[0], &qz_bias_output[0], IMU_bias_lower_limit_int, IMU_bias_upper_limit_int);

    float gyro_x_corrected = gyro_wx_rad() + omega_offset_x;
    float gyro_y_corrected = gyro_wy_rad() + omega_offset_y;
    float gyro_z_corrected = gyro_wz_rad() + omega_offset_z;

    q_next[0] = IMU_q[0] - dt*0.5*(IMU_q[1]*gyro_x_corrected + IMU_q[2]*gyro_y_corrected + IMU_q[3]*gyro_z_corrected);
    q_next[1] = IMU_q[1] + dt*0.5*(IMU_q[0]*gyro_x_corrected - IMU_q[3]*gyro_y_corrected + IMU_q[2]*gyro_z_corrected);
    q_next[2] = IMU_q[2] + dt*0.5*(IMU_q[3]*gyro_x_corrected + IMU_q[0]*gyro_y_corrected - IMU_q[1]*gyro_z_corrected);
    q_next[3] = IMU_q[3] - dt*0.5*(IMU_q[2]*gyro_x_corrected - IMU_q[1]*gyro_y_corrected - IMU_q[0]*gyro_z_corrected);

    memcpy(&IMU_q[0], &q_next[0],4*sizeof(float));

    if (normalize_counter++ == 10){
        IMU_normalizeVector(IMU_q,4);
        normalize_counter = 0;
    }

    float quat_ret[4], temp_yaw = 0;
    IMU_rotateXby180(&IMU_q[0],&quat_ret[0]);
    // returns radians
    IMU_quaternion2euler(quat_ret, &IMU_roll, &IMU_pitch, &temp_yaw);

    // Phase wrap yaw angle
    int yaw_diff = (int)COM_round((IMU_yaw - temp_yaw)/(2*M_PI));
    IMU_yaw = temp_yaw + yaw_diff*(2*M_PI);
}

void IMU_quaternion2euler(float q[4], float * roll, float * pitch, float * yaw){
  *roll = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 1.0 - 2.0 *
                        (pow(q[1], 2.0) + pow(q[2], 2.0)));
  *pitch = asin(2.0 * (q[0] * q[2] - q[3] * q[1]));
  *yaw = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 *
                       (pow(q[2], 2.0) + pow(q[3], 2.0)));
}


void IMU_quaternionRotate(const float q1[4], const float q2[4], float q[4]){
  float b_q2[16];
  int i9;
  int i10;

  /*  rotation q1 by q2 */
  b_q2[0] = q2[0];
  b_q2[4] = -q2[1];
  b_q2[8] = -q2[2];
  b_q2[12] = -q2[3];
  b_q2[1] = q2[1];
  b_q2[5] = q2[0];
  b_q2[9] = -q2[3];
  b_q2[13] = q2[2];
  b_q2[2] = q2[2];
  b_q2[6] = q2[3];
  b_q2[10] = q2[0];
  b_q2[14] = -q2[1];
  b_q2[3] = q2[3];
  b_q2[7] = -q2[2];
  b_q2[11] = q2[1];
  b_q2[15] = q2[0];
  for (i9 = 0; i9 < 4; i9++) {
    q[i9] = 0.0;
    for (i10 = 0; i10 < 4; i10++) {
      q[i9] += b_q2[i9 + (i10 << 2)] * q1[i10];
    }
  }
}

void IMU_rotateXby180(const float q_in[4], float q_out[4]){
    q_out[0] = -q_in[1];
    q_out[1] =  q_in[0];
    q_out[2] = -q_in[3];
    q_out[3] =  q_in[2];
}

float * IMU_getRoll(void){
    return &IMU_roll;
}

float * IMU_getPitch(void){
    return &IMU_pitch;
}

float * IMU_getYaw(void){
    return &IMU_yaw;
}

INT16 IMU_getRoll16BIT(void){
    // Gives 0.01 degree resolution on receive side
    return (INT16)(IMU_roll*18000/M_PI);
}

INT16 IMU_getPitch16BIT(void){
    // Gives 0.01 degree resolution on receive side
    return (INT16)(IMU_pitch*18000/M_PI);
}

INT16 IMU_getYaw16BIT(void){
    // Gives 0.01 degree resolution on receive side
    return (INT16)(IMU_yaw*18000/M_PI);
}

float * IMU_getQuaternion(void){
    return &IMU_q[0];
}

float IMU_getQuaternion_q0(void){
    return IMU_q[0];
}

float IMU_getQuaternion_q1(void){
    return IMU_q[1];
}

float IMU_getQuaternion_q2(void){
    return IMU_q[2];
}

float IMU_getQuaternion_q3(void){
    return IMU_q[3];
}

void IMU_normalizeVector(float array[], int len){
    unsigned char k;
    float norm = 0.0;

    for (k = 0; k < len; k++){
        norm += pow(array[k],2);
    }

    norm = sqrt(norm);

    for (k = 0; k < len; k++){
        array[k] = array[k]/norm;
    }
}

void IMU_quaternionError(const float q1[4], const float q2[4], float q_err[4]){
  float b_q1[16];
  float b_q2[4];
  int i0;
  int i1;
  b_q1[0] = q1[0];
  b_q1[4] = -q1[1];
  b_q1[8] = -q1[2];
  b_q1[12] = -q1[3];
  b_q1[1] = q1[1];
  b_q1[5] = q1[0];
  b_q1[9] = -q1[3];
  b_q1[13] = q1[2];
  b_q1[2] = q1[2];
  b_q1[6] = q1[3];
  b_q1[10] = q1[0];
  b_q1[14] = -q1[1];
  b_q1[3] = q1[3];
  b_q1[7] = -q1[2];
  b_q1[11] = q1[1];
  b_q1[15] = q1[0];
  b_q2[0] = q2[0];
  b_q2[1] = -q2[1];
  b_q2[2] = -q2[2];
  b_q2[3] = -q2[3];
  for (i0 = 0; i0 < 4; i0++) {
    q_err[i0] = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      q_err[i0] += b_q1[i0 + (i1 << 2)] * b_q2[i1];
    }
  }
}

void IMU_quaternionMultiply(const float q1[4], const float q2[4], float q[4]){
    float b_q1[16];
    int i0;
    int i1;
    b_q1[0] = q1[0];
    b_q1[4] = -q1[1];
    b_q1[8] = -q1[2];
    b_q1[12] = -q1[3];
    b_q1[1] = q1[1];
    b_q1[5] = q1[0];
    b_q1[9] = -q1[3];
    b_q1[13] = q1[2];
    b_q1[2] = q1[2];
    b_q1[6] = q1[3];
    b_q1[10] = q1[0];
    b_q1[14] = -q1[1];
    b_q1[3] = q1[3];
    b_q1[7] = -q1[2];
    b_q1[11] = q1[1];
    b_q1[15] = q1[0];
    for (i0 = 0; i0 < 4; i0++) {
        q[i0] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
            q[i0] += b_q1[i0 + (i1 << 2)] * q2[i1];
        }
    }
}

/* this function rotates a vector from body frame to inertial with the
   quaternion generated from the state-propagation */
void Reb(const float q[4], const float x[3], float y[3]){
  float b_q[9];
  int i12;
  int i13;
  b_q[0] = ((q[0] * q[0] + q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
  b_q[3] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  b_q[6] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  b_q[1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  b_q[4] = ((q[0] * q[0] - q[1] * q[1]) + q[2] * q[2]) - q[3] * q[3];
  b_q[7] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  b_q[2] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  b_q[5] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  b_q[8] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
  for (i12 = 0; i12 < 3; i12++) {
    y[i12] = 0.0;
    for (i13 = 0; i13 < 3; i13++) {
      y[i12] += b_q[i12 + 3 * i13] * x[i13];
    }
  }
}

#undef IMU_H_IMPORT