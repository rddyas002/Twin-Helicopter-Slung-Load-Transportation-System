
#ifndef IO_H
#define	IO_H

#include <plib.h>
#include <stdbool.h>
#include <string.h>
#include "HardwareProfile.h"
#include <GenericTypeDefs.h>
#include <math.h>
#include "MDD File System/FSIO.h"
#include "spektrumRX.h"

#ifdef IO_H_IMPORT
	#define IO_EXTERN
#else
	#define IO_EXTERN extern
#endif

typedef enum{
            SYS_BOOT,
            SYS_MANUAL,
            SYS_TAKEOFF,
            SYS_FLY,
            SYS_LAND,
            SYS_ERROR,
} IO_SYSTEM_STATE;

typedef enum{
            FLY_ATTITUDE,
            FLY_POSITION
} IO_FLY_STATE;

typedef enum{
            LAND_NORMAL,
            LAND_FORCED
} IO_LAND_STATE;

typedef enum{
            NO_ERROR,
            WRITE_ERROR,
            READ_ERROR,
            OPEN_ERROR,
            CLOSE_ERROR
} IO_FILE_ERROR;

#define SD_WRITE_BUFFER_SIZE 512
typedef struct {
    int file_id;
    bool file_opened;
    bool close_file;
    FSFILE * fsFile;
    char file_name[20];
    char data_buffer[SD_WRITE_BUFFER_SIZE];
    char interrupt_buffer[SD_WRITE_BUFFER_SIZE];
    char overflow_buffer[SD_WRITE_BUFFER_SIZE];
    UINT16 overflow_length;
    bool buffer_overflow;
    UINT16 write_length;
    unsigned int data_buffer_len;
    UINT64 time;
    IO_FILE_ERROR error;
    bool flush_active;
    UINT16 int_write_length;
} IO_file;

typedef struct {
    char interrupt_buffer[SD_WRITE_BUFFER_SIZE];
    UINT16 write_length;
} IO_buffer;

#define IO_OUTPUT_TRIS              0
#define IO_INPUT_TRIS               1

#define IO_HEARTBEAT_TRIS()         PORTSetPinsDigitalOut(IOPORT_C, BIT_1)
#define IO_TOGGLE_HEARTBEAT()       PORTToggleBits(IOPORT_C, BIT_1)
#define IO_SET_HEARTBEAT()          PORTSetBits(IOPORT_C, BIT_1)
#define IO_CLEAR_HEARTBEAT()        PORTClearBits(IOPORT_C, BIT_1)
#define IO_PROGRAM_SWITCH_TRIS()    PORTSetPinsDigitalIn(IOPORT_A, BIT_1)
#define IO_DEBUG_TRIS()             PORTSetPinsDigitalOut(IOPORT_E, BIT_7)
#define IO_LOW                      0
#define IO_HIGH                     1
#define IO_DEBUG_LAT_LOW()          PORTClearBits(IOPORT_E, BIT_7)
#define IO_DEBUG_LAT_HIGH()         PORTSetBits(IOPORT_E, BIT_7)
#define IO_DEBUG_TOGGLE()           PORTToggleBits(IOPORT_E, BIT_7)
#define IO_LEVELSHIFT1TRIS()        PORTSetPinsDigitalOut(IOPORT_A, BIT_4)
#define IO_LEVELSHIFT2TRIS()        PORTSetPinsDigitalOut(IOPORT_A, BIT_5)

#define IO_PWM_PERIOD               (500)

#define TOGGLE_PER_SEC              (1000)
#define CORE_TICK_RATE              (GetSystemClock()/2/TOGGLE_PER_SEC)

#define IO_ERROR_BLINK              (1000)

#define IO_ADC_READ                 (100)
#define IO_SD_FLUSH                 (10)
#define IO_DATA_PERIOD              (20)
#define IO_STATE_PROJECT            (20)
#define IO_CONTROL_PERIOD_MATCH     (3125)

#define IO_SPEED_TIMEOUT            (2000)

#define IO_RPM_MAX                  (3500)
#define IO_RPM_MIN                  (400)
#define IO_PWM_SAT_MAX              (0xFF)
#define IO_PWM_SAT_MIN              (0x00)

// RADIO input capture pins
#define IO_OPTO_ENCODER_PIND           (0x40)
#define IO_GYROGAIN_PIND               (0x80)
#define IO_ULTRASONIC_PIND             (0x2000)

// PGEC1/AN1/CN3/RB1 (24): ESC
// AN2/C2IN-/CN4/RB2 (23): RIGHT
// AN3/C2IN+/CN5/RB3 (22): REAR
// AN4/C1IN-/CN6/RB4 (21): RUDDER
// AN5/C1IN+/V BUSON /CN7/RB5 (20): LEFT
#define IO_ESC_PINB                    (0x02)
#define IO_RIGHT_PINB                  (0x04)
#define IO_REAR_PINB                   (0x08)
#define IO_RUDDER_PINB                 (0x10)
#define IO_LEFT_PINB                   (0x20)

#define CN_PWM_MAX                      (84000) // equivalent to 2.2ms
#define CN_PWM_MIN                      (32000) // equivalent to 0.8ms

#define IO_ULTRASONIC_DT_MAX            (1774800)
#define IO_ULTRASONIC_DT_MIN            (37120)

#define IO_RPM_2_RAD_PER_SEC        (2*M_PI/60)

// Interrupt Priorities
// ADC
#define IO_ADC_IPL              ipl1
#define IO_ADC_PRIORITY         ADC_INT_PRI_1
#define IO_ADC_SUBPRIORITY      ADC_INT_SUB_PRI_3
// TIMER3
#define IO_T3_IPL               ipl5
#define IO_T3_PRIORITY          T3_INT_PRIOR_5
#define IO_T3_SUBPRIORITY       T3_INT_SUB_PRIOR_3
// CT
#define IO_CT_IPL               IPL7SRS
#define IO_CT_PRIORITY          CT_INT_PRIOR_7
#define IO_CT_SUBPRIORITY       CT_INT_SUB_PRIOR_2
// CN
#define IO_CN_IPL               IPL7SRS
#define IO_CN_PRIORITY          CHANGE_INT_PRI_7
#define IO_CN_SUBPRIORITY       INT_SUB_PRIORITY_LEVEL_3

// File System parameters
#define IO_FS_DMESG "dmesg.txt"
#define IO_FS_LOG "log.bin"

#define IO_AUTO_MODE            (1 << 0)
#define IO_RN131_TIMEOUT        (1 << 1)
#define IO_RN131_SYNC           (1 << 2)
#define IO_POSITION_CNTL        (1 << 3)
#define IO_STABLE_EKF           (1 << 4)
#define IO_SPEKTRUM_TIMEOUT     (1 << 5)
#define IO_PWM_ACK              (1 << 6)
#define IO_SPEED_SENSOR_TIMEOUT (1 << 7)

#define IO_SERVO_RIGHT_SIGN     (1)
#define IO_SERVO_REAR_SIGN     (-1)
#define IO_SERVO_LEFT_SIGN     (-1)
typedef enum IO_PID_TYPE_t{
    IO_PID,
    IO_PD
} IO_PID_TYPE;

IO_EXTERN void IO_setup(void);
IO_EXTERN void IO_setupPWM_default(void);
IO_EXTERN UINT16 IO_PWM_dc(float dc);
IO_EXTERN IO_SYSTEM_STATE IO_getSystemState(void);
IO_EXTERN void IO_setSystemState(IO_SYSTEM_STATE state);
IO_EXTERN bool IO_getSpeedTimeout(void);
IO_EXTERN UINT16 IO_getBatteryVoltage(void);
IO_EXTERN void IO_delayms(unsigned int num_ms_delay);
IO_EXTERN void IO_initialize_FS(void);
IO_EXTERN bool IO_initializeFile(IO_file * IO_file_ds, const char * IO_file_name);
IO_EXTERN void IO_initialiseLogFiles(void);
IO_EXTERN void IO_terminate_FS(void);
IO_EXTERN bool IO_flush_file(IO_file * io_file);
IO_EXTERN bool IO_flush_FS(void);
IO_EXTERN void IO_logMessage(char * str, UINT16 len);
IO_EXTERN void IO_dmesgMessage(char * str, UINT16 len);
IO_EXTERN bool IO_getSDFlush(void);
IO_EXTERN void IO_setSDFlush(bool val);
IO_EXTERN void IO_terminate_FS(void);
IO_EXTERN bool IO_getCloseFiles(void);
IO_EXTERN void IO_setCloseFiles(bool val);
IO_EXTERN void IO_LED_init(int toggle_number);
IO_EXTERN void IO_leds_on(float power);
IO_EXTERN volatile UINT32 IO_get_time_ms(void);
IO_EXTERN volatile UINT32 IO_updateCoreTime_us(void);
IO_EXTERN volatile UINT32 IO_getTime_us(void);
IO_EXTERN void IO_adjIO_time_us(INT32 offset);
IO_EXTERN bool IO_getLocalControl(void);
IO_EXTERN void IO_setLocalControl(bool val);
IO_EXTERN void IO_changeNotificationSetup(void);
IO_EXTERN void IO_logRadio(void);
IO_EXTERN void IO_logQuaternion(void);
IO_EXTERN void IO_SystemIdentification(void);
IO_EXTERN void IO_logControl(void);
IO_EXTERN bool IO_getRadioError(void);
IO_EXTERN void IO_logRadioMapping(void);
IO_EXTERN bool IO_getLEDState(void);
IO_EXTERN volatile UINT32 IO_getLEDON_time(void);
IO_EXTERN void IO_sendData(UINT32 time_us);
IO_EXTERN void IO_Control_Int_Enable(void);
IO_EXTERN void IO_control_exec(void);
IO_EXTERN UINT32 IO_getDataTime(void);
IO_EXTERN bool IO_writePWMmodule(float esc, float lateral, float longitudinal, float collective, float delta_rudder);
IO_EXTERN float IO_ESC_controller(float ref_speed, float dt);
IO_EXTERN float IO_getAltitudeReference(float throttle);
IO_EXTERN void IO_main_control(float dt);
IO_EXTERN void IO_getRefInputs(float * throttle, float * del_right, float * del_rear,
        float * del_left, float * alt_ref, float * yaw_ref, volatile float * roll_ref, volatile float * pitch_ref,
        volatile float * x_ref, volatile float * y_ref);
IO_EXTERN void IO_getRotorInputs(float * main_collective, float * lat, float * lon);
IO_EXTERN void IO_setLandState(IO_LAND_STATE state);

IO_EXTERN float IO_getGyroGain_dt(void);
IO_EXTERN float IO_getUltrasonic_dt(void);
IO_EXTERN unsigned short int IO_getRotorSpeed(void);
IO_EXTERN float IO_getFilteredRotorSpeed(void);
IO_EXTERN  char IO_limit_PWM(int sig);

IO_EXTERN UINT16 IO_getGyroGain(void);
IO_EXTERN UINT16 IO_getUltrasonic(void);
IO_EXTERN UINT16 IO_getESC(void);
IO_EXTERN UINT16 IO_getRight(void);
IO_EXTERN UINT16 IO_getRear(void);
IO_EXTERN UINT16 IO_getRudder(void);
IO_EXTERN UINT16 IO_getLeft(void);
IO_EXTERN float IO_getAltitude(void);

IO_EXTERN float IO_limits(float lower, float upper, float input);
IO_EXTERN void IO_filter_1st_order(float n1, float n2, float d1, float d2, float input[3], float output[3]);
IO_EXTERN void IO_filter_2nd_order(float n0, float n1, float n2, float d0, float d1, float d2, float input[3], float output[3]);
IO_EXTERN float IO_filter_notch(float wn, float damp_p, float damp_z, float Ts, float input[3], float output[3]);
IO_EXTERN float IO_PI_with_limits(const float Kp, const float omega_Ti, const float Ts, const float current_error,
    float filter_input[3], float filter_output[3], const float lower_limit, const float upper_limit);
IO_EXTERN float IO_lead_lag(float omega_lead, float omega_lag, float Ts, float filter_input[3], float filter_output[3]);
IO_EXTERN float IO_complex_lead_lag(float wn_z, float damp_z, float wn_p, float damp_p, float Ts, float input[3], float output[3]);
IO_EXTERN float IO_heave_control(float heave_reference, float height_sensor, float lower_ctrl_limit, float upper_ctrl_limit, float dt, bool initialise);
IO_EXTERN float IO_yaw_control(float dt, float yaw_ref);
IO_EXTERN float IO_LPF_1st(float input[3], float output[3], float dt, float wn);

IO_EXTERN void IO_PIDComputation(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]);
IO_EXTERN void IO_PID_with_limits(float Kp, float omega_lead, float omega_Ti, float alpha, float Ts,
        float int_inputs[3], float int_outputs[3], float current_error,
        float int_lower_limit, float int_upper_limit,
        float diff_inputs[3], float diff_outputs[3]);
IO_EXTERN void IO_PID_rateAux(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]);
IO_EXTERN float IO_roll_control(float roll_reference, float dt, bool initialise);
IO_EXTERN float IO_pitch_control(float pitch_reference, float dt, bool initialise);
IO_EXTERN void IO_position_control(float x_ref, float y_ref, float z_ref, volatile float * roll_cmd, volatile float * pitch_cmd, float dt, bool initialise);
IO_EXTERN void IO_shiftData(float data[3]);
IO_EXTERN float IO_filter_speed(float speed_in, float dt, float wn);
IO_EXTERN float IO_filter_altitude_prefilter(float alt_in, float dt, float wn);
IO_EXTERN float IO_filter_yaw_prefilter(float yaw_in, float dt, float wn);
IO_EXTERN void IO_filter_translationref_prefilter(volatile float * x_ref, volatile float * y_ref, float dt, float wn);
IO_EXTERN float IO_ESC_feedforward(float collective);
IO_EXTERN void IO_HeaveYaw_decoupler(float collective_input, float yawrate_input);

IO_EXTERN bool IO_getStateProject(void);
IO_EXTERN void IO_setStateProject(bool val);

volatile float IO_longitudinal;
volatile float IO_lateral;
volatile float IO_collective;
volatile float IO_esc;
volatile float IO_ref_speed;
volatile float IO_tailrate;
volatile float IO_roll_ref;
volatile float IO_pitch_ref;
volatile float IO_yaw_ref;
volatile float IO_x_ref;
volatile float IO_y_ref;
volatile float IO_z_ref;

#endif	/* IO_H */

