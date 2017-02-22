#define IO_H_IMPORT

#include "../inc/io.h"
#include "../inc/ultrasonic.h"
#include "../inc/rn131.h"
#include "../inc/imu.h"

/*
 * Describe interrupts:
 *      CoreTimer INT PRIORITY:SUBPRIORITY 7:2
 *      Battery ADC interrupt 1:3
 *
 * Other notes:
 * IO uses Timer 1 for delays
 * IO uses Timer 2 for PWM module
 * IO uses Timer 3 for executing the control loop
 * Interrupt on CN is highest priority for pulsewidth measurements
 */

IO_file IO_log;
IO_file IO_dmesg;

char IO_sd_buffer[256];

volatile UINT32 IO_time_ms = 0;
volatile UINT64 IO_time_raw = 0;
volatile UINT32 IO_DATA_TIME = 0;
volatile IO_SYSTEM_STATE IO_SystemState = SYS_BOOT;
volatile IO_LAND_STATE IO_LandState = LAND_NORMAL;
const char * IO_SystemStateString[] = {"BOOT","MANUAL","TAKEOFF","FLY","LAND","ERROR"};
volatile IO_SYSTEM_STATE IO_SystemPreviousState = SYS_BOOT;
volatile UINT16 IO_batteryVoltage = 0;
volatile bool IO_localControl = true;
volatile bool IO_stateProject = false;
volatile bool IO_sdFlush = false;
volatile bool IO_closeFiles = false;
volatile UINT8 IO_gyro_gain = 115;
volatile UINT8 IO_statusReg = 0x00;

/* Mutex type variables */
volatile bool IO_coreupdate_mutex = false;

void IO_enableLevelShifter1(BOOL val);
void IO_enableLevelShifter2(BOOL val);
void IO_setupPWM(void);
void IO_setupTICK(void);
void IO_setupADC(void);
void IO_delayUnder200ms(unsigned int val);
bool IO_write_FS(IO_file * io_file, char * data, UINT16 len);
bool IO_write_FSwInt(IO_file * io_file, char * data, UINT16 len);
bool IO_flush_FS(void);
void IO_getConfiguration(void);
int IO_getLine(FSFILE * fd, char * string);

#define CONFIG (CN_ON | CN_IDLE_CON)
#define PINS (CN3_ENABLE | CN4_ENABLE | CN5_ENABLE | CN6_ENABLE | CN7_ENABLE | CN15_ENABLE | CN16_ENABLE | CN19_ENABLE)
#define PULLUPS (CN_PULLUP_DISABLE_ALL)

volatile unsigned int IO_rotor_speed_dt = 0, IO_gyrogain_dt = 0, IO_ultrasonic_dt = 0;
volatile unsigned int IO_esc_dt = 0, IO_right_dt = 0, IO_rear_dt = 0, IO_rudder_dt = 0, IO_left_dt = 0;
volatile UINT16 IO_esc_nom = 0, IO_right_nom = 0, IO_rear_nom = 0, IO_rudder_nom = 0, IO_left_nom = 0;
volatile float IO_rotor_speed = 0;
volatile float IO_filteredRotorSpeed = 0;
volatile unsigned int IO_rotor_speed_update = 0;
volatile bool IO_rotor_speed_timeout = false;
volatile bool IO_LED_ON = false;
volatile bool IO_RADIO_ERROR = true;
volatile bool ultrasonic_updated = false;
volatile UCHAR IO_valid_PWM_data = 0x00;
volatile UINT8 IO_PWM_packet[10] = {0};

// **********************************
// Roll channel control parameters
// ***********************************
// +ve lateral  input ----> -ve roll
#define IO_ROLL_KP              (-1200.0)
#define IO_ROLL_NOTCH_WN_Z      (16.5)
#define IO_ROLL_NOTCH_WZ_DAMP   (0.2)
#define IO_ROLL_NOTCH_WN_P      (16.5)
#define IO_ROLL_NOTCH_WP_DAMP   (1.0)
#define IO_LATERAL_BIAS         (3.0)
#define IO_ROLL_BIAS            (-2*M_PI/180)
#define IO_ROLL_ANGLE_LOWER_LIMIT   (-30*M_PI/180)
#define IO_ROLL_ANGLE_UPPER_LIMIT   (30*M_PI/180)
#define IO_LATERAL_UPPER_LIMIT  (300.0)
#define IO_LATERAL_LOWER_LIMIT  (-300.0)
#define IO_LAT_LOWER_LIMIT      (-500.0)
#define IO_LAT_UPPER_LIMIT      (500.0)

// ***********************************
// Pitch channel control parameters
// ***********************************
// +ve longitudinal input ----> -ve pitch
#define IO_PITCH_KP              (-2000.0)
#define IO_PITCH_NOTCH_WN_Z      (16.0)
#define IO_PITCH_NOTCH_WZ_DAMP   (0.2)
#define IO_PITCH_NOTCH_WN_P      (16.0)
#define IO_PITCH_NOTCH_WP_DAMP   (1.0)
#define IO_LONGITUDINAL_BIAS     (13.0)
#define IO_PITCH_BIAS           (-3.4*M_PI/180)
#define IO_PITCH_ANGLE_LOWER_LIMIT   (-30*M_PI/180)
#define IO_PITCH_ANGLE_UPPER_LIMIT   (30*M_PI/180)
#define IO_LONGITUDINAL_UPPER_LIMIT (300.0)
#define IO_LONGITUDINAL_LOWER_LIMIT (-300.0)
#define IO_LON_LOWER_LIMIT      (-500.0)
#define IO_LON_UPPER_LIMIT      (500.0)

// ***********************************
// Position control
#define IO_X_KP             (-0.014)
#define IO_X_OMEGA_TI       (0.6)
#define IO_X_OMEGA_LEAD_1   (0.1)
#define IO_X_OMEGA_LAG_1    (40.0)
#define IO_X_OMEGA_LEAD_2   (0.6)
#define IO_X_OMEGA_LAG_2    (40.0)
#define IO_X_LPF_WN         (50.0)

#define IO_Y_KP             (0.015)
#define IO_Y_OMEGA_TI       (0.6)
#define IO_Y_OMEGA_LEAD_1   (0.1)
#define IO_Y_OMEGA_LAG_1    (40.0)
#define IO_Y_OMEGA_LEAD_2   (0.65)
#define IO_Y_OMEGA_LAG_2    (40.0)
#define IO_Y_LPF_WN         (50.0)

#define IO_TRANSLATION_WN   (1.0)

#define IO_X_LOWER_LIMIT    (-1.0)
#define IO_X_UPPER_LIMIT    (1.0)
#define IO_Y_LOWER_LIMIT    (-1.0)
#define IO_Y_UPPER_LIMIT    (1.0)

// ***********************************

// **********************************
// Heave channel control parameters
// ***********************************
// Heave yaw decouple matrix
#define IO_HEAVE_YAW_DECOUPLER_11   (1.0)
#define IO_HEAVE_YAW_DECOUPLER_12   (0.0)
#define IO_HEAVE_YAW_DECOUPLER_21   (1.2)
#define IO_HEAVE_YAW_DECOUPLER_22   (1.0)
#define IO_HEAVE_KP          (-75.0)
#define IO_HEAVE_OMEGA_LEAD  (1.0)
#define IO_HEAVE_OMEGA_LAG   (40.0)
#define IO_HEAVE_OMEGA_TI    (1.3)
#define IO_NORMAL_HEAVE_UPPER_LIMIT (200.0)
#define IO_NORMAL_HEAVE_LOWER_LIMIT (0.0)
#define IO_TAKEOFF_HEAVE_UPPER_LIMIT (140.0)
#define IO_TAKEOFF_HEAVE_LOWER_LIMIT (0.0)
#define IO_ALTITUDE_LIMIT       (2.0)
#define IO_ALTITUDE_LPF_WN      (2.0)
#define IO_THROTTLE_ST1_MIN     (931.0)
#define IO_THROTTLE_ST1_MAX     (1320.0)
#define IO_ST1_MAX_HEIGHT       (0.3)
#define IO_ST1_DIV              (1297.0)  // (1320-931)/0.3
#define IO_THROTTLE_ST2_MAX     (1998.0)
#define IO_ST2_MAX_HEIGHT       IO_ALTITUDE_LIMIT
#define IO_ST2_DIV              (197.0)   // (1998-1320)/(2-0.3)
#define IO_LANDING_REF_THRESH   (0.28)
#define IO_COLLECTIVE_NOMINAL   (100.0)
#define IO_OL_CLIMB_RATE        (0.04)
#define IO_CL_DESCENT_RATE      (0.8e-3)        // 80mm/s
#define IO_COLLECTIVE_TAKEOFF_MAX   (136.0)
#define IO_HEAVE_DT_NOMINAL     (80e-3)
#define IO_COLLECTIVE_START     (115.0)
#define IO_COLLECTIVE_AUTOROT   (150.0)
#define IO_COLLECTIVE_LAND      (100.0)
#define IO_LANDING_HEIGHT       (0.3)
#define IO_COLLECTIVE_LAND_T0   (125.0)
#define IO_COL_UPPER_LIMIT      (500.0)
#define IO_COL_LOWER_LIMIT      (0)

// **********************************
// Yaw channel control parameters
// ***********************************
#define IO_YAW_KP          (300.0)
#define IO_YAW_OMEGA_LEAD  (2.0)
#define IO_YAW_OMEGA_LAG   (40.0)
#define IO_YAW_OMEGA_TI    (5.0)
#define IO_YAW_UPPER_LIMIT (420.0)
#define IO_YAW_LOWER_LIMIT (-420.0)
#define IO_YAW_LPF_WN        (5.0)
#define IO_YAW_WN          (15.0)
#define IO_YAW_ANGLE_LIMIT (2*M_PI)
#define IO_GYRO_GAIN       (52.0)

// Speed control parameters
// Input PWM 0-255, Output speed rad/s
// **********************************
//             (s/2.13 + 1)
// G(s) = 1.44 -----------      omega_gc = 4.12 rad/s
//                  s
// With H(s) = 1/(s/(2*pi*10 + 1) + 1) on sensor signal
#define IO_SPEED_KP          (1.5)
#define IO_SPEED_OMEGA_TI    (2.13)
#define IO_SPEED_UPPER_LIMIT (255.0)
#define IO_SPEED_LOWER_LIMIT (0.0)
#define IO_SPEED_WN_LPF      (60.0)
#define IO_SPEED_RATE2       (1500.0)     // Speed at which reference rate doubles
#define IO_THROTTLE_THRESH   (1100.0)
#define IO_FEEDFORWARD_SPEED (0.1)

void IO_setup(void){
    IO_SystemState = SYS_BOOT;

    // Setup direction for heartbeat LED
    IO_HEARTBEAT_TRIS();
    // Set PROGRAM switch as digital input
    IO_PROGRAM_SWITCH_TRIS();

    // Setup debug direction
    IO_DEBUG_TRIS();
    IO_DEBUG_LAT_LOW();

    // Enforce PWM pins as output
    mPORTDSetPinsDigitalOut(BIT_1|BIT_2|BIT_3);
    // Start up as low
    PORTClearBits(IOPORT_D,BIT_1|BIT_2|BIT_3);

    int i = 0;
    for (i = 0; i < 100000; i++);

    // Enable level shifter
    IO_enableLevelShifter1(true);
    IO_enableLevelShifter2(true);

    IO_getConfiguration();

    // Setup PWM
    IO_setupPWM();

    // Setup CT interrupt
    IO_setupTICK();

    // Setup ADC for battery voltage read
    IO_setupADC();

    IO_roll_ref = 0;
    IO_pitch_ref = 0;
    IO_yaw_ref = 0;

    IO_longitudinal = 0;
    IO_lateral = 0;
    IO_collective = 0;
    IO_esc = 0;
    IO_ref_speed = 0;
    IO_tailrate = 0;
    IO_x_ref = 0;
    IO_y_ref = 0;
}

void IO_getConfiguration(void){
   // Open file in read mode
   FSFILE * fd = FSfopen ("helicopter.conf", "r");
   if (fd == NULL)
      while(1);

   char string_temp[128] = {0};
   float itg3200_px1 = 0;
   float itg3200_px2 = 0;
   float itg3200_py1 = 0;
   float itg3200_py2 = 0;
   float itg3200_pz1 = 0;
   float itg3200_pz2 = 0;
   int parameters = 0;
   IO_getLine(fd, &string_temp[0]);
   parameters = sscanf(string_temp,"ITG3200_PX1=%f\n",&itg3200_px1);
   IO_getLine(fd, &string_temp[0]);
   parameters += sscanf(string_temp,"ITG3200_PX2=%f\n",&itg3200_px2);
   IO_getLine(fd, &string_temp[0]);
   parameters += sscanf(string_temp,"ITG3200_PY1=%f\n",&itg3200_py1);
   IO_getLine(fd, &string_temp[0]);
   parameters += sscanf(string_temp,"ITG3200_PY2=%f\n",&itg3200_py2);
   IO_getLine(fd, &string_temp[0]);
   parameters += sscanf(string_temp,"ITG3200_PZ1=%f\n",&itg3200_pz1);
   IO_getLine(fd, &string_temp[0]);
   parameters += sscanf(string_temp,"ITG3200_PZ2=%f\n",&itg3200_pz2);

   // if close file returns 1 or parameter number does not matched halt
   if (FSfclose(fd) || (parameters != CONFIG_PARAMETERS))
      while(1);

   // Update gyro temperature compensation parameters
   ITG3200_setCalibrationCoefficients(itg3200_px1, itg3200_px2,
        itg3200_py1, itg3200_py2, itg3200_pz1, itg3200_pz2);
}

int IO_getLine(FSFILE * fd, char * string){
    int numbytes = 0;

    if (FSfread (string, 1, 1, fd) != 1)
        while(1);
    numbytes++;

    while(*string != '\n'){
       string++;
       if (FSfread (string, 1, 1, fd) != 1)
          while(1);
       numbytes++;
    }
    string++;
    *string = '\0';
}

void IO_Control_Int_Enable(void){
    IO_gyro_gain = PWM_GAIN2BYTE(IO_GYRO_GAIN);

    // clear interrupt flag
    mT3ClearIntFlag();
    // setup timer with pre-scaler = 256 --> a resolution of 3.2e-6s
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, IO_CONTROL_PERIOD_MATCH);

    ConfigIntTimer3(T3_INT_ON | IO_T3_PRIORITY | IO_T3_SUBPRIORITY);

    // clear interrupt flag
    mT3ClearIntFlag();
}

void IO_getRadioNominal(void){
    // For simplicity, assume capture is working and can retrieve updated
    // pulsewidth atleast every 25ms
    long int IO_esc_tmp = 0, IO_right_tmp = 0, IO_rear_tmp = 0, IO_rudder_tmp = 0, IO_left_tmp = 0;
    int i = 0;
    int samples[5] = {0};

    // Wait here while radio not detected
    while(IO_RADIO_ERROR);
    IO_delayms(200);

    // try for 100 samples
    for (i = 0; i < 100; i++){
        if (IO_getESC()){
            // non-zero
            IO_esc_tmp += IO_getESC();
            samples[0]++;
        }
        if (IO_getRight()){
            // non-zero
            IO_right_tmp += IO_getRight();
            samples[1]++;
        }
        if (IO_getRear()){
            // non-zero
            IO_rear_tmp += IO_getRear();
            samples[2]++;
        }
        if (IO_getRudder()){
            // non-zero
            IO_rudder_tmp += IO_getRudder();
            samples[3]++;
        }
        if (IO_getLeft()){
            // non-zero
            IO_left_tmp += IO_getLeft();
            samples[4]++;
        }
        IO_delayms(25); // wait 25 ms
    }

    // find average
    IO_esc_nom = IO_esc_tmp / samples[0];
    IO_right_nom = IO_right_tmp / samples[1];
    IO_rear_nom = IO_rear_tmp / samples[2];
    IO_rudder_nom = IO_rudder_tmp / samples[3];
    IO_left_nom = IO_left_tmp / samples[4];

    // sanity check: all norms should be below 2000
    if ((IO_esc_nom > 2000) ||
        (IO_right_nom > 2000) ||
        (IO_rear_nom > 2000) ||
        (IO_rudder_nom > 2000) ||
        (IO_left_nom > 2000)){
        while(1);
        // do not proceed...something is wrong!
    }

}

void IO_changeNotificationSetup(void){
    // Set speed sensor pin and config change notice interrupt
    // D6,CN15: OPTO-ENCODER
    // D7,CN16: GYROGAIN [5]
    // D13,CN19: ULTRASONIC SENSOR PULSEWIDTH

    // PGEC1/AN1/CN3/RB1 (24): ESC
    // AN2/C2IN-/CN4/RB2 (23): RIGHT
    // AN3/C2IN+/CN5/RB3 (22): REAR
    // AN4/C1IN-/CN6/RB4 (21): RUDDER
    // AN5/C1IN+/V BUSON /CN7/RB5 (20): LEFT

    #if defined(SETUP_RADIO_DETECT)
        PORTSetPinsDigitalIn(IOPORT_D, BIT_6 | BIT_7 | BIT_13);
        PORTSetPinsDigitalIn(IOPORT_B, BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5);
    #else
        PORTSetPinsDigitalIn(IOPORT_D, BIT_6 | BIT_13);
    #endif

    mCNOpen(CONFIG, PINS, PULLUPS);
    ConfigIntCN(CHANGE_INT_ON | IO_CN_PRIORITY);
    INTSetVectorSubPriority(_CHANGE_NOTICE_VECTOR, IO_CN_SUBPRIORITY);
}

void IO_initialiseLogFiles(void){
    IO_initializeFile(&IO_log, IO_FS_LOG);

#if defined (DEBUG)
    IO_initializeFile(&IO_dmesg, IO_FS_DMESG);
#endif
}

void IO_initialize_FS(void){
    // Wait here until SD card responds with idle
    while(!MDD_SDSPI_MediaDetect());
    // Initialize file system
    while (!FSInit());
}

bool IO_initializeFile(IO_file * IO_file_ds, const char * IO_file_name){
    // setup debug file for writing
    memset(IO_file_ds,0,sizeof(IO_file));
    sprintf(&(IO_file_ds->file_name[0]), IO_file_name);
    IO_file_ds->fsFile = FSfopen(&(IO_file_ds->file_name[0]), "w");
    if (IO_file_ds->fsFile == NULL){
        IO_file_ds->error = OPEN_ERROR;
        IO_file_ds->file_opened = false;
        return false;
    }
    else
    {
        IO_file_ds->file_opened = true;
        IO_file_ds->write_length = 0;
        return true;
    }
}

void IO_terminate_FS(void){
    // Flush any remaining data before closing files
    IO_flush_FS();  // Flush overflow

    if (IO_log.file_opened){
        if (FSfwrite (IO_log.data_buffer, 1, IO_log.write_length, IO_log.fsFile)){
            IO_log.error = WRITE_ERROR;
        }
    }
    if (IO_dmesg.file_opened){
        if (FSfwrite (IO_dmesg.data_buffer, 1, IO_dmesg.write_length, IO_dmesg.fsFile)){
            IO_dmesg.error = WRITE_ERROR;
        }
    }

    if (IO_log.file_opened){
        if(FSfclose(IO_log.fsFile))
            IO_log.error = CLOSE_ERROR;
        else
            IO_log.file_opened = false;
    }

    if (IO_dmesg.file_opened){
        if(FSfclose(IO_dmesg.fsFile))
            IO_dmesg.error = CLOSE_ERROR;
        else
            IO_dmesg.file_opened = false;
    }
}

bool IO_write_FSwInt(IO_file * io_file, char * data, UINT16 len){
    if (io_file->file_opened){
        // if currently being flushed then write to interrupt buffer
        if (io_file->flush_active){
            UINT16 buffer_remainder = SD_WRITE_BUFFER_SIZE - io_file->int_write_length;
            if (buffer_remainder > len){
                memcpy(&(io_file->interrupt_buffer[io_file->int_write_length]), data, len);
                io_file->int_write_length += len;
                return true;
            }
            else{
                // if delayed data cannot be stored in interrupt buffer then do not bother
                // to store i.e. discard
                return false;
            }
        }

        // if interrupt_buffer was written during flush move data to main buffer
        if (io_file->int_write_length > 0){
            IO_write_FS(io_file, &(io_file->interrupt_buffer[0]), io_file->int_write_length);
            io_file->int_write_length = 0;
        }
        // Then fill in current data
        IO_write_FS(io_file, data, len);
    }
}

bool IO_write_FS(IO_file * io_file, char * data, UINT16 len){
    // this function writes data to the fd buffer
    // if the fd buffer overflows then it copies the balance into the overflow buffer
    // and signals buffer overflow--so that flush_FS can be called with full 512 buffer
    // assume len is greater than 512
    if (io_file->file_opened){
        if (!(io_file->buffer_overflow)){
            // if more data can be written to main buffer--> add to buffer
            UINT16 buffer_remainder = SD_WRITE_BUFFER_SIZE - io_file->write_length;
            if (buffer_remainder > len){
                memcpy(&(io_file->data_buffer[io_file->write_length]), data, len);
                io_file->write_length += len;
            }
            else
            {
                // first fill in whatever fits in main buffer
                memcpy(&(io_file->data_buffer[io_file->write_length]), data, buffer_remainder);
                io_file->write_length += buffer_remainder;

                // now write remainer into overflow buffer
                memcpy(&(io_file->overflow_buffer[0]), data + buffer_remainder, len - buffer_remainder);
                io_file->overflow_length = len - buffer_remainder;
                io_file->buffer_overflow = true;
            }
        }
        else{
            // if in here-means that overflow buffer has been written to and has not been flushed
            memcpy(&(io_file->overflow_buffer[io_file->overflow_length]), data, len);
            io_file->overflow_length += len;
        }

        return true;
    }
    else
        return false;
}

bool IO_flush_file(IO_file * io_file){
    io_file->flush_active = true;
    // only if main buffer is full (512), do a write
    if (io_file->buffer_overflow && io_file->file_opened){
        if (FSfwrite (io_file->data_buffer, 1, io_file->write_length, io_file->fsFile) != io_file->write_length){
            io_file->error = WRITE_ERROR;
            io_file->file_opened = false;
            io_file->flush_active = false;
            return false;
        }
        else{
            io_file->buffer_overflow = false;
            io_file->write_length = 0;
            // now move data from overflow to main buffer
            memcpy(&(io_file->data_buffer[0]), &(io_file->overflow_buffer[0]), io_file->overflow_length);
            io_file->write_length = io_file->overflow_length;
            io_file->overflow_length = 0;
            io_file->flush_active = false;
            return true;
        }
    }
    io_file->flush_active = false;
}

bool IO_flush_FS(void){
    int ret = 0;
    ret = IO_flush_file(&IO_log);
    ret = IO_flush_file(&IO_dmesg);
    return ret;
}

void IO_logMessage(char * str, UINT16 len){
    IO_write_FSwInt(&IO_log, str, len);
}

void IO_dmesgMessage(char * str, UINT16 len){
    IO_write_FSwInt(&IO_dmesg, str, len);
}

bool IO_getCloseFiles(void){
    return IO_closeFiles;
}

void IO_setCloseFiles(bool val){
    IO_closeFiles = val;
}

void IO_enableLevelShifter1(BOOL val)
{
    IO_LEVELSHIFT1TRIS();
    if (val)
	PORTSetBits(IOPORT_A, BIT_4);
    else
	PORTClearBits(IOPORT_A, BIT_4);
}

void IO_enableLevelShifter2(BOOL val)
{
    IO_LEVELSHIFT2TRIS();
    if (val)
	PORTSetBits(IOPORT_A, BIT_5);
    else
	PORTClearBits(IOPORT_A, BIT_5);
}

void IO_setupPWM(void){
    // Enforce PWM pins as output
    mPORTDSetPinsDigitalOut(BIT_1|BIT_2|BIT_3);//|BIT_4);
    // Start up as low
    PORTClearBits(IOPORT_D,BIT_1|BIT_2|BIT_3);//|BIT_4);

    OpenOC2( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC3( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC4( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    OpenOC5( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);

    // Timer 2 drives PWM module
    OpenTimer2( T2_ON | T2_PS_1_32 | T2_SOURCE_INT, IO_PWM_PERIOD);

    // Put out PWM but LED's are off!
    IO_setupPWM_default();
}

void IO_setupPWM_default(void){
    IO_LED_ON = false;
    SetDCOC2PWM(IO_PWM_dc(0.0));
    SetDCOC3PWM(IO_PWM_dc(0.0));
    SetDCOC4PWM(IO_PWM_dc(0.0));
}

UINT16 IO_PWM_dc(float dc){
    if (dc > 100.0)
        return IO_PWM_PERIOD;

    if (dc < 0.0)
        return 0;

    return (UINT16)floor(IO_PWM_PERIOD*dc/100);
}

void IO_LED_init(int toggle_number){
    int i = 0;

    for (i = 0; i < toggle_number; i++){
        IO_leds_on(80.0);
        IO_delayms(200);

        IO_setupPWM_default();
        IO_delayms(200);
    }

    IO_delayms(1000);
}

void IO_leds_on(float power){
    IO_LED_ON = true;
    SetDCOC2PWM(IO_PWM_dc(power));
    SetDCOC3PWM(IO_PWM_dc(power));
    SetDCOC4PWM(IO_PWM_dc(power));
}

bool IO_getLEDState(void){
    return IO_LED_ON;
}

void IO_toggle_beacon(void){
    if (IO_LED_ON)
        IO_setupPWM_default();
    else
        IO_leds_on(50);
}

void IO_setupTICK(void){
    // Open up the core timer at 1ms rate
    OpenCoreTimer(CORE_TICK_RATE);
    // Set up the core timer interrupt with a prioirty of 7 and sub-priority 2
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_7 | CT_INT_SUB_PRIOR_2));
}

void IO_setSystemState(IO_SYSTEM_STATE state){
    IO_SystemState = state;
}

void IO_setLandState(IO_LAND_STATE state){
    IO_LandState = state;
}

IO_SYSTEM_STATE IO_getSystemState(void){
    return IO_SystemState;
}

void IO_delayms(unsigned int num_ms_delay)
{
    if (num_ms_delay < 200)
    {
	IO_delayUnder200ms(num_ms_delay);
    }
    else
    {
	// if the delay is greater than 200ms
	unsigned int remainder = num_ms_delay;
	while(remainder > 200)
	{
            IO_delayUnder200ms(100);
            remainder -= 100;
	}
	if (remainder > 0)
            IO_delayUnder200ms(remainder);
    }
}

void IO_delayUnder200ms(unsigned int val)
{
    // calculate required period
    unsigned short int temp_period = (unsigned short int)((double)val*80e3/256);
    // clear interrupt flag
    mT1ClearIntFlag();
    // setup timer with pre-scaler = 256 --> a resolution of 3.2e-6s
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, temp_period);
    // wait here until period match
    while(!mT1GetIntFlag());
    // clear interrupt flag
    mT1ClearIntFlag();
    CloseTimer1();
}

void IO_setupADC(void)
{
    // make AD0 an analog pin
    AD1PCFG = 0xFFFE;
    AD1CHS = 0x0000;
    AD1CON1 = 0x00E0;
    AD1CSSL = 0x0001;
    AD1CON2 = 0x6000;
    AD1CON3 = 0x1FFF;

    /* Set up interrupts */
    ConfigIntADC10(IO_ADC_PRIORITY | IO_ADC_SUBPRIORITY |ADC_INT_ON);
    AD1CON1 |= 0x8000;					/* Turn ADC module on */
}

volatile UINT32 IO_get_time_ms(void){
    return IO_time_ms;
}

void IO_clrDataTime(void){
    IO_DATA_TIME = 0;
}

UINT32 IO_getDataTime(void){
    return IO_DATA_TIME;
}

UINT16 IO_getBatteryVoltage(void){
    return IO_batteryVoltage;
}

bool IO_getLocalControl(void){
    return IO_localControl;
}

void IO_setLocalControl(bool val){
    IO_localControl = val;
}

bool IO_getStateProject(void){
    return IO_stateProject;
}

void IO_setStateProject(bool val){
    IO_stateProject = val;
}

bool IO_getSDFlush(void){
    return IO_sdFlush;
}

void IO_setSDFlush(bool val){
    IO_sdFlush = val;
}

bool IO_getRadioError(void){
    return IO_RADIO_ERROR;
}

float IO_getGyroGain_dt(void){
    return (float) IO_gyrogain_dt / (40e3);
}

float IO_getUltrasonic_dt(void){
    return (float) IO_ultrasonic_dt / (40e6);
}

float IO_getAltitude(void){
    static float previous_altitude = 0;

    // 232e3 = 40e6*5.8...e-3(s/m)
    // 16cm -> 928e-6s -> 37120
    // 765cm -> 44.37e-3 -> 1774800

    float current_altitude = (float)IO_ultrasonic_dt/(232000);
    // If there is a sudden change in altitude signal greater than 0.7 meter
    // assume last calculation is the best estimate
    if (fabs(current_altitude - previous_altitude) < 0.7){
        previous_altitude = current_altitude;
        return current_altitude;
    }
    else
        return previous_altitude;
}

unsigned short int IO_getRotorSpeed(void){
    static float previous_speed = 0;

    // angular_speed    1 rev     60 rev       6 rpm
    //               = ------- = ---------- = ------ , where dt is the period NOT mark time
    //                 dt*10 s    dt*10 min    dt

    if (IO_rotor_speed_dt == 0)
        return 0;

    if (!IO_rotor_speed_timeout){
        float rotor_speed_temp = ((float)240e6)/((float)IO_rotor_speed_dt);

        // This is to prevent outliers from passing through...best estimate is last valid measurement
        // If the difference in speed between current measurement and last is less than 600 rpm accept it
        if (fabs(rotor_speed_temp - previous_speed) < 600){
            IO_rotor_speed = rotor_speed_temp;
            previous_speed = rotor_speed_temp;
        }
        else{
            IO_rotor_speed = previous_speed;
        }

        return (unsigned short int) floor(IO_rotor_speed + 0.5);
    }
    else{
        // if there is a timeout and the previous speed is less than 1000 rpm then reset speed to 0
        if (previous_speed < 500){
            previous_speed = 0;
            IO_rotor_speed = 0;
            return 0;
        }
    }

    // if there is a timeout and the speed is greater than 500rpm then best bet
    // for current speed is previous speed
    return (unsigned short int) floor(previous_speed + 0.5);
}

float IO_getFilteredRotorSpeed(void){
    return IO_filteredRotorSpeed;
}

bool IO_timeoutSpeed(void){
    if (IO_rotor_speed_update++ > IO_SPEED_TIMEOUT){
        IO_rotor_speed_timeout = true;
        return true;
    }
    else
    {
        IO_rotor_speed_timeout = false;
        return false;
    }
}

bool IO_getSpeedTimeout(void){
    return IO_rotor_speed_timeout;
}

UINT16 IO_getGyroGain(void){
    return (UINT16) (IO_gyrogain_dt / (40));
}

UINT16 IO_getUltrasonic(void){
    return (UINT16) (IO_ultrasonic_dt / (40));
}

UINT16 IO_getESC(void){
    return (UINT16) (IO_esc_dt / (40));
}

UINT16 IO_getRight(void){
    return (UINT16) (IO_right_dt / (40));
}

UINT16 IO_getRear(void){
    return (UINT16) (IO_rear_dt / (40));
}

UINT16 IO_getRudder(void){
    return (UINT16) (IO_rudder_dt / (40));
}

UINT16 IO_getLeft(void){
    return (UINT16) (IO_left_dt / (40));
}

void IO_sendData(UINT32 time_us){
    UINT16 data_pack[25] = {0};
    ITG3200_readData();
    ADXL345_readData();
    data_pack[0] = ITG3200_getx();
    data_pack[1] = ITG3200_gety();
    data_pack[2] = ITG3200_getz();
    data_pack[3] = ADXL345_getx();
    data_pack[4] = ADXL345_gety();
    data_pack[5] = ADXL345_getz();
    data_pack[6] = (UINT16)(IO_getAltitude()*1000);
    data_pack[7] = IO_getBatteryVoltage();
    data_pack[8] = IMU_getRoll16BIT();
    data_pack[9] = IMU_getPitch16BIT();
    data_pack[10] = IMU_getYaw16BIT();
    data_pack[11] = RN131_getTx();
    data_pack[12] = RN131_getTy();
    data_pack[13] = RN131_getTz();
    data_pack[14] = ((UINT16)IO_PWM_packet[0] << 8) | IO_PWM_packet[1];
    data_pack[15] = ((UINT16)IO_PWM_packet[2] << 8) | IO_PWM_packet[3];
    data_pack[16] = ((UINT16)IO_PWM_packet[4] << 8) | IO_PWM_packet[5];
    data_pack[17] = IO_getRotorSpeed();
    data_pack[18] = (UINT16) IO_statusReg;
    data_pack[19] = ITG3200_getRawTemperature();
    data_pack[20] = RN131_getIncrKey();
    data_pack[21] = (UINT16)(0x0000FFFF & time_us);
    data_pack[22] = (UINT16)((0xFFFF0000 & time_us) >> 16);
    RN131_SendDataPacket(data_pack, 23);
}

void IO_control_exec(void){
    static UINT32 prev_timestamp = 0;

    UINT32 current_time_us = IO_updateCoreTime_us();

    IO_sendData(current_time_us);

    int temp_time = ReadCoreTimer();
    float control_dt = (float)((unsigned int)((unsigned int) temp_time - (int) prev_timestamp))/40e6;
    if (control_dt > 1000e-3){
        control_dt = 10e-3;
    }
    prev_timestamp = temp_time;

    IMU_propagateState(control_dt);
    IO_main_control(control_dt);
}

UINT32 stable_ekf = 0;
bool position_control = false;
bool rn131_timeout = true;
void IO_main_control(float dt){
    // Static variables
    static float internal_collective = IO_COLLECTIVE_START;
    static UINT32 IO_heave_prev_timestamp = 0;
    static float internal_reference_speed = 0.0;
    static float collective_sig = IO_COLLECTIVE_START;
    static float altitude_ref_forced_landing = IO_LANDING_HEIGHT;

    // Dynamic variables
    float throttle = 0, delta_right = 0, delta_rear, delta_left = 0;
    float yaw_angle_reference = 0, altitude_reference = 0;

    // If manual mode is activated, override system state to manual
    if (!SPEKTRUM_isAutoMode()){
        /********************* MODE SWITCH **************************/
       IO_setSystemState(SYS_MANUAL);
       IO_setupPWM_default();
    }
    else{
       IO_leds_on(70.0);
    }

    // Get inputs
    IO_getRefInputs(&throttle, &delta_right, &delta_rear, &delta_left,
        &altitude_reference, &yaw_angle_reference, &IO_roll_ref, &IO_pitch_ref,
            &IO_x_ref, &IO_y_ref);

    float collective_raw = 0, lateral_raw = 0, longitudinal_raw = 0;
    IO_getRotorInputs(&collective_raw, &lateral_raw, &longitudinal_raw);

    /* Filtering operations */
    // Filter altitude reference
    float altitude_reference_filtered = IO_filter_altitude_prefilter(altitude_reference, dt, IO_ALTITUDE_LPF_WN);
    altitude_reference_filtered = IO_limits(0, IO_ALTITUDE_LIMIT, altitude_reference_filtered);
    IO_z_ref = altitude_reference_filtered;
    // Filter yaw reference
    float yaw_ref_filtered = IO_filter_yaw_prefilter(yaw_angle_reference, dt, IO_YAW_LPF_WN);
    yaw_ref_filtered = IO_limits(-IO_YAW_ANGLE_LIMIT, IO_YAW_ANGLE_LIMIT, yaw_ref_filtered);
    IO_yaw_ref = yaw_ref_filtered;
    // Filter speed; updates IO_filteredRotorSpeed
    IO_filter_speed((float)IO_getRotorSpeed(), dt, IO_SPEED_WN_LPF);
    // Filter translation reference
    IO_filter_translationref_prefilter(&IO_x_ref, &IO_y_ref, dt, IO_TRANSLATION_WN);

//       -------> MANUAL <--------
//       |          /|\          |
//       |           |           |
//      \|/          |           |
//    TAKEOFF ----> FLY ------> LAND
//      /|\                      |
//       -------------------------

    // Detemine if valid data had been received through wifi
    rn131_timeout = RN131_getTimeout();
    // every 10ms if stable and no timeout we increment stable_ekf
    if ((RN131_ekfStable()) && !rn131_timeout)
        stable_ekf++;
    else
        stable_ekf = 0;

    // If ekf covariance is under the limit for at least 5s and there is no timeout, allow for position control
    position_control = (stable_ekf > 500) ? true : false;

    // Perfrom control execution based on system state
    switch(IO_SystemState){
        case SYS_BOOT:
        case SYS_ERROR:
        case SYS_MANUAL:
            // Reset variables so that reentering auto mode will act predictably
            internal_reference_speed = 0;
            internal_collective = IO_COLLECTIVE_AUTOROT;

            /********************* MODE SWITCH **************************/
            if ((throttle > IO_THROTTLE_THRESH) && (SPEKTRUM_isAutoMode())){
                internal_collective = IO_COLLECTIVE_START;
                IO_setSystemState(SYS_TAKEOFF);
            }

            IO_ref_speed = internal_reference_speed;

            break;

        case SYS_TAKEOFF:
            // Collective should remain at IO_COLLECTIVE_START during run up
            // Speed reference generation
            if (throttle > IO_THROTTLE_THRESH){
                internal_reference_speed = internal_reference_speed + 2;    // 100rpm increase per second
                // double rate of increase if speed is greater than 1500 rpm
                if (internal_reference_speed > IO_SPEED_RATE2)
                    internal_reference_speed = internal_reference_speed + 3;    // incl. above --> 300 rpm incr/s
            }
            else{
                // If user has throttle below threshold for any reason reset reference speed
                internal_reference_speed = 0;
            }

            IO_ref_speed = internal_reference_speed;
            // Limit controllable reference
            if (IO_ref_speed < IO_RPM_MIN)
                IO_ref_speed = 0;
            if (IO_ref_speed > IO_RPM_MAX)
                IO_ref_speed = internal_reference_speed = IO_RPM_MAX;

            // If reference has reached peak
            if (IO_ref_speed >= IO_RPM_MAX){
                // runs every 0.01s
                internal_collective = internal_collective + IO_OL_CLIMB_RATE;
                /********************* MODE SWITCH **************************/
                if (internal_collective >= IO_COLLECTIVE_TAKEOFF_MAX)
                    IO_setSystemState(SYS_FLY);

                collective_sig = internal_collective;
                collective_sig = IO_limits(IO_COLLECTIVE_NOMINAL, IO_COLLECTIVE_TAKEOFF_MAX, collective_sig);
            }
            else{
                // Not enough to get off the ground NB. Make sure not negative pitch
                collective_sig = IO_COLLECTIVE_START;
            }

            break;

        case SYS_FLY:
            // only change plant input if there is an update in the sensor reading
            if (ultrasonic_updated){
                float IO_heave_dt = (IO_get_time_ms() - IO_heave_prev_timestamp)*1e-3;
                // Set dt on initialisation because IO_heave_prev_timestamp is wrong for first run
                if (IO_heave_prev_timestamp == 0)
                    IO_heave_dt = IO_HEAVE_DT_NOMINAL;
                IO_heave_prev_timestamp = IO_get_time_ms();

                collective_sig = IO_heave_control(altitude_reference_filtered, IO_getAltitude(), -IO_COLLECTIVE_TAKEOFF_MAX, 60, IO_heave_dt, false) + IO_COLLECTIVE_TAKEOFF_MAX;
                ultrasonic_updated = false;
            }

//            /********************/
//            /* Position control */
//            /********************/
//            // IO_roll_ref and IO_pitch_ref are modified here
//            IO_position_control(IO_x_ref, IO_y_ref, &IO_roll_ref, &IO_pitch_ref, dt, false);

            /********************* MODE SWITCH **************************/
            if (altitude_reference <= IO_LANDING_REF_THRESH){
                IO_setSystemState(SYS_LAND);
                // If manual land then it is a normal landing
                IO_setLandState(LAND_NORMAL);
                internal_collective = IO_COLLECTIVE_LAND_T0;
            }

            // If in fly mode and position control is false, initiate forced landing
            if (!position_control){
                IO_setSystemState(SYS_LAND);
                // If manual land then it is a normal landing
                IO_setLandState(LAND_FORCED);
                // Initialise attitude to descend from
                altitude_ref_forced_landing = altitude_reference_filtered;
            }

            break;

        case SYS_LAND:
            switch (IO_LandState){
                case LAND_NORMAL:
                    // runs every 0.01s
                    internal_collective = internal_collective - 0.02;

                    // Reduce speed as well
                    internal_reference_speed = internal_reference_speed - 1;
                    IO_ref_speed = internal_reference_speed;
                    // Limit controllable reference
                    if (IO_ref_speed < IO_RPM_MIN)
                        IO_ref_speed = internal_reference_speed = 0;
                    if (IO_ref_speed > IO_RPM_MAX)
                        IO_ref_speed = IO_RPM_MAX;

                    // Implies that in x seconds internal collective would decrease to IO_COLLECTIVE_LAND
                    /********************* MODE SWITCH **************************/
                    // Only switch to takeoff mode if collective has reduced to autorotation value and
                    // pilot has killed throttle
                    if ((internal_collective <= IO_COLLECTIVE_LAND) && (throttle < IO_THROTTLE_THRESH)){
                        IO_setSystemState(SYS_TAKEOFF);
                    }

                    collective_sig = internal_collective;
                    collective_sig = IO_limits(IO_COLLECTIVE_LAND, IO_COLLECTIVE_TAKEOFF_MAX, collective_sig);
                    break;

                case LAND_FORCED:
                default:
                    // Controlled descent to IO_LANDING_HEIGHT @ -8cm/second
                    // State that called forced landing must set altitude_ref_forced_landing to last reference altitude
                    altitude_ref_forced_landing = altitude_ref_forced_landing - IO_CL_DESCENT_RATE;   // drop of 8cm per second
                    if (altitude_ref_forced_landing > IO_LANDING_HEIGHT){
                        // only change plant input if there is an update in the sensor reading
                        if (ultrasonic_updated){
                            float IO_heave_dt = (IO_get_time_ms() - IO_heave_prev_timestamp)*1e-3;
                            // Set dt on initialisation because IO_heave_prev_timestamp is wrong for first run
                            if (IO_heave_prev_timestamp == 0)
                                IO_heave_dt = IO_HEAVE_DT_NOMINAL;
                            IO_heave_prev_timestamp = IO_get_time_ms();

                            collective_sig = IO_heave_control(altitude_ref_forced_landing, IO_getAltitude(), -IO_COLLECTIVE_TAKEOFF_MAX, 60, IO_heave_dt, false) + IO_COLLECTIVE_TAKEOFF_MAX;
                            ultrasonic_updated = false;
                        }
                    }
                    else{
                        // After "ref" LANDING height has been reached proceed with normal landing
                        IO_setSystemState(SYS_LAND);    // Redundant
                        IO_setLandState(LAND_NORMAL);
                        internal_collective = IO_COLLECTIVE_LAND_T0;
                    }

                    break;
            }
            break;
        default:
            break;
    }

    /********************* MODE SWITCH **************************/
    if (throttle < IO_THROTTLE_THRESH){
        IO_ref_speed = 0;
        internal_reference_speed = 0;
        collective_sig = IO_COLLECTIVE_AUTOROT;
        IO_setSystemState(SYS_MANUAL);
    }

    // Update IO_esc control signal
    IO_esc = IO_ESC_controller(IO_ref_speed, dt);
    // Update IO_tailrate
    //IO_tailrate = (float)IO_getRudder() - (float)SPEKTRUM_getRudderNominal();    /* Manual control */
    IO_tailrate = IO_yaw_control(dt, yaw_ref_filtered); /* Yaw hold */
    // Heave control only operational in SYS_FLY mode but is generated for open-loop SYS_TAKEOFF AND SYS_LAND
    IO_lateral = lateral_raw;
    IO_longitudinal = longitudinal_raw;
    IO_collective = collective_sig;

//            /********************/
//            /* Position control */
//            /********************/
//            // IO_roll_ref and IO_pitch_ref are modified here
//            IO_position_control(IO_x_ref, IO_y_ref, IO_z_ref, &IO_roll_ref, &IO_pitch_ref, dt, false);

    /*****************/
    /* Atitude control/
    /*****************/
    // Add roll and pitch input bias
    IO_roll_ref += IO_ROLL_BIAS;
    IO_pitch_ref += IO_PITCH_BIAS;
    // Limit roll and pitch angles
    IO_roll_ref = IO_limits(IO_ROLL_ANGLE_LOWER_LIMIT, IO_ROLL_ANGLE_UPPER_LIMIT, IO_roll_ref);
    IO_pitch_ref = IO_limits(IO_PITCH_ANGLE_LOWER_LIMIT, IO_PITCH_ANGLE_UPPER_LIMIT, IO_pitch_ref);
    IO_lateral = IO_roll_control(IO_roll_ref, dt, false) + IO_LATERAL_BIAS;
    IO_lateral = IO_limits(IO_LATERAL_LOWER_LIMIT, IO_LATERAL_UPPER_LIMIT, IO_lateral);
    IO_longitudinal = IO_pitch_control(IO_pitch_ref, dt, false) + IO_LONGITUDINAL_BIAS;
    IO_longitudinal = IO_limits(IO_LONGITUDINAL_LOWER_LIMIT, IO_LONGITUDINAL_UPPER_LIMIT, IO_longitudinal);
    /*****************/

    // Decouple heave yaw dynamics via precompensation
    // IO_HeaveYaw_decoupler(IO_collective, IO_tailrate);
    IO_esc += IO_ESC_feedforward(IO_collective);
    // Supply input to plant
    IO_writePWMmodule(IO_esc, IO_lateral, IO_longitudinal, IO_collective, IO_tailrate);
    // Vital logging
    IO_logControl();
}

void IO_getRefInputs(float * throttle, float * del_right, float * del_rear, float * del_left,
        float * alt_ref, float * yaw_ref, volatile float * roll_ref, volatile float * pitch_ref,
        volatile float * x_ref, volatile float * y_ref){

    *throttle = (float)IO_getESC();

    *del_right = (int)SPEKTRUM_getRIGHT() - SPEKTRUM_getRightNominal();
    *del_rear = -((int)SPEKTRUM_getREAR() - SPEKTRUM_getRearNominal());
    *del_left = -((int)SPEKTRUM_getLEFT() - SPEKTRUM_getLeftNominal());

    *alt_ref = IO_getAltitudeReference((float)SPEKTRUM_getESC());

    *yaw_ref = -(float)(SPEKTRUM_getRUDDER()-SPEKTRUM_getRudderNominal());
    if (*yaw_ref < 0)
        *yaw_ref = (float)2*M_PI/IO_YAW_UPPER_LIMIT*(*yaw_ref);
    else
        *yaw_ref = (float)2*M_PI/IO_YAW_UPPER_LIMIT*(*yaw_ref);

    // Get roll and pitch reference in degrees
    float lat = -(float)((*del_right)-(*del_left));
    float lon = (float)(((*del_right)+(*del_left))/2-(*del_rear));

    *roll_ref = lat*IO_ROLL_ANGLE_UPPER_LIMIT/SPEKTRUM_DELTA_LAT_MAX;
    *pitch_ref = lon*IO_PITCH_ANGLE_UPPER_LIMIT/SPEKTRUM_DELTA_LONG_MAX;
    // Put limit on angle range in degrees
    *roll_ref = IO_limits(IO_ROLL_ANGLE_LOWER_LIMIT, IO_ROLL_ANGLE_UPPER_LIMIT, *roll_ref);
    *pitch_ref = IO_limits(IO_PITCH_ANGLE_LOWER_LIMIT, IO_PITCH_ANGLE_UPPER_LIMIT, *pitch_ref);

    // Get position reference in meters
    *x_ref = -lon*IO_X_UPPER_LIMIT/SPEKTRUM_DELTA_LONG_MAX;
    *y_ref = -lat*IO_Y_UPPER_LIMIT/SPEKTRUM_DELTA_LAT_MAX;
    // Put limit on range
    *x_ref = IO_limits(IO_X_LOWER_LIMIT, IO_X_UPPER_LIMIT, *x_ref);
    *y_ref = IO_limits(IO_Y_LOWER_LIMIT, IO_Y_UPPER_LIMIT, *y_ref);
}

void IO_getRotorInputs(float * main_collective, float * lat, float * lon){
    /*
     * Servo to plant input mapping
     * ----------------------------
     * S = [dSrt dSrr dSlt]'
     * U = [theta A B]
     * U = A*S
     *       1/3   1/3   1/3
     * A =     1     0    -1
              -1     2    -1
     */

    int del_right = IO_SERVO_RIGHT_SIGN*(int)IO_getRight() - SPEKTRUM_getRightNominal();
    int del_rear = IO_SERVO_REAR_SIGN*((int)IO_getRear() - SPEKTRUM_getRearNominal());
    int del_left = IO_SERVO_LEFT_SIGN*((int)IO_getLeft() - SPEKTRUM_getLeftNominal());
    *main_collective = (float)(del_right + del_left + del_rear)/3;
    *lat = (float)(del_right - del_left);
    *lon = (float)(2*del_rear - (del_right + del_left));

    *main_collective = IO_limits(IO_COL_LOWER_LIMIT, IO_COL_UPPER_LIMIT, *main_collective); // should never be more than 500us
    *lat = IO_limits(IO_LAT_LOWER_LIMIT, IO_LAT_UPPER_LIMIT, *lat); // should never be more than 500us
    *lon = IO_limits(IO_LON_LOWER_LIMIT, IO_LON_UPPER_LIMIT, *lon); // should never be more than 500us
}

bool IO_writePWMmodule(float esc, float lateral, float longitudinal, float collective, float delta_rudder){
    /*
     * Plant to servo input mapping
     * ----------------------------
     * S = [dSrt dSrr dSlt]'
     * U = [theta A B]
     * U = A*S
     *       1/3   1/3   1/3
     * A =     1     0    -1
     *        -1     2    -1
     *
     *            1   1/2  -1/6
     * A^-1 =     1     0   1/3
     *            1  -1/2  -1/6
     *
     * Hard saturation limits
     * Collective = +-500
     * Lateral    = +- 1000
     * Longitudinal = +- 2000
     */

    float delta_right = collective + lateral/2 - longitudinal/6;
    float delta_rear = collective + longitudinal/3;
    float delta_left = collective - lateral/2 - longitudinal/6 ;

    IO_PWM_packet[0] = (UINT8) IO_limits(IO_PWM_SAT_MIN, IO_PWM_SAT_MAX, esc);
    IO_PWM_packet[1] = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getRightNominal() + IO_SERVO_RIGHT_SIGN*delta_right));
    IO_PWM_packet[2] = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getRearNominal() + IO_SERVO_REAR_SIGN*delta_rear));
    IO_PWM_packet[3] = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getRudderNominal() + delta_rudder));
    IO_PWM_packet[4] = IO_gyro_gain;
    IO_PWM_packet[5] = PWM_PULSEWIDTH2BYTE((int)(SPEKTRUM_getLeftNominal() + IO_SERVO_LEFT_SIGN*delta_left));

    // Check if require to send with config mode auto
    bool auto_mode_on = SPEKTRUM_isAutoMode();
    // Reset status register
    if (auto_mode_on)
        IO_statusReg = IO_AUTO_MODE;
    else
        IO_statusReg = 0x00;

    // Make sure sync is complete as well
    auto_mode_on &= RN131_syncComplete();

    // update sync status
    if (RN131_syncComplete())
        IO_statusReg |= IO_RN131_SYNC;
    else
        IO_statusReg &= ~IO_RN131_SYNC;

    // update timeout status
    if (RN131_getTimeout())
        IO_statusReg |= IO_RN131_TIMEOUT;
    else
        IO_statusReg &= ~IO_RN131_TIMEOUT;

    // update position control status
    if (position_control)
        IO_statusReg |= IO_POSITION_CNTL;
    else
        IO_statusReg &= ~IO_POSITION_CNTL;

    if (RN131_ekfStable())
        IO_statusReg |= IO_STABLE_EKF;
    else
        IO_statusReg &= ~IO_STABLE_EKF;

    if (SPEKTRUM_getTimeout())
        IO_statusReg |= IO_SPEKTRUM_TIMEOUT;
    else
        IO_statusReg &= ~IO_SPEKTRUM_TIMEOUT;

    if (IO_getSpeedTimeout())
        IO_statusReg |= IO_SPEED_SENSOR_TIMEOUT;
    else
        IO_statusReg &= ~IO_SPEED_SENSOR_TIMEOUT;
    
    // Compute checksum and send via I2C
    // Time from write to PWM change with one shot firmware ~660us
    bool pwm_ack = PWM_sendPacket(auto_mode_on, IO_PWM_packet, 6);

    if (pwm_ack)
        IO_statusReg |= IO_PWM_ACK;
    else
        IO_statusReg &= IO_PWM_ACK;

    return pwm_ack;
}

void IO_HeaveYaw_decoupler(float collective_input, float yawrate_input){
    /*
                      |-------------|
    G_HEAVE_OUT ----> |             | -----> COLLECTIVE_INPUT
                      | K decouple  |
    G_YAWRATE_OUT --> |             | -----> YAWRATE_INPUT
                      |-------------|

     COLLECTIVE_INPUT = K11*G_HEAVE_OUT + K12*G_YAWRATE_OUT
     YAWRATE_INPUT    = K21*G_HEAVE_OUT + K22*G_YAWRATE_OUT
     */
    IO_collective = IO_HEAVE_YAW_DECOUPLER_11*collective_input + IO_HEAVE_YAW_DECOUPLER_12*yawrate_input;
    IO_tailrate = IO_HEAVE_YAW_DECOUPLER_21*collective_input + IO_HEAVE_YAW_DECOUPLER_22*yawrate_input;
}

float IO_ESC_feedforward(float collective){
    return IO_FEEDFORWARD_SPEED*collective;
}

float IO_ESC_controller(float ref_speed, float dt){
    static float speed_input[3] = {0}, speed_output[3] = {0};

    float current_error = (ref_speed - IO_getFilteredRotorSpeed())*IO_RPM_2_RAD_PER_SEC;
    float upper_limit_int_speed = IO_SPEED_UPPER_LIMIT*IO_SPEED_OMEGA_TI/IO_SPEED_KP;
    float esc = IO_PI_with_limits(IO_SPEED_KP, IO_SPEED_OMEGA_TI, dt, current_error,
        &speed_input[0], &speed_output[0], 0, upper_limit_int_speed);
    return esc;
}

float IO_getAltitudeReference(float throttle){
    float altitude_ref = 0;

    if (throttle < IO_THROTTLE_ST1_MAX)
        altitude_ref = (throttle - IO_THROTTLE_ST1_MIN)/IO_ST1_DIV;
    else
        altitude_ref = (throttle - IO_THROTTLE_ST1_MAX)/IO_ST2_DIV + IO_ST1_MAX_HEIGHT;

    return altitude_ref;
}

char IO_limit_PWM(int sig){
    if (sig > 255)
        sig = 255;
    if (sig < 0)
        sig = 0;
    return (char)sig;
}

float IO_limits(float lower, float upper, float input){
    if (input > upper)
        input = upper;
    if (input < lower)
        input = lower;
    return input;
}

void IO_SystemIdentification(void){
    float tx = 0; float ty = 0; float tz = 0;
    RN131_extrapolatePosition(&tx, &ty, &tz);
    int sdlen = sprintf(&IO_sd_buffer[0],"%u,%u,%u,%u,%u,%u,%.0f,%.2f,%.2f,%.2f,%d,%d,%d,%.0f,%.0f,%.0f,%d,%d,%d,%d,%u\r\n",
                        IO_PWM_packet[0],IO_PWM_packet[1],IO_PWM_packet[2],IO_PWM_packet[3],IO_PWM_packet[4],IO_PWM_packet[5],
                        IO_getFilteredRotorSpeed(),
                        *IMU_getRoll()*180/M_PI, *IMU_getPitch()*180/M_PI, *IMU_getYaw()*180/M_PI,
                        ITG3200_getx(), ITG3200_gety(), ITG3200_getz(),
                        tx*1000, ty*1000, tz*1000, //RN131_getTx(), RN131_getTy(),  RN131_getTz(),
                        RN131_getVx(), RN131_getVy(),  RN131_getVz(),
                        RN131_getFilteredOffset(),
                        IO_updateCoreTime_us());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logControl(void){
    float tx = 0; float ty = 0; float tz = 0;
    RN131_extrapolatePosition(&tx, &ty, &tz);

    int i = 0;
    // log attitude reference (rads)
    memcpy(&IO_sd_buffer[i], (float *)&IO_roll_ref, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_pitch_ref, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_yaw_ref, 4); i += 4;
    // log attitude (measurement) (rads)
    memcpy(&IO_sd_buffer[i], (float *)IMU_getRoll(), 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)IMU_getPitch(), 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)IMU_getYaw(), 4); i += 4;
    // log position reference (m)
    memcpy(&IO_sd_buffer[i], (float *)&IO_x_ref, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_y_ref, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_z_ref, 4); i += 4;
    // log position measurement (m)
    float ultrasonic_height = IO_getAltitude();
    memcpy(&IO_sd_buffer[i], (float *)&tx, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&ty, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&ultrasonic_height, 4); i += 4;
    // log velocity measurement (mm/s)
    memcpy(&IO_sd_buffer[i], (short int *) RN131_getVx(), 2); i += 2;
    memcpy(&IO_sd_buffer[i], (short int *) RN131_getVy(), 2); i += 2;
    memcpy(&IO_sd_buffer[i], (short int *) RN131_getVz(), 2); i += 2;
    // log control signals
    memcpy(&IO_sd_buffer[i], (float *)&IO_lateral, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_longitudinal, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_collective, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_esc, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_tailrate, 4); i += 4;
    // log PWM packet
    memcpy(&IO_sd_buffer[i], (UINT8 *)&IO_PWM_packet, 6); i += 6;
    // log ref and actual speed
    memcpy(&IO_sd_buffer[i], (float *)&IO_ref_speed, 4); i += 4;
    memcpy(&IO_sd_buffer[i], (float *)&IO_filteredRotorSpeed, 4); i += 4;
    // log battery voltage
    memcpy(&IO_sd_buffer[i], (UINT16 *)&IO_batteryVoltage, 2); i += 2;
    // offset between pc and helicopter
    memcpy(&IO_sd_buffer[i], (INT32 *) RN131_getFilteredOffset(), 4); i += 4;
    // status register
    IO_sd_buffer[i] = IO_statusReg; i += 1;
    // log time
    UINT32 now = IO_updateCoreTime_us();
    memcpy(&IO_sd_buffer[i], (UINT32 *)&now, 4); i += 4;

    IO_logMessage(&IO_sd_buffer[0], i);
}

void IO_logQuaternion(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu\r\n",
            IMU_getQuaternion_q0(),IMU_getQuaternion_q1(),IMU_getQuaternion_q2(),IMU_getQuaternion_q3(),
            RN131_getQuaternion_q0(),RN131_getQuaternion_q1(),RN131_getQuaternion_q2(),RN131_getQuaternion_q3(),
            gyro_wx_rad(), gyro_wy_rad(), gyro_wz_rad(),
            IO_get_time_ms());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logRadio(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu\r\n",
                (UINT16)IO_getESC(),
                (UINT16)IO_getRight(),
                (UINT16)IO_getRear(),
                (UINT16)IO_getRudder(),
                (UINT16)IO_getGyroGain(),
                (UINT16)IO_getLeft(),
                SPEKTRUM_getEscRaw(),
                SPEKTRUM_getRightRaw(),
                SPEKTRUM_getRearRaw(),
                SPEKTRUM_getRudderRaw(),
                SPEKTRUM_getGyrogainRaw(),
                SPEKTRUM_getLeftRaw(),
                IO_getRadioError(),
                IO_updateCoreTime_us());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_logRadioMapping(void){
    int sdlen = sprintf(&IO_sd_buffer[0],"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu\r\n",
                SPEKTRUM_getESC(),
                SPEKTRUM_getRIGHT(),
                SPEKTRUM_getREAR(),
                SPEKTRUM_getRUDDER(),
                SPEKTRUM_getGAIN(),
                SPEKTRUM_getLEFT(),
                IO_PWM_packet[0],IO_PWM_packet[1],IO_PWM_packet[2],IO_PWM_packet[3],IO_PWM_packet[4],IO_PWM_packet[5],
                IO_updateCoreTime_us());
    IO_logMessage(&IO_sd_buffer[0], sdlen);
}

void IO_PIDComputation(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]){
    float nz1 = 0, nz2 = 0, nz3 = 0;
    float dz1 = 0, dz2 = 0, dz3 = 0;
    // compute coefficients

    if (pidtype == IO_PID){
        float Ts_squared = pow(Ts,2);
        nz1 = (4*Kp*alpha + 2*Kp*Ts*alpha*omega_Td + 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti);
        nz2 = (2*Kp*alpha*omega_Td*omega_Ti*Ts_squared - 8*Kp*alpha);
        nz3 = 4*Kp*alpha - 2*Kp*Ts*alpha*omega_Td - 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti;
        dz1 = (4*omega_Ti);
        dz2 = (-8*omega_Ti);
        dz3 = (4*omega_Ti);
    }
    else
    {
        nz1 = (2*Kp*alpha + Kp*Ts*alpha*omega_Td);
        nz2 = Kp*Ts*alpha*omega_Td - 2*Kp*alpha;
        nz3 = 0;
        dz1 = (Ts*alpha*omega_Td + 2);
        dz2 = Ts*alpha*omega_Td - 2;
        dz3 = 0;
    }
    // second-order discrete algo from z-transform
    u[0] = (nz1*e[0] + nz2*e[1] + nz3*e[2] - dz2*u[1] - dz3*u[2])/dz1;
}

void IO_PID_rateAux(IO_PID_TYPE pidtype, float Kp, float omega_Td, float omega_Ti, float alpha, float Ts,float u[3], float e[3]){
    float nz1 = 0, nz2 = 0, nz3 = 0;
    float dz1 = 0, dz2 = 0, dz3 = 0;
    // compute coefficients

    if (pidtype == IO_PID){
        float Ts_squared = pow(Ts,2);
        nz1 = (4*Kp*alpha + 2*Kp*Ts*alpha*omega_Td + 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti);
        nz2 = (2*Kp*alpha*omega_Td*omega_Ti*Ts_squared - 8*Kp*alpha);
        nz3 = 4*Kp*alpha - 2*Kp*Ts*alpha*omega_Td - 2*Kp*Ts*alpha*omega_Ti + Kp*Ts_squared*alpha*omega_Td*omega_Ti;
        dz1 = (4*omega_Ti);
        dz2 = (-8*omega_Ti);
        dz3 = (4*omega_Ti);
    }
    else
    {
        nz1 = (Kp*Ts*alpha*omega_Td);
        nz2 = (Kp*Ts*alpha*omega_Td);
        nz3 = 0;
        dz1 = (Ts*alpha*omega_Td + 2);
        dz2 = (Ts*alpha*omega_Td - 2);
        dz3 = 0;
    }
    // second-order discrete algo from z-transform
    u[0] = (nz1*e[0] + nz2*e[1] + nz3*e[2] - dz2*u[1] - dz3*u[2])/dz1;
}

void IO_PID_with_limits(float Kp, float omega_lead, float omega_Ti, float alpha, float Ts,
        float int_inputs[3], float int_outputs[3], float current_error,
        float int_lower_limit, float int_upper_limit,
        float diff_inputs[3], float diff_outputs[3]){

    // Perform PI control first
    IO_PI_with_limits(Kp, omega_Ti, Ts, current_error,
        &int_inputs[0], &int_outputs[0], int_lower_limit, int_upper_limit);

    // supply the output of the PI controller to the lead-lag filter
    float omega_lag = alpha*omega_lead;
    IO_lead_lag(omega_lead, omega_lag, Ts, &diff_inputs[0], &diff_outputs[0]);
}

float IO_PI_with_limits(const float Kp, const float omega_Ti, const float Ts, const float current_error,
    float filter_input[3], float filter_output[3], const float lower_limit, const float upper_limit){
    float nz1 = 0, nz2 = 0;
    float dz1 = 0, dz2 = 0;

    nz1 = 0;
    nz2 = 2*(Ts*omega_Ti);
    dz1 = (Ts*omega_Ti + 2);
    dz2 = (Ts*omega_Ti - 2);

    // shift input and output data
    IO_shiftData(&filter_input[0]);
    IO_shiftData(&filter_output[0]);
    // perform filter operation
    IO_filter_1st_order(nz1,nz2,dz1,dz2,&filter_input[0],&filter_output[0]);

    float current_output = current_error + filter_output[0];

    // Put saturation limits on integral control
    filter_input[0] = IO_limits(lower_limit, upper_limit, current_output);

    return filter_input[0]*Kp/omega_Ti;
}

float IO_lead_lag(float omega_lead, float omega_lag, float Ts, float filter_input[3], float filter_output[3]){
    float n1 = omega_lag*(Ts*omega_lead + 2);
    float n2 = omega_lag*(Ts*omega_lead - 2);
    float d1 = omega_lead*(Ts*omega_lag + 2);
    float d2 = omega_lead*(Ts*omega_lag - 2);

    IO_filter_1st_order(n1,n2,d1,d2,&filter_input[0],&filter_output[0]);

    return filter_output[0];
}

void IO_filter_1st_order(float n1, float n2, float d1, float d2, float input[3], float output[3]){
    output[0] = (n1*input[0] + n2*input[1] - d2*output[1])/d1;
}

void IO_filter_2nd_order(float n0, float n1, float n2, float d0, float d1, float d2, float input[3], float output[3]){
    output[0] = (n0*input[0] + n1*input[1] + n2*input[2] - d1*output[1] - d2*output[2])/d0;
}

float IO_filter_notch(float wn, float damp_p, float damp_z, float Ts, float input[3], float output[3]){
    float Ts_sqrd = pow(Ts,2);
    float wn_sqrd = pow(wn,2);
    float n0 = (Ts_sqrd*wn_sqrd + 4*damp_z*Ts*wn + 4);
    float n1 = (2*Ts_sqrd*wn_sqrd - 8);
    float n2 =  Ts_sqrd*wn_sqrd - 4*damp_z*Ts*wn + 4;
    float d0 = (Ts_sqrd*wn_sqrd + 4*damp_p*Ts*wn + 4);
    float d1 = (2*Ts_sqrd*wn_sqrd - 8);
    float d2 =  Ts_sqrd*wn_sqrd - 4*damp_p*Ts*wn + 4;

    IO_filter_2nd_order(n0, n1, n2, d0, d1, d2, &input[0], &output[0]);
    return output[0];
}

float IO_complex_lead_lag(float wn_z, float damp_z, float wn_p, float damp_p, float Ts, float input[3], float output[3]){
    float Ts_sqrd = pow(Ts,2);
    float wn_z_sqrd = pow(wn_z,2);
    float wn_p_sqrd = pow(wn_p,2);

    float n0 = (Ts_sqrd*wn_p_sqrd*wn_z_sqrd + 4*damp_z*Ts*wn_p_sqrd*wn_z + 4*wn_p_sqrd);
    float n1 = (2*Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 8*wn_p_sqrd);
    float n2 = Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 4*damp_z*Ts*wn_p_sqrd*wn_z + 4*wn_p_sqrd;
    float d0 = (Ts_sqrd*wn_p_sqrd*wn_z_sqrd + 4*damp_p*Ts*wn_p*wn_z_sqrd + 4*wn_z_sqrd);
    float d1 = (2*Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 8*wn_z_sqrd);
    float d2 = Ts_sqrd*wn_p_sqrd*wn_z_sqrd - 4*damp_p*Ts*wn_p*wn_z_sqrd + 4*wn_z_sqrd;

    IO_filter_2nd_order(n0, n1, n2, d0, d1, d2, &input[0], &output[0]);
    return output[0];
}

float IO_filter_speed(float speed_in, float dt, float wn){
    static float speed_input[3] = {0}, speed_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&speed_input[0]);
    speed_input[0] = speed_in;
    IO_shiftData(&speed_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &speed_input[0], &speed_output[0]);
    IO_filteredRotorSpeed = speed_output[0];
    return IO_filteredRotorSpeed;
}

float IO_filter_altitude_prefilter(float alt_in, float dt, float wn){
    static float alt_input[3] = {0}, alt_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&alt_input[0]);
    alt_input[0] = alt_in;
    IO_shiftData(&alt_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &alt_input[0], &alt_output[0]);
    return alt_output[0];
}

float IO_filter_yaw_prefilter(float yaw_in, float dt, float wn){
    static float yaw_input[3] = {0}, yaw_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&yaw_input[0]);
    yaw_input[0] = yaw_in;
    IO_shiftData(&yaw_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &yaw_input[0], &yaw_output[0]);
    return yaw_output[0];
}

void IO_filter_translationref_prefilter(volatile float * x_ref, volatile float * y_ref, float dt, float wn){
    static float x_input[3] = {0}, x_output[3] = {0};
    static float y_input[3] = {0}, y_output[3] = {0};

    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_shiftData(&x_input[0]);
    x_input[0] = *x_ref;
    IO_shiftData(&x_output[0]);

    IO_shiftData(&y_input[0]);
    y_input[0] = *y_ref;
    IO_shiftData(&y_output[0]);

    IO_filter_1st_order(n0, n1, d0, d1, &x_input[0], &x_output[0]);
    IO_filter_1st_order(n0, n1, d0, d1, &y_input[0], &y_output[0]);

    *x_ref = x_output[0];
    *y_ref = y_output[0];
}

float IO_LPF_1st(float input[3], float output[3], float dt, float wn){
    float n0 = (dt*wn);
    float n1 = (dt*wn);
    float d0 = (dt*wn + 2);
    float d1 = (dt*wn - 2);

    IO_filter_1st_order(n0, n1, d0, d1, &input[0], &output[0]);
    return output[0];
}

// Roll reference in rads
float IO_roll_control(float roll_reference, float dt, bool initialise){
    static float notch_input[3] = {0}, notch_output[3] = {0};

    float roll_error = roll_reference - (*IMU_getRoll());

    IO_shiftData(&notch_input[0]);
    notch_input[0] = roll_error;
    IO_shiftData(&notch_output[0]);

    if (initialise){
        memset(&notch_input[0],0,sizeof(float)*3);
        memset(&notch_output[0],0,sizeof(float)*3);
        return 0;
    }
    return IO_ROLL_KP*IO_complex_lead_lag(IO_ROLL_NOTCH_WN_Z, IO_ROLL_NOTCH_WZ_DAMP, IO_ROLL_NOTCH_WN_P, IO_ROLL_NOTCH_WP_DAMP, dt, &notch_input[0], &notch_output[0]);
}

// Pitch reference in rads
float IO_pitch_control(float pitch_reference, float dt, bool initialise){
    static float notch_input[3] = {0}, notch_output[3] = {0};

    float pitch_error = pitch_reference - (*IMU_getPitch());

    IO_shiftData(&notch_input[0]);
    notch_input[0] = pitch_error;
    IO_shiftData(&notch_output[0]);

    if (initialise){
        memset(&notch_input[0],0,sizeof(float)*3);
        memset(&notch_output[0],0,sizeof(float)*3);
        return 0;
    }

    return IO_PITCH_KP*IO_complex_lead_lag(IO_PITCH_NOTCH_WN_Z, IO_PITCH_NOTCH_WZ_DAMP, IO_PITCH_NOTCH_WN_P, IO_PITCH_NOTCH_WP_DAMP, dt, &notch_input[0], &notch_output[0]);
}

float IO_heave_control(float heave_reference, float height_sensor, float lower_ctrl_limit, float upper_ctrl_limit, float dt, bool initialise){
    static float heave_error[3] = {0}, heave_input[3] = {0};
    static float int_input[3] = {0}, int_output_[3] = {0};

    if (initialise){
        memset(&heave_error[0],0,sizeof(float)*3);
        memset(&heave_input[0],0,sizeof(float)*3);
        memset(&int_input[0],0,sizeof(float)*3);
        memset(&int_output_[0],0,sizeof(float)*3);
        return 0;
    }

    IO_shiftData(&heave_error[0]);
    // Negative because of the opposite frame of reference
    heave_error[0] = -(heave_reference - height_sensor);
    IO_shiftData(&heave_input[0]);

    // calculate derivative control signal
    float derivative_signal = IO_HEAVE_KP*IO_lead_lag(IO_HEAVE_OMEGA_LEAD, IO_HEAVE_OMEGA_LAG, dt, &heave_error[0], &heave_input[0]);
    float lower_limit_int = lower_ctrl_limit*IO_HEAVE_OMEGA_TI;
    float upper_limit_int = upper_ctrl_limit*IO_HEAVE_OMEGA_TI;

    return IO_PI_with_limits(1, IO_HEAVE_OMEGA_TI, dt, derivative_signal,
        &int_input[0], &int_output_[0], lower_limit_int, upper_limit_int);
}

float IO_yaw_control(float dt, float yaw_ref){
    static float yaw_error[3] = {0}, yaw_input[3] = {0};
    static float int_input[3] = {0}, int_output_[3] = {0};

    IO_shiftData(&yaw_error[0]);
    yaw_error[0] = yaw_ref - (*IMU_getYaw());
    IO_shiftData(&yaw_input[0]);

    // calculate derivative control signal
    float derivative_signal = IO_lead_lag((float)IO_YAW_OMEGA_LEAD, (float)IO_YAW_OMEGA_LAG, dt, &yaw_error[0], &yaw_input[0]);
    float lower_limit_int = IO_YAW_LOWER_LIMIT*IO_YAW_OMEGA_TI/IO_YAW_KP;
    float upper_limit_int = IO_YAW_UPPER_LIMIT*IO_YAW_OMEGA_TI/IO_YAW_KP;

    float yaw_control_sig = IO_PI_with_limits((float)IO_YAW_KP, (float)IO_YAW_OMEGA_TI, dt, derivative_signal,
        &int_input[0], &int_output_[0], lower_limit_int, upper_limit_int);

    return (-yaw_control_sig);
}

void IO_position_control(float x_ref, float y_ref, float z_ref, volatile float * roll_cmd, volatile float * pitch_cmd, float dt, bool initialise){
    static float error_x[3] = {0}, pitch_x[3] = {0};
    static float error_y[3] = {0}, roll_y[3] = {0};
    static float error_x2[3] = {0}, pitch_x2[3] = {0};
    static float error_y2[3] = {0}, roll_y2[3] = {0};
    static float x_int_input[3] = {0}, x_int_output[3] = {0};
    static float y_int_input[3] = {0}, y_int_output[3] = {0};
    static float x_lpf_input[3] = {0}, x_lpf_output[3] = {0};
    static float y_lpf_input[3] = {0}, y_lpf_output[3] = {0};

    float tx = 0; float ty = 0; float tz = 0;
    RN131_extrapolatePosition(&tx, &ty, &tz);

    float position_err_body[3] = {0};
    float position_err_inertial[3] = {x_ref - tx,y_ref - ty, 0};

    // use conjugate of quaternion to find body frame errors
    float q_conjugate[4] = {IMU_getQuaternion_q0(),-IMU_getQuaternion_q1(),-IMU_getQuaternion_q2(),-IMU_getQuaternion_q3()};
    Reb(&q_conjugate[0], &position_err_inertial[0], &position_err_body[0]);

    IO_shiftData(&error_x[0]);
    error_x[0] = position_err_body[0];
    IO_shiftData(&pitch_x[0]);

    IO_shiftData(&error_y[0]);
    error_y[0] = position_err_body[1];
    IO_shiftData(&roll_y[0]);

    if (initialise){
        memset(&error_x[0],0,sizeof(float)*3);
        memset(&pitch_x[0],0,sizeof(float)*3);
        memset(&error_y[0],0,sizeof(float)*3);
        memset(&roll_y[0],0,sizeof(float)*3);

        memset(&error_x2[0],0,sizeof(float)*3);
        memset(&pitch_x2[0],0,sizeof(float)*3);
        memset(&error_y2[0],0,sizeof(float)*3);
        memset(&roll_y2[0],0,sizeof(float)*3);

        memset(&x_int_input[0],0,sizeof(float)*3);
        memset(&x_int_output[0],0,sizeof(float)*3);
        memset(&y_int_input[0],0,sizeof(float)*3);
        memset(&y_int_output[0],0,sizeof(float)*3);

        memset(&x_lpf_input[0],0,sizeof(float)*3);
        memset(&x_lpf_output[0],0,sizeof(float)*3);
        memset(&y_lpf_input[0],0,sizeof(float)*3);
        memset(&y_lpf_output[0],0,sizeof(float)*3);

        *pitch_cmd = 0;
        *roll_cmd = 0;
        return;
    }

    // Calculate lead output
    float pitch_lead = IO_X_KP*IO_lead_lag(IO_X_OMEGA_LEAD_1, IO_X_OMEGA_LAG_1, dt, &error_x[0], &pitch_x[0]);
    float roll_lead = IO_Y_KP*IO_lead_lag(IO_Y_OMEGA_LEAD_1, IO_Y_OMEGA_LAG_1, dt, &error_y[0], &roll_y[0]);

    // add further lpf for sensor noise attenuation
    IO_shiftData(&x_lpf_input[0]);
    x_lpf_input[0] = pitch_lead;
    IO_shiftData(&x_lpf_output[0]);

    IO_shiftData(&y_lpf_input[0]);
    y_lpf_input[0] = roll_lead;
    IO_shiftData(&y_lpf_output[0]);

    pitch_lead = IO_LPF_1st(&x_lpf_input[0], &x_lpf_output[0], dt, IO_X_LPF_WN);
    roll_lead = IO_LPF_1st(&y_lpf_input[0], &y_lpf_output[0], dt, IO_Y_LPF_WN);

//    *pitch_cmd = IO_LPF_1st(&x_lpf_input[0], &x_lpf_output[0], dt, IO_X_LPF_WN);
//    *roll_cmd = IO_LPF_1st(&y_lpf_input[0], &y_lpf_output[0], dt, IO_Y_LPF_WN);

//    IO_shiftData(&error_x2[0]);
//    error_x2[0] = pitch_lead;
//    IO_shiftData(&pitch_x2[0]);
//
//    IO_shiftData(&error_y2[0]);
//    error_y2[0] = roll_lead;
//    IO_shiftData(&roll_y2[0]);
//
//    pitch_lead = IO_lead_lag(IO_X_OMEGA_LEAD_2, IO_X_OMEGA_LAG_2, dt, &error_x2[0], &pitch_x2[0]);
//    roll_lead = IO_lead_lag(IO_Y_OMEGA_LEAD_2, IO_Y_OMEGA_LAG_2, dt, &error_y2[0], &roll_y2[0]);

    float pitch_lower_limit_int = IO_PITCH_ANGLE_LOWER_LIMIT*IO_X_OMEGA_TI;
    float pitch_upper_limit_int = IO_PITCH_ANGLE_UPPER_LIMIT*IO_X_OMEGA_TI;
    float roll_lower_limit_int = IO_ROLL_ANGLE_LOWER_LIMIT*IO_Y_OMEGA_TI;
    float roll_upper_limit_int = IO_ROLL_ANGLE_UPPER_LIMIT*IO_Y_OMEGA_TI;

    *pitch_cmd = IO_PI_with_limits(1, IO_X_OMEGA_TI, dt, pitch_lead,
        &x_int_input[0], &x_int_output[0], pitch_lower_limit_int, pitch_upper_limit_int);
    *roll_cmd = IO_PI_with_limits(1, IO_Y_OMEGA_TI, dt, roll_lead,
        &y_int_input[0], &y_int_output[0], roll_lower_limit_int, roll_upper_limit_int);
}


void IO_shiftData(float data[3]){
    data[2] = data[1];
    data[1] = data[0];
}

// When most accurate current time is required, call this function
// Resolution of 1 microseconds
// Maximum count equivalent to 71 minutes
volatile UINT32 IO_updateCoreTime_us(void){
    static UINT32 previous_time = 0;

    // If this function is running by some process do not interrupt
    if (!IO_coreupdate_mutex){
        IO_coreupdate_mutex = true;
        UINT32 current_time = ReadCoreTimer();
        UINT32 dt = (UINT32)((UINT32)current_time - (INT32)previous_time);
        previous_time = current_time;
        IO_time_raw += dt;
        IO_coreupdate_mutex = false;
    }

    return (UINT32)(IO_time_raw / 40);
}

volatile UINT32 IO_getTime_us(void){
    return (UINT32)(IO_time_raw / 40);
}


// Used by RN131 to offset initial estimate
void IO_adjIO_time_us(INT32 offset_us){
    IO_time_raw = IO_time_raw + (INT64)offset_us*40;
}

/*
    ADC interrupt
    PRIORITY: 1
    SUBPRIORITY: 3
*/
void __ISR(_ADC_VECTOR, IO_ADC_IPL) ADCInterrupt( void)
{
    mAD1ClearIntFlag();
    IO_batteryVoltage = ADC1BUF0;
}

/*
    Enter interrupt every 10ms
    PRIORITY: 6
    SUBPRIORITY: 3
 *      - excute control loop
 *  takes worst case 1.5ms to execute
*/
void __ISR(_TIMER_3_VECTOR, IO_T3_IPL) _Timer3Handler(void){
    IO_control_exec();
    // clear the interrupt flag
    mT3ClearIntFlag();
}

/*
    Enter interrupt every 1ms
    PRIORITY: 7
    SUBPRIORITY: 2
 *      - used for timing reference
 *      - indicate system running
*/
void __ISR(_CORE_TIMER_VECTOR, IO_CT_IPL) CoreTimerHandler(void){
    static UINT16 heartbeat_count = 0;
    // Clear the interrupt flag
    mCTClearIntFlag();

    // Must be called regularly to keep time updated (atleast every 107s)
    IO_updateCoreTime_us();

    IO_time_ms++;
    IO_DATA_TIME++;

    // Update the period
    UpdateCoreTimer(CORE_TICK_RATE);

    // Must be called at least every ms to keep USB active
    #ifdef USE_USB
        CDCTxService();
    #endif

    // Initiate ADC read
    if ((IO_time_ms % IO_ADC_READ) == 0){
	AD1CON1bits.CLRASAM = 1;
	AD1CON1bits.ASAM = 1;
    }

    if ((IO_time_ms % IO_SD_FLUSH) == 0){
        IO_sdFlush = true;
    }

    SPEKTRUM_timeoutInc();
    RN131_timeoutInc();
    IO_timeoutSpeed();

    if (IO_SystemPreviousState != IO_SystemState)
        heartbeat_count = 0;

    switch(IO_SystemState){
        case SYS_BOOT:
            switch (heartbeat_count++){
                case 0:
                    IO_SET_HEARTBEAT();
                    break;
                case 100:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 175:
                    IO_SET_HEARTBEAT();
                    break;
                case 300:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 375:
                    IO_SET_HEARTBEAT();
                    break;
                case 800:
                    heartbeat_count = 0;
                    break;
            }
            break;
        case SYS_TAKEOFF:
        case SYS_FLY:
        case SYS_LAND:
            switch (heartbeat_count++){
                case 0:
                    IO_SET_HEARTBEAT();
                    break;
                case 100:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 175:
                    IO_SET_HEARTBEAT();
                    break;
                case 300:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 375:
                    IO_SET_HEARTBEAT();
                    break;
                case 1500:
                    heartbeat_count = 0;
                    break;
            }
            break;
        case SYS_MANUAL:
            switch (heartbeat_count++){
                case 1:
                    IO_SET_HEARTBEAT();
                    break;
                case 50:
                    IO_CLEAR_HEARTBEAT();
                    break;
                case 100:
                    heartbeat_count = 0;
                    break;
            }
            break;
        case SYS_ERROR:
            // If an error, toggle every 1 second
            if ((IO_time_ms % IO_ERROR_BLINK) == 0){
                IO_TOGGLE_HEARTBEAT();
            }
            break;
    }

    IO_SystemPreviousState = IO_SystemState;

    // check if PROGRAM button is pressed, if so, close any open files
    if (!PORTReadBits(IOPORT_A, BIT_1)){
        IO_closeFiles = true;
    }
}

/*
 * This interrupt is used for measuring motor speed
 * PRIORITY: 7
 * SUBPRIORITY: 3
 */
void __ISR(_CHANGE_NOTICE_VECTOR, IO_CN_IPL) changeNotificationHandler(void){
    static int prev_time_stamp[10] = {0};
    static bool first_run[10] = {true};
    static bool pin_prev_state[10] = {low};

    int temp_time;
    // clear the mismatch condition
    unsigned int port_val_D = mPORTDRead();
    unsigned int port_val_B = mPORTBRead();

    // clear the interrupt flag
    mCNClearIntFlag();

#if defined(SETUP_RADIO_DETECT)
    // Measuring the period of the pulse...not the mark time
    if ((port_val_D & IO_OPTO_ENCODER_PIND) && (pin_prev_state[0] == low))
    {
        pin_prev_state[0] = high;
        if (first_run[0])
        {
            prev_time_stamp[0] = ReadCoreTimer();
            first_run[0] = false;
        }
        else
        {
            temp_time = ReadCoreTimer();
            IO_rotor_speed_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[0];
            IO_rotor_speed_update = 0;
            prev_time_stamp[0] = temp_time;
        }
    }
    if (!(port_val_D & IO_OPTO_ENCODER_PIND) && (pin_prev_state[0] == high))
    {
        pin_prev_state[0] = low;
    }

    if ((port_val_D & IO_GYROGAIN_PIND) && (pin_prev_state[1] == low))
    {
        pin_prev_state[1] = high;
        prev_time_stamp[1] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_GYROGAIN_PIND) && (pin_prev_state[1] == high))
    {
        temp_time = ReadCoreTimer();
        IO_gyrogain_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[1];
        prev_time_stamp[1] = temp_time;
        pin_prev_state[1] = low;

        if ((IO_gyrogain_dt > CN_PWM_MAX) || (IO_gyrogain_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11101111;
        }
        else{
            IO_valid_PWM_data |= 0b00010000;
        }
    }

    if ((port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == low))
    {
        pin_prev_state[2] = high;
        prev_time_stamp[2] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == high))
    {
        temp_time = ReadCoreTimer();
        IO_ultrasonic_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[2];
        prev_time_stamp[2] = temp_time;
        pin_prev_state[2] = low;

        ultrasonic_updated = true;

        if (IO_ultrasonic_dt > IO_ULTRASONIC_DT_MAX)
            IO_ultrasonic_dt = IO_ULTRASONIC_DT_MAX;
    }

    if ((port_val_B & IO_ESC_PINB) && (pin_prev_state[3] == low))
    {
        pin_prev_state[3] = high;
        prev_time_stamp[3] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_ESC_PINB) && (pin_prev_state[3] == high))
    {
        temp_time = ReadCoreTimer();
        IO_esc_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[3];
        prev_time_stamp[3] = temp_time;
        pin_prev_state[3] = low;

        if ((IO_esc_dt > CN_PWM_MAX) || (IO_esc_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11111110;
        }
        else{
            IO_valid_PWM_data |= 0b00000001;
        }
    }

    if ((port_val_B & IO_RIGHT_PINB) && (pin_prev_state[4] == low))
    {
        pin_prev_state[4] = high;
        prev_time_stamp[4] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_RIGHT_PINB) && (pin_prev_state[4] == high))
    {
        temp_time = ReadCoreTimer();
        IO_right_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[4];
        prev_time_stamp[4] = temp_time;
        pin_prev_state[4] = low;

        if ((IO_right_dt > CN_PWM_MAX) || (IO_right_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11111101;
        }
        else{
            IO_valid_PWM_data |= 0b00000010;
        }
    }

    if ((port_val_B & IO_REAR_PINB) && (pin_prev_state[5] == low))
    {
        pin_prev_state[5] = high;
        prev_time_stamp[5] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_REAR_PINB) && (pin_prev_state[5] == high))
    {
        temp_time = ReadCoreTimer();
        IO_rear_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[5];
        prev_time_stamp[5] = temp_time;
        pin_prev_state[5] = low;

        if ((IO_rear_dt > CN_PWM_MAX) || (IO_rear_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11111011;
        }
        else{
            IO_valid_PWM_data |= 0b00000100;
        }
    }

    if ((port_val_B & IO_RUDDER_PINB) && (pin_prev_state[6] == low))
    {
        pin_prev_state[6] = high;
        prev_time_stamp[6] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_RUDDER_PINB) && (pin_prev_state[6] == high))
    {
        temp_time = ReadCoreTimer();
        IO_rudder_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[6];
        prev_time_stamp[6] = temp_time;
        pin_prev_state[6] = low;

        if ((IO_rudder_dt > CN_PWM_MAX) || (IO_rudder_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11110111;
        }
        else{
            IO_valid_PWM_data |= 0b00001000;
        }
    }

    if ((port_val_B & IO_LEFT_PINB) && (pin_prev_state[7] == low))
    {
        pin_prev_state[7] = high;
        prev_time_stamp[7] = ReadCoreTimer();
    }
    if(!(port_val_B & IO_LEFT_PINB) && (pin_prev_state[7] == high))
    {
        temp_time = ReadCoreTimer();
        IO_left_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[7];
        prev_time_stamp[7] = temp_time;
        pin_prev_state[7] = low;

        if ((IO_left_dt > CN_PWM_MAX) || (IO_left_dt < CN_PWM_MIN)){
            IO_valid_PWM_data &= 0b11011111;
        }
        else{
            IO_valid_PWM_data |= 0b00100000;
        }
    }

    // if any pin is invalid --> radio error true
    if (IO_valid_PWM_data == 0b00111111){
        IO_RADIO_ERROR = false;
    }
    else{
        IO_RADIO_ERROR = true;
    }

#else
    // Measuring the period of the pulse...not the mark time
    if ((port_val_D & IO_OPTO_ENCODER_PIND) && (pin_prev_state[0] == low))
    {
        pin_prev_state[0] = high;
        if (first_run[0])
        {
            prev_time_stamp[0] = ReadCoreTimer();
            first_run[0] = false;
        }
        else
        {
            temp_time = ReadCoreTimer();
            IO_rotor_speed_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[0];
            IO_rotor_speed_update = 0;
            prev_time_stamp[0] = temp_time;
        }
    }
    if (!(port_val_D & IO_OPTO_ENCODER_PIND) && (pin_prev_state[0] == high))
    {
        pin_prev_state[0] = low;
    }

    if ((port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == low))
    {
        pin_prev_state[2] = high;
        prev_time_stamp[2] = ReadCoreTimer();
    }
    if(!(port_val_D & IO_ULTRASONIC_PIND) && (pin_prev_state[2] == high))
    {
        temp_time = ReadCoreTimer();
        IO_ultrasonic_dt = (unsigned int) ReadCoreTimer() - (int) prev_time_stamp[2];
        prev_time_stamp[2] = temp_time;
        pin_prev_state[2] = low;
    }
#endif

}

#undef IO_H_IMPORT