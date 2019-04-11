// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         May 16, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED
// -----------------------------------------------------------------------------

//=================================================================     includes
#include <device.h>
#include "stdlib.h"
#include "math.h"
#include "commands.h"

//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION                 "IMUboard v0.1"

#define NUM_OF_ANALOG_INPUTS      1
#define N_IMU_MAX                   17


//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

//Main frequency 1000 Hz
#define CALIBRATION_DIV         10     // 100 Hz

#define DIV_INIT_VALUE          1

//==============================================================================
//                                                                           DMA
//==============================================================================
    
#define DMA_BYTES_PER_BURST 2
#define DMA_REQUEST_PER_BURST 1
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)
    
//==============================================================================
//                                                                     INTERRUPT
//==============================================================================
#define    WAIT_START   0
#define    WAIT_ID      1
#define    WAIT_LENGTH  2
#define    RECEIVE      3
#define    UNLOAD       4

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE                   0
#define TRUE                    1

#define DEFAULT_EEPROM_DISPLACEMENT 8   // in pages

#define MAX_WATCHDOG_TIMER 250          // num * 2 [cs]

#define PWM_MAX_VALUE           100     // PWM is from 0 to 100

#define ANTI_WINDUP             1000
#define DEFAULT_CURRENT_LIMIT   1000    // Current limit for hand closing
                                        // 0 means unlimited
#define CURRENT_HYSTERESIS      10      // mA of hysteresis for current control

#define EMG_SAMPLE_TO_DISCARD   500     // Number of sample to discard before calibration
#define SAMPLES_FOR_MEAN        100

#define SAMPLES_FOR_EMG_MEAN    1000

#define CALIB_DECIMATION        1
#define NUM_OF_CLOSURES         5

#define POS_INTEGRAL_SAT_LIMIT  50000000
#define CURR_INTEGRAL_SAT_LIMIT 100000

#define MIN_CURR_SAT_LIMIT      30

//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references

//struct st_ref {
//    int32 pos[NUM_OF_MOTORS];       // motor reference position
//    int32 curr[NUM_OF_MOTORS];
//    int32 pwm[NUM_OF_MOTORS];
//    uint8 onoff;                    // enable flags
//};
//
////=============================================================     measurements
//
//struct st_meas {
//    int32 pos[NUM_OF_SENSORS];      // sensor position
//
//    int32 curr[NUM_OF_MOTORS];      // motor currents
//
//    int8 rot[NUM_OF_SENSORS];      // sensor rotations
//
//    int32 emg[NUM_OF_EMGS];         // EMG values
//};

//==============================================================     data packet

struct st_data {
    uint8   buffer[128];            // CMD/DATA/CHECKSUM
    int16   length;                 // length
    int16   ind;                    // index
    uint8   ready;                  // Flag
};

//============================================     settings stored on the memory

struct st_mem {
    uint8   flag;                       // Device has been configured               1
    uint8   id;                         // device id                                1
   
    uint8   baud_rate;                  // Baud Rate Setted                         1
    uint8   watchdog_period;            // Watchdog period setted, 255 = disable    1                                                                //TOT           102 bytes
};


//==============================================     hand calibration parameters


//=================================================     emg status

struct st_imu {
    uint8 flags;        // Flags to know what we are reading (0/1) from each imu [ accel | gyro | magn ]
    float accel_value[3];
    float gyro_value[3];
    float mag_value[3];
};


//====================================      external global variables definition

extern struct st_data   g_rx;                       // income data
extern struct st_mem    g_mem, c_mem;               // memory

extern uint32 timer_value;
extern uint32 timer_value0;

// Device Data

extern int32   dev_tension;                         // Power supply tension
extern uint8   dev_pwm_limit;

// Bit Flag

extern CYBIT reset_last_value_flag;
extern CYBIT tension_valid;                         // tension validation bit
extern CYBIT interrupt_flag;                        // interrupt flag enabler
extern CYBIT watchdog_flag;                         // watchdog flag enabler
extern float tau_feedback;

// DMA Buffer

extern int16 ADC_buf[1];

// PWM value

extern int8 pwm_sign;

// IMU variables
extern uint8 N_IMU_Connected;
extern uint8 IMU_connected[N_IMU_MAX];
extern uint8 IMU_conf[N_IMU_MAX][3];
extern uint8 imu_send_flag;
extern int imus_data_size;
extern int single_imu_size[N_IMU_MAX];
extern struct st_imu g_imu[N_IMU_MAX], g_imuNew[N_IMU_MAX];

extern uint8 Accel[N_IMU_MAX][6];
extern uint8 Gyro[N_IMU_MAX][6];
extern uint8 Mag[N_IMU_MAX][6];


// -----------------------------------------------------------------------------


#endif

//[] END OF FILE