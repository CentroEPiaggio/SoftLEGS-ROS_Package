// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         globals.c
*
* \brief        Global variables.
* \date         May 16, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_data  g_rx;                       // income data
struct st_mem   g_mem, c_mem;               // memory

float tau_feedback;

// Timer value for debug field

uint32 timer_value;
uint32 timer_value0;

// Device Data

int32   dev_tension;                // Power supply tension
uint8   dev_pwm_limit;

// Bit Flag

CYBIT reset_last_value_flag;
CYBIT tension_valid;
CYBIT interrupt_flag;
CYBIT watchdog_flag;

// DMA Buffer

int16 ADC_buf[1]; 

// PWM value
int8 pwm_sign;

// IMU variables
uint8 N_IMU_Connected;
uint8 IMU_connected[N_IMU_MAX];
uint8 IMU_conf[N_IMU_MAX][3];
uint8 imu_send_flag;
int imus_data_size;
int single_imu_size[N_IMU_MAX];

struct st_imu g_imu[N_IMU_MAX];
struct st_imu g_imuNew[N_IMU_MAX];

uint8 Accel[N_IMU_MAX][6];
uint8 Gyro[N_IMU_MAX][6];
uint8 Mag[N_IMU_MAX][6];

/* END OF FILE */