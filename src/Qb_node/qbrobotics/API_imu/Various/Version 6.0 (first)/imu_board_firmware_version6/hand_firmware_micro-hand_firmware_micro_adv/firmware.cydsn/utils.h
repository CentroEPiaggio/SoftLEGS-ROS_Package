// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         utils.h
*
* \brief        Declaration of utility functions.
* \date         May 16, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/


#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <globals.h>

//------------------------------------------------------------------     DEFINES

#define TIMER_CLOCK 10000

#define ALPHA 3
#define BETA 50

#define SIGN(A) (((A) >=0) ? (1) : (-1))

//-------------------------------------------------------------     DECLARATIONS

int32 filter_v(int32 new_value);
int32 filter_ch1(int32 value);
int32 filter_ch2(int32 value);
int32 filter_i1(int32 value);

int my_round(const double x);
uint32 my_mod(int32 val, int32 divisor);
CYBIT check_enc_data(const uint32*);

int calc_turns_fcn(const int32,const int32);

void calibration();

void torque_feedback();

#endif

/* [] END OF FILE */
