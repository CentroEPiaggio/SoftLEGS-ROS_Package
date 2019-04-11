// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         May 16, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED

//==================================================================     include
#include <device.h>

//=====================================================        Interrupt Handler
    
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);
CY_ISR_PROTO(ISR_WATCHDOG_Handler);

//=====================================================     function declaration

void function_scheduler();

void analog_read_end();


void interrupt_manager();


// ----------------------------------------------------------------------------

#endif

/* [] END OF FILE */