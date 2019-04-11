// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------
// File:        data_processing.c
//
// Description: Data processing functions.
// ----------------------------------------------------------------------------
//
// Project:             qbotFirmware
// Project Manager(s):  Mattia Poggiani and Alessandro Raugi
//
// Soft. Ver:           6.1.0
// Date:                2016-09-23
//
// This version changes:
//      - not reported
//
// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// ----------------------------------------------------------------------------
// Permission is granted to copy, modify and redistribute this file, provided
// header message is retained.
// ----------------------------------------------------------------------------
#ifndef COMMAND_PROCESSING_H_INCLUDED
#define COMMAND_PROCESSING_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//==============================================================================
//                                                          function definitions
//==============================================================================


void	setZeros 			(void);
void	get_param_list		(uint16 index); 
void    infoPrepare        (unsigned char *);
void    infoSend           ();
void    infoGet            (uint16);
void    commProcess        ();
void    commWrite          (uint8*, const uint16);
uint8   memStore           (int);
void    sendAcknowledgment (const uint8);
void    memRecall          ();
uint8   memRestore         ();
uint8   memInit            ();
uint8   LCRChecksum        (uint8 *, uint8);


//==============================================================================
//                                            Service Routine interrupt function
//==============================================================================

void cmd_activate();
void cmd_set_inputs();
void cmd_get_measurements();
void cmd_get_currents();
void cmd_get_emg();
void cmd_set_watchdog();
void cmd_get_activate();
void cmd_set_baudrate();
void cmd_get_inputs();
void cmd_store_params();
void cmd_ping();
void cmd_get_n_imu();
void cmd_get_mag_param();
void cmd_get_imu_readings();
void cmd_set_imu_mode();
void cmd_get_imus_mode();
void cmd_get_imus_sfs();
    
#endif

/* [] END OF FILE */