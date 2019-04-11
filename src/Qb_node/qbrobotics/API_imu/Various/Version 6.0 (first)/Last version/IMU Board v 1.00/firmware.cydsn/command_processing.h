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
// Project Manager(s):  Fabio Bonomo and Felipe Belo
//
// Soft. Ver:           0.1b4
// Date:                2012-02-06
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


void    paramSet           (uint16);
void    paramGet           (uint16);
void    infoPrepare        (unsigned char *);
void    infoSend           ();
void    infoGet            (uint16);
void    commProcess        ();
void    commWrite          (uint8*, const uint8);
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
    
#endif

/* [] END OF FILE */