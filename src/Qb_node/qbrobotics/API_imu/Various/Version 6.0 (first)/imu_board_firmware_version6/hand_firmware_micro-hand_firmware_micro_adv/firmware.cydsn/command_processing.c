// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/**
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         May 16, 2016
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include <interruptions.h>
#include <stdio.h>
#include <utils.h>
#include <IMU_functions.h>
#include <globals.h>

#include "commands.h"

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(){

    uint8 CYDATA rx_cmd;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }


    switch(rx_cmd) {

//=============================================================     CMD_ACTIVATE
        case CMD_ACTIVATE:
            cmd_activate();
            break;

//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            cmd_set_inputs();
            break;

//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            cmd_get_measurements();
            break;

//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            cmd_get_currents();
            break;


//=========================================================     CMD_GET_EMG

        case CMD_GET_EMG:
            cmd_get_emg();
            break;

//=============================================================     CMD_WATCHDOG
            
        case CMD_SET_WATCHDOG:
            cmd_set_watchdog();
            break;
            
//=========================================================     CMD_GET_ACTIVATE
            
        case CMD_GET_ACTIVATE:
            cmd_get_activate();
            break;
            
//=========================================================     CMD_SET_BAUDRATE
            
        case CMD_SET_BAUDRATE:
            cmd_set_baudrate();
            break;  
            
//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            cmd_get_inputs();
            break;

//=============================================================     CMD_GET_INFO

        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]));
            break;

//============================================================     CMD_SET_PARAM

        case CMD_SET_PARAM:
            paramSet( *((uint16 *) &g_rx.buffer[1]) );
            break;

//============================================================     CMD_GET_PARAM

        case CMD_GET_PARAM:
            paramGet( *((uint16 *) &g_rx.buffer[1]) );
            break;

//=================================================================     CMD_PING
            
        case CMD_PING:
            cmd_ping();
            break;

//=========================================================     CMD_STORE_PARAMS
            
        case CMD_STORE_PARAMS:
            cmd_store_params();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS

        case CMD_STORE_DEFAULT_PARAMS:
            if(memStore(DEFAULT_EEPROM_DISPLACEMENT))
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            if(memRestore())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if(memInit())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//===========================================================     CMD_BOOTLOADER

        case CMD_BOOTLOADER:
            //Not sure if ACK_OK is correct, should check
            sendAcknowledgment(ACK_OK);
            CyDelay(1000);
            FTDI_ENABLE_REG_Write(0x00);
            CyDelay(1000);
            Bootloadable_Load();
            break;

//============================================================     CMD_CALIBRATE

        case CMD_CALIBRATE:
            break;
  
//============================================================     CMD_GET_N_IMU

        case CMD_GET_N_IMU:
            cmd_get_n_imu();
            break;
            
//========================================================     CMD_GET_MAG_PARAM

        case CMD_GET_MAG_PARAM:
            cmd_get_mag_param();
            break;   
            
//=====================================================     CMD_GET_IMU_READINGS

        case CMD_GET_IMU_READINGS:
            cmd_get_imu_readings();
            break;
            
//=========================================================     CMD_SET_IMU_MODE

        case CMD_SET_IMU_MODE:
            cmd_set_imu_mode();
            break;
            
//========================================================     CMD_GET_IMUS_MODE

        case CMD_GET_IMUS_MODE:
            cmd_get_imus_mode();
            break;    
            
    }
    
}


//==============================================================================
//                                                                     INFO SEND
//==============================================================================

void infoSend(){
    unsigned char packet_string[1100];
    infoPrepare(packet_string);
    UART_RS485_PutString(packet_string);
}


//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type) {
    static unsigned char packet_string[1100];

    //==================================     choose info type and prepare string

    switch (info_type) {
        case INFO_ALL:
            infoPrepare(packet_string);
            UART_RS485_PutString(packet_string);
            break;

        default:
            break;
    }
}

//==============================================================================
//                                                        COMMAND SET PARAMETER
//==============================================================================


void paramSet(uint16 param_type)
{
    uint8 CYDATA i;        // iterator
    int32 aux_int;  // auxiliary variable
    uint8 aux_uchar;

    switch(param_type) {

//===================================================================     set_id

        case PARAM_ID:
            g_mem.id = g_rx.buffer[3];
            break;

//=======================================================     set_pid_parameters

        case PARAM_PID_CONTROL:
            break;

//==================================================     set_curr_pid_parameters

        case PARAM_PID_CURR_CONTROL:
            break;

//===================================================     set_startup_activation

        case PARAM_STARTUP_ACTIVATION:
            break;

//===========================================================     set_input_mode

        case PARAM_INPUT_MODE:
            break;

//=========================================================     set_control_mode

        case PARAM_CONTROL_MODE:
            break;

//===========================================================     set_resolution

        case PARAM_POS_RESOLUTION:
            break;

//===============================================================     set_offset

        case PARAM_MEASUREMENT_OFFSET:
            break;

//===========================================================     set_multiplier

        case PARAM_MEASUREMENT_MULTIPLIER:
            break;

//=====================================================     set_pos_limit_enable

        case PARAM_POS_LIMIT_FLAG:
            break;

//============================================================     set_pos_limit

        case PARAM_POS_LIMIT:
            break;

//===============================================     set_max_step_pos_per_cycle

        case PARAM_MAX_STEP_POS:
            break;

//===============================================     set_max_step_neg_per_cycle

        case PARAM_MAX_STEP_NEG:
            break;

//========================================================     set_current_limit

        case PARAM_CURRENT_LIMIT:
            break;

//=======================================================     set_emg_calib_flag

        case PARAM_EMG_CALIB_FLAG:
            break;

//========================================================     set_emg_threshold

        case PARAM_EMG_THRESHOLD:
            break;

//========================================================     set_emg_max_value

        case PARAM_EMG_MAX_VALUE:
            break;

//============================================================     set_emg_speed

        case PARAM_EMG_SPEED:
            break;

//================================================     set_double_encoder_on_off
        case PARAM_DOUBLE_ENC_ON_OFF:
            break;

//===================================================     set_motor_handle_ratio
        case PARAM_MOT_HANDLE_RATIO:
            break;

//===================================================     set_motor_supply_type
        case PARAM_MOTOR_SUPPLY:
            break;

    }
    //Not sure if ACK_OK is correct, should check
    sendAcknowledgment(ACK_OK);
}


//==============================================================================
//                                                         COMMAND GET PARAMETER
//==============================================================================

void paramGet(uint16 param_type)
{
    uint8 packet_data[20];
    uint16 packet_lenght;
    uint8 i;                // iterator

    packet_data[0] = CMD_GET_PARAM;

    switch(param_type) {

//===================================================================     get_id

        case PARAM_ID:
            packet_data[1] = c_mem.id;
            packet_lenght = 3;
            break;

//=======================================================     get_pid_parameters

        case PARAM_PID_CONTROL:
            break;

//=======================================================     get_pid_parameters

        case PARAM_PID_CURR_CONTROL:
            break;

//===================================================     get_startup_activation

        case PARAM_STARTUP_ACTIVATION:
            break;

//===========================================================     get_input_mode

        case PARAM_INPUT_MODE:
            break;

//=========================================================     get_control_mode

        case PARAM_CONTROL_MODE:
            break;

//===========================================================     get_resolution

        case PARAM_POS_RESOLUTION:
            break;

//===============================================================     get_offset

        case PARAM_MEASUREMENT_OFFSET:
            break;

//===========================================================     get_multiplier

        case PARAM_MEASUREMENT_MULTIPLIER:
            break;

//=====================================================     get_pos_limit_enable

        case PARAM_POS_LIMIT_FLAG:
            break;

//============================================================     get_pos_limit

        case PARAM_POS_LIMIT:
            break;

//=========================================================     get_max_step_pos

        case PARAM_MAX_STEP_POS:
            break;

//=========================================================     get_max_step_neg

        case PARAM_MAX_STEP_NEG:
            break;

//========================================================     get_current_limit

        case PARAM_CURRENT_LIMIT:
            break;

//=======================================================     get_emg_calib_flag

        case PARAM_EMG_CALIB_FLAG:
            break;

//========================================================     get_emg_threshold

        case PARAM_EMG_THRESHOLD:
            break;

//========================================================     get_emg_max_value

        case PARAM_EMG_MAX_VALUE:
            break;

//============================================================     get_emg_speed

        case PARAM_EMG_SPEED:
            break;

//================================================     get_double_encoder_on_off
        case PARAM_DOUBLE_ENC_ON_OFF:
            break;

//===================================================     get_motor_handle_ratio
        case PARAM_MOT_HANDLE_RATIO:
            break;

//===================================================     get_motor_supply_type
        case PARAM_MOTOR_SUPPLY:
            break;

//===================================================     default
        default:
            break;
    }

    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int i;
    if(c_mem.id != 250){                //To avoid dummy board ping
        unsigned char str[100];
        strcpy(info_string, "");
        strcat(info_string, "\r\n");
        strcat(info_string, "Firmware version: ");
        strcat(info_string, VERSION);
        strcat(info_string, "\r\n\r\n");

        strcat(info_string, "DEVICE INFO\r\n");
        sprintf(str, "ID: %d\r\n", (int) c_mem.id);
        strcat(info_string, str);
        
      
        strcat(info_string, "ACC\r\n");
        sprintf(str, "X - acc H: %d   acc L: %d \r\n", (int) Accel[0][0], (int) Accel[0][1]);
        strcat(info_string, str);
        sprintf(str, "Y - acc H: %d   acc L: %d \r\n", (int) Accel[0][2], (int) Accel[0][3]);
        strcat(info_string, str);
        sprintf(str, "Z - acc H: %d   acc L: %d \r\n", (int) Accel[0][4], (int) Accel[0][5]);
        strcat(info_string, str);
        
        
        strcat(info_string, "IMU Connected\r\n");
        sprintf(str, "numero: %d\r\n", (int) N_IMU_Connected);
        strcat(info_string, str);
        
        for (i=0; i<N_IMU_Connected; i++){
            sprintf(str, "Imu %d: %d\r\n", i, (int) IMU_connected[i]);
            strcat(info_string, str);
            sprintf(str, "Flags: %d, %d, %d\r\n", (int)(IMU_conf[IMU_connected[i]][0]), (int)(IMU_conf[IMU_connected[i]][1]), (int)(IMU_conf[IMU_connected[i]][2]));
            strcat(info_string, str);
           
            
            
/*            sprintf(str, "Acc values %f, %f, %f\r\n", (float)g_imu[i].accel_value[0], (float)g_imu[i].accel_value[1], (float)g_imu[i].accel_value[2]);
            strcat(info_string, str);
            sprintf(str, "Gyro values %f, %f, %f\r\n", (float)g_imu[i].gyro_value[0], (float)g_imu[i].gyro_value[1], (float)g_imu[i].gyro_value[2]);
            strcat(info_string, str);
            sprintf(str, "Mag values %f, %f, %f\r\n", (float)g_imu[i].mag_value[0], (float)g_imu[i].mag_value[1], (float)g_imu[i].mag_value[2]);
            strcat(info_string, str);
 */           
            strcat(info_string, "\r\n");
        }       
    
        sprintf(str, "Last RX buffer: %s\r\n", g_rx.buffer);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        strcat(info_string, "\r\n");
    }
}

//==============================================================================
//                                                      WRITE FUNCTION FOR RS485
//==============================================================================

void commWrite(uint8 *packet_data, const uint8 packet_lenght)
{
    uint16 CYDATA index;

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    
    // frame - ID
    UART_RS485_PutChar(g_mem.id);
    
    // frame - length
    UART_RS485_PutChar(packet_lenght);
    
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index)
        UART_RS485_PutChar(packet_data[index]);
    
    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}


//==============================================================================
//                                                             CHECKSUM FUNCTION
//==============================================================================

// Performs a XOR byte by byte on the entire vector

uint8 LCRChecksum(uint8 *data_array, uint8 data_length) {
    
    uint8 CYDATA i;
    uint8 CYDATA checksum = 0x00;
    
    for(i = 0; i < data_length; ++i)
       checksum ^= data_array[i];
  
    return checksum;
}

//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment(const uint8 value) {
    int packet_lenght = 2;
    uint8 packet_data[2];

    packet_data[0] = value;
    packet_data[1] = value;

    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================

/**
* This function stores current memory settings on the eeprom with the specified
* displacement
**/

uint8 memStore(int displacement)
{
    int i;  // iterator
    uint8 writeStatus;
    int pages;
    uint8 ret_val = 1;

    // Disable Interrupt
    ISR_RS485_RX_Disable();


    // Retrieve temperature for better writing performance
    CySetTemp();

    memcpy( &c_mem, &g_mem, sizeof(g_mem) );

    pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);

    for(i = 0; i < pages; ++i) {
        writeStatus = EEPROM_Write(&g_mem.flag + 16 * i, i + displacement);
        if(writeStatus != CYRET_SUCCESS) {
            ret_val = 0;
            break;
        }
    }

    memcpy( &g_mem, &c_mem, sizeof(g_mem) );

    // Re-Enable Interrupt
    ISR_RS485_RX_Enable();

    return ret_val;
}


//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================

/**
* This function loads user settings from the eeprom.
**/

void memRecall()
{
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        memRestore();
    } else {
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    }
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================

/**
* This function loads default settings from the eeprom.
**/

uint8 memRestore() {
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        return memInit();
    } else {
        return memStore(0);
    }
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================

/**
* This function initialize memory when eeprom is compromised.
**/

uint8 memInit()
{
    uint8 i;

    //initialize memory settings
    g_mem.id            = 1;

 
    // set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;
    
    //write that configuration to EEPROM
    return ( memStore(0) && memStore(DEFAULT_EEPROM_DISPLACEMENT) );
}

//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================
/**
* Bunch of functions used on request from UART communication
**/

void cmd_get_measurements(){
   //nothing to do and nothing to say 
}

void cmd_set_inputs(){
   //nothing to do and nothing to say
}

void cmd_activate(){
    //nothing to do and nothing to say
}

void cmd_get_activate(){
    //nothing to do and nothing to say
}

void cmd_get_curr_and_meas(){
    //nothing to do and nothing to say   
}

void cmd_get_currents(){
    //nothing to do and nothing to say
}

void cmd_set_baudrate(){
    
    // Set BaudRate
    c_mem.baud_rate = g_rx.buffer[1];
    
    switch(g_rx.buffer[1]){
        case 13:
            CLOCK_UART_SetDividerValue(13);
            break;
        default:
            CLOCK_UART_SetDividerValue(3);
    }
}

void cmd_ping(){

    uint8 packet_data[2];

    // Header
    packet_data[0] = CMD_PING;
    
    // Load Payload
    packet_data[1] = CMD_PING;

    // Send Package to uart
    commWrite(packet_data, 2);
}

void cmd_set_watchdog(){
      
    if (g_rx.buffer[1] <= 0){
        // Deactivate Watchdog
        WATCHDOG_ENABLER_Write(1); 
        g_mem.watchdog_period = 0;   
    }
    else{
        // Activate Watchdog        
        if (g_rx.buffer[1] > MAX_WATCHDOG_TIMER)
            g_rx.buffer[1] = MAX_WATCHDOG_TIMER;
            
        // Period * Time_CLK = WDT
        // Period = WTD / Time_CLK =     (WTD    )  / ( ( 1 / Freq_CLK ) )
        // Set request watchdog period - (WTD * 2)  * (250 / 1024        )
        g_mem.watchdog_period = (uint8) (((uint32) g_rx.buffer[1] * 2 * 250 ) >> 10);   
        WATCHDOG_COUNTER_WritePeriod(g_mem.watchdog_period); 
        WATCHDOG_ENABLER_Write(0); 
    }
}

void cmd_get_inputs(){
    //nothing to do and nothing to say
}

void cmd_store_params(){
   
    if(memStore(0))
        sendAcknowledgment(ACK_OK);
    else
        sendAcknowledgment(ACK_ERROR);
}

void cmd_get_emg(){
    //nothing to do and nothing to say
}

void cmd_get_n_imu(){
    //Get number of IMUs connected to board
    
    uint8 packet_data[3];

    // Header        
    packet_data[0] = CMD_GET_N_IMU;
    
    // Fill payload
    packet_data[1] = (uint8) N_IMU_Connected;
    
    // Calculate checksum
    packet_data[2] = LCRChecksum(packet_data, 2);
    
    // Send package to UART
    commWrite(packet_data, 3);
}


void cmd_get_mag_param(){
    //nothing to do and nothing to say
}

void cmd_get_imu_readings(){
    //Retrieve accelerometers, gyroscopes and magnetometers readings
    
    // Packet to send is handled by isr_imu interrupt routine (in this way you work with consistent data and no other imu interrupt disturbs data sending)
    imu_send_flag = 1;
}

void cmd_set_imu_mode(){
    uint8 k_imu = 0;
    
    // Set sensors to read for each IMU
    uint8 id = (uint8) g_rx.buffer[1];
    uint8 flags = (uint8) g_rx.buffer[2];
    
    IMU_conf[id][0] = (flags & 0x04) >> 2;
    IMU_conf[id][1] = (flags & 0x02) >> 1;
    IMU_conf[id][2] = flags & 0x01;
    
    imus_data_size = 1; //header
    
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++)
    {
        single_imu_size[IMU_connected[k_imu]] = 1 + 6*IMU_conf[IMU_connected[k_imu]][0] + 6*IMU_conf[IMU_connected[k_imu]][1] + 6*IMU_conf[IMU_connected[k_imu]][2] + 1;
        imus_data_size = imus_data_size + single_imu_size[IMU_connected[k_imu]];
    }
    imus_data_size = imus_data_size + 1;    //checksum
}

void cmd_get_imus_mode(){
    // Retrieve a summary of what we are reading from all imus
    uint8 CYDATA k_imu = 0;
    uint8 flags;
    uint8 sz = 1 + 2*N_IMU_Connected + 1;
    
    // Packet: header + imu id(uint8) + imu flags(uint8) + crc  
    uint8 packet_data[50];

    //Header package 
    packet_data[0] = CMD_GET_IMUS_MODE;

    for (k_imu=0; k_imu < N_IMU_Connected; k_imu++) 
    {	
 	    packet_data[1 + 2*k_imu] = (uint8) IMU_connected[k_imu];        // IMU id
        packet_data[1 + 2*k_imu + 1] = (uint8) ((IMU_conf[IMU_connected[k_imu]][0]<<2) | (IMU_conf[IMU_connected[k_imu]][1]<<1) | (IMU_conf[IMU_connected[k_imu]][2]));      // IMU reading flags
	}

    // Calculate Checksum and send message to UART 
    packet_data[sz-1] = LCRChecksum (packet_data, sz-1);
    commWrite(packet_data, sz);

}
/* [] END OF FILE */