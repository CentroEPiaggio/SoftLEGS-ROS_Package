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
            
        case CMD_SET_ZEROS:
            setZeros();
            break;

//============================================================     CMD_GET_PARAM
            
        case CMD_GET_PARAM_LIST:
            get_param_list( *((uint16 *) &g_rx.buffer[1]));
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
            
//========================================================     CMD_GET_IMUS_SFS

        case CMD_GET_IMUS_SFS:
            cmd_get_imus_sfs();
            break;              
            
//=========================================================== ALL OTHER COMMANDS
        default:
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
//                                                                     SET ZEROS
//==============================================================================

void setZeros()
{
    //nothing to do and nothing to say
}

//==============================================================================
//                                                            GET PARAMETER LIST
//==============================================================================

void get_param_list(uint16 index)
{
    //Package to be sent variables
    uint8 packet_data[901] = "";
    uint16 packet_lenght = 901;

    //Auxiliary variables
    uint16 CYDATA i;
    int32 aux_int;

    //Parameters menu string definitions
    char id_str[15]             = "1 - Device ID:";
    char acc_unity_str[34]      = "2 - Accelerometers unity:";
    char gyro_unity_str[28]     = "3 - Gyroscopes unity:";
    char mag_unity_str[28]      = "4 - Magnetometers unity:";
    char acc_fs_str[39]         = "5 - Accelerometers full range:";
    char gyro_fs_str[42]        = "6 - Gyroscopes full range:";

    //Parameters menus
    char acc_unity_menu[21] = "0 -> G\n1 -> m/(s^2)\n";
    char gyro_unity_menu[23] = "0 -> rad/s\n1 -> deg/s\n";
    char acc_fs_menu[50] = "0 -> +/- 2G\n1 -> +/- 4G\n2 -> +/- 8G\n3 -> +/- 16G\n";
    char gyro_fs_menu[59] = "0 -> +/- 250 deg/s\n1 -> +/- 500 deg/s\n2 -> +/- 2000 deg/s\n";

    //Strings lenghts
    uint8 CYDATA id_str_len = strlen(id_str);
    uint8 CYDATA acc_unity_str_len = strlen(acc_unity_str);
    uint8 CYDATA gyro_unity_str_len = strlen(gyro_unity_str);
    uint8 CYDATA mag_unity_str_len = strlen(mag_unity_str);
    uint8 CYDATA acc_fs_str_len = strlen(acc_fs_str);
    uint8 CYDATA gyro_fs_str_len = strlen(gyro_fs_str);
    uint8 CYDATA acc_unity_menu_len = strlen(acc_unity_menu);
    uint8 CYDATA gyro_unity_menu_len = strlen(gyro_unity_menu);
    uint8 CYDATA acc_fs_menu_len = strlen(acc_fs_menu);
    uint8 CYDATA gyro_fs_menu_len = strlen(gyro_fs_menu);

    packet_data[0] = CMD_GET_PARAM_LIST;
    packet_data[1] = NUM_OF_PARAMS;

    switch(index) {
        case 0:         //List of all parameters with relative types
            /*-----------------ID-----------------*/

            packet_data[2] = TYPE_UINT8;
            packet_data[3] = 1;
            packet_data[4] = c_mem.id;
            for(i = id_str_len; i != 0; i--)
                packet_data[5 + id_str_len - i] = id_str[id_str_len - i];

            /*-------------ACC UNITY--------------*/

            packet_data[52] = TYPE_FLAG;
            packet_data[53] = 1;
            packet_data[54] = c_mem.acc_unity;
            switch(c_mem.acc_unity){
                case 0:
                    strcat(acc_unity_str, " G\0");
                    acc_unity_str_len = 28;
                break;
                case 1:
                    strcat(acc_unity_str, " m/(s^2)\0");
                    acc_unity_str_len = 34;
                break;
            }
            for(i = acc_unity_str_len; i != 0; i--)
                packet_data[55 + acc_unity_str_len - i] = acc_unity_str[acc_unity_str_len - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[55 + acc_unity_str_len] = 1;
            
            /*-------------GYRO UNITY-------------*/

            packet_data[102] = TYPE_FLAG;
            packet_data[103] = 1;
            packet_data[104] = c_mem.gyro_unity;
            switch(c_mem.gyro_unity){
                case 0:
                    strcat(gyro_unity_str, " deg/s\0");
                    gyro_unity_str_len = 28;
                break;
                case 1:
                    strcat(gyro_unity_str, " rad/s\0");
                    gyro_unity_str_len = 28;
                break;
            }
            for(i = gyro_unity_str_len; i != 0; i--)
                packet_data[105 + gyro_unity_str_len - i] = gyro_unity_str[gyro_unity_str_len - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[105 + gyro_unity_str_len] = 2;    
                
            /*-------------MAG UNITY--------------*/

            packet_data[152] = TYPE_FLAG;
            packet_data[153] = 1;
            packet_data[154] = c_mem.mag_unity;
            switch(c_mem.mag_unity){
                case 0:
                    strcat(mag_unity_str, " uT\0");
                    mag_unity_str_len = 28;
                break;
            }
            for(i = mag_unity_str_len; i != 0; i--)
                packet_data[155 + mag_unity_str_len - i] = mag_unity_str[mag_unity_str_len - i];

            /*------------ACC FULL RANGE-------------*/
            
            packet_data[202] = TYPE_FLAG;
            packet_data[203] = 1;
            packet_data[204] = c_mem.acc_sf;
            switch(c_mem.acc_sf){
                case 0:
                    strcat(acc_fs_str, " +/- 2G\0");
                    acc_fs_str_len = 38;
                break;
                case 1:
                    strcat(acc_fs_str, " +/- 4G\0");
                    acc_fs_str_len = 38;
                break;
                case 2:
                    strcat(acc_fs_str, " +/- 8G\0");
                    acc_fs_str_len = 38;
                break;
                case 3:
                    strcat(acc_fs_str, " +/- 16G\0");
                    acc_fs_str_len = 39;
                break;
            }
            for(i = acc_fs_str_len; i != 0; i--)
                packet_data[205 + acc_fs_str_len - i] = acc_fs_str[acc_fs_str_len - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[205 + acc_fs_str_len] = 3;   
                
            /*------------GYRO FULL RANGE-------------*/
            
            packet_data[252] = TYPE_FLAG;
            packet_data[253] = 1;
            packet_data[254] = c_mem.gyro_sf;
            switch(c_mem.gyro_sf){
                case 0:
                    strcat(gyro_fs_str, " +/- 250 deg/s\0");
                    gyro_fs_str_len = 41;
                break;
                case 1:
                    strcat(gyro_fs_str, " +/- 500 deg/s\0");
                    gyro_fs_str_len = 41;
                break;
                case 2:
                    strcat(gyro_fs_str, " +/- 2000 deg/s\0");
                    gyro_fs_str_len = 42;
                break;
            }
            for(i = gyro_fs_str_len; i != 0; i--)
                packet_data[255 + gyro_fs_str_len - i] = gyro_fs_str[gyro_fs_str_len - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[255 + gyro_fs_str_len] = 4;                

            /*------------PARAMETERS MENU-----------*/

            for(i = acc_unity_menu_len; i != 0; i--)
                packet_data[302 + acc_unity_menu_len - i] = acc_unity_menu[acc_unity_menu_len - i];
                
            for(i = gyro_unity_menu_len; i != 0; i--)
                packet_data[452 + gyro_unity_menu_len - i] = gyro_unity_menu[gyro_unity_menu_len - i];
                
            for(i = acc_fs_menu_len; i != 0; i--)
                packet_data[602 + acc_fs_menu_len - i] = acc_fs_menu[acc_fs_menu_len - i];
             
            for(i = gyro_fs_menu_len; i != 0; i--)
                packet_data[752 + gyro_fs_menu_len - i] = gyro_fs_menu[gyro_fs_menu_len - i];

            packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);
            UART_RS485_ClearTxBuffer();
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;
        
//=========================================================     set_acc_unity
        case 2:         //Acc unity - uint8
            g_mem.acc_unity = g_rx.buffer[3];
        break;

//=========================================================     set_gyro_unity
        case 3:         //Gyro unity - uint8
            g_mem.gyro_unity = g_rx.buffer[3];
        break;
            
//=========================================================     set_mag_unity
        case 4:         //Mag unity - uint8
            g_mem.mag_unity = g_rx.buffer[3];
            
            g_mem.mag_unity = 0;    // Further use (only uT measures at the moment)
        break;
            
//=========================================================     set_acc_sf
        case 5:         //Acc Full Range Scale - uint8
            g_mem.acc_sf = g_rx.buffer[3];
            
            ImusReset();
        break; 

//=========================================================     set_gyro_sf
        case 6:         //Gyro Full Range Scale - uint8
            g_mem.gyro_sf = g_rx.buffer[3];
            
            ImusReset();
        break; 
            
    }
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
     
        sprintf(str, "IMU Connected: %d\r\n", (int) N_IMU_Connected);
        strcat(info_string, str);
        
        strcat(info_string, "\r\n");
        
        strcat(info_string, "DEVICE PARAMETERS\r\n");
        strcat(info_string, "Accelerometers unity: ");
        switch(c_mem.acc_unity){
            case 0:
                strcat(info_string, "G\r\n");
                break;
            case 1:
                strcat(info_string, "m/(s^2)\r\n");
                break;
        }
        
        strcat(info_string, "Gyroscopes unity: ");
        switch(c_mem.gyro_unity){
            case 0:
                strcat(info_string, "deg/s\r\n");
                break;
            case 1:
                strcat(info_string, "rad/s\r\n");
                break;
        }
        
        strcat(info_string, "Magnetometers unity: ");
        switch(c_mem.mag_unity){
            case 0:
                strcat(info_string, "uT\r\n");
                break;
        }
        
        strcat(info_string, "Accelerometers full range: ");
        switch(c_mem.acc_sf){
            case 0:
                strcat(info_string, "+/- 2G\r\n");
            break;
            case 1:
                strcat(info_string, "+/- 4G\r\n");
            break;
            case 2:
                strcat(info_string, "+/- 8G\r\n");
            break;
            case 3:
                strcat(info_string, "+/- 16G\r\n");
            break;
        }
        
        strcat(info_string, "Gyroscopes full range: ");
        switch(c_mem.gyro_sf){
            case 0:
                strcat(info_string, "+/- 250 deg/s\r\n");
            break;
            case 1:
                strcat(info_string, "+/- 500 deg/s\r\n");
            break;
            case 2:
                strcat(info_string, "+/- 2000 deg/s\r\n");
            break;
        }
        strcat(info_string, "\r\n");
        
        strcat(info_string, "IMUs CONFIGURATION\r\n");
        for (i=0; i<N_IMU_Connected; i++){
            sprintf(str, "Imu %d \r\n\tID: %d\r\n", i, (int) IMU_connected[i]);
            strcat(info_string, str);
            
            sprintf(str, "\tAccelerometers: ");
            if ((IMU_conf[IMU_connected[i]][0]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n"); 
            strcat(str, "\tGyroscopes: ");
            if ((IMU_conf[IMU_connected[i]][1]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n"); 
            strcat(str, "\tMagnetometers: ");
            if ((IMU_conf[IMU_connected[i]][2]))
                strcat(str, "YES\r\n");
            else
                strcat(str, "NO\r\n"); 
            
            strcat(info_string, str);
        }       
        
        strcat(info_string, "\r\n");
            
        strcat(info_string, "SENSORS INFO\r\n");
        for (i=0; i<N_IMU_Connected; i++){
            sprintf(str, "Imu %d \r\n\tID: %d\r\n", i, (int) IMU_connected[i]);
            strcat(info_string, str);
            
            if ((IMU_conf[IMU_connected[i]][0])){
                sprintf(str, "\tAcc: %d\t%d\t%d\r\n", (int16) g_imu[i].accel_value[0], (int16) g_imu[i].accel_value[1],(int16) g_imu[i].accel_value[2]);
                strcat(info_string, str);
            }

            if ((IMU_conf[IMU_connected[i]][1])){
                sprintf(str, "\tGyro: %d\t%d\t%d\r\n", (int16) g_imu[i].gyro_value[0], (int16) g_imu[i].gyro_value[1],(int16) g_imu[i].gyro_value[2]);
                strcat(info_string, str);
            }

            if ((IMU_conf[IMU_connected[i]][2])){
                sprintf(str, "\tMag: %d\t%d\t%d\r\n", (int16) g_imu[i].mag_value[0], (int16) g_imu[i].mag_value[1],(int16) g_imu[i].mag_value[2]);
                strcat(info_string, str);
            }
        
        }
                   
        strcat(info_string, "\r\n");
    }
}

//==============================================================================
//                                                      WRITE FUNCTION FOR RS485
//==============================================================================

void commWrite(uint8 *packet_data, const uint16 packet_lenght)
{
    uint16 CYDATA index;

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    
    // frame - ID
    UART_RS485_PutChar(g_mem.id);
    
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    
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

 
    g_mem.acc_unity     = 0;
    
    g_mem.gyro_unity    = 0;
    
    g_mem.mag_unity     = 0;
    
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
    
    // Retrieve a summary of what we are reading from all imus
    uint8 CYDATA k_imu = 0;
    uint8 flags;
    uint8 sz = 1 + 4*N_IMU_Connected + 1;
    
    // Packet: header + imu id(uint8) + imu flags(uint8) + crc  
    uint8 packet_data[50];

    //Header package 
    packet_data[0] = CMD_GET_MAG_PARAM;

    for (k_imu=0; k_imu < N_IMU_Connected; k_imu++) 
    {	
 	    packet_data[1 + 4*k_imu]     = (uint8) IMU_connected[k_imu];                // IMU id
        packet_data[1 + 4*k_imu + 1] = (uint8) MagCal[IMU_connected[k_imu]][0];     // IMU MagCal X
        packet_data[1 + 4*k_imu + 2] = (uint8) MagCal[IMU_connected[k_imu]][1];     // IMU MagCal Y
        packet_data[1 + 4*k_imu + 3] = (uint8) MagCal[IMU_connected[k_imu]][2];     // IMU MagCal X
	}

    // Calculate Checksum and send message to UART 
    packet_data[sz-1] = LCRChecksum (packet_data, sz-1);
    commWrite(packet_data, sz);
    
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

void cmd_get_imus_sfs(){
    // Retrieve a summary of ACC, GYRO and MAG scale factors and unities
    uint8 packet_data[50];
    uint8 sz = 1 + (6 * sizeof(float)) + 1;
    
    float acc_sf_par = 0.0;
    float gyro_sf_par = 0.0;
    float mag_sf_par = 0.0;
    
    acc_sf_par = ACC_SF * (float)rateAcc;   // Obtain SF for g
    switch (g_mem.acc_unity) {
        case 0:     // leave SF as is
            break;
        case 1:
            acc_sf_par = acc_sf_par * G_TO_MS2; // for m/(s^2) measures
            break;
        default:
            break;
    }
    
    gyro_sf_par = GYRO_SF * (float)rateGyro;     // Obtain SF for deg/s
    switch (g_mem.gyro_unity) {
        case 0:     // leave SF as is
            break;
        case 1:
            gyro_sf_par = gyro_sf_par * DEG_TO_RAD;   // for rad/s measures
            break;
        default:
            break;
    }
    
    mag_sf_par = MAG_SF;
    switch (g_mem.mag_unity) {      // further uses (only uT measures at the moment)
        default:
            break;
    }
    
    packet_data[0] = CMD_GET_IMUS_SFS;
    
    // Scale factors
    *((float *) ( packet_data + 1 )) = (float) acc_sf_par;      // SF to g
    *((float *) ( packet_data + 5 )) = (float) gyro_sf_par;     // SF to deg/s
    *((float *) ( packet_data + 9 )) = (float) mag_sf_par;      // SF to uT
    
    // Unities
    *((float *) ( packet_data + 13 )) = (float) g_mem.acc_unity;    // 0 -> g, 1 -> m/(s^2)
    *((float *) ( packet_data + 17 )) = (float) g_mem.gyro_unity;   // 0 -> deg/s, 1 -> rad/s
    *((float *) ( packet_data + 21 )) = (float) g_mem.mag_unity;    // 0 -> uT
    
    // Calculate Checksum and send message to UART 
    packet_data[sz-1] = LCRChecksum (packet_data, sz-1);
    commWrite(packet_data, sz);
    
}
/* [] END OF FILE */