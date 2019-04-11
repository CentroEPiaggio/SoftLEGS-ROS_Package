// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------


/**
* \file         main.c
*
* \brief        Firmware main file.
* \date         May 16, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

/**
* \mainpage     Firmware
* \brief        This is the firmware of the qb move.
* \version      0.1 beta 4
*
* \author       _qbrobotics_
*
* \date         May 16, 2012
*
* \details      This is the firmware of the qb move.
*
* \copyright    (C)  qbrobotics. All rights reserved.
*
*/


// ----------------------------------------------------------------------------
// This version changes:
//      - not reported


//=================================================================     includes

////#include <device.h>
////#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
////#include <interruptions.h>
////#include <command_processing.h>
////#include <utils.h>

//==============================================================================
//                                                                 MAIN FUNCTION
//==============================================================================

////int i;              //iterator


////void main()
////{
//====================================     initializations - psoc and components

    // EEPROM

////    EEPROM_Start();
////    memRecall();                                        // recall configuration

    // RS485

////    CyDelay(100);
////    FTDI_ENABLE_REG_Write(0x01);

////    UART_RS485_Stop();                                  // stop UART
////    UART_RS485_Start();                                 // start UART
////    UART_RS485_Init();

////    UART_RS485_ClearRxBuffer();
////    UART_RS485_ClearTxBuffer();

////    ISR_RS485_RX_StartEx(ISR_RS485_RX_ExInterrupt);     // RS485 isr function

    // ADC

////    ADC_Start();                            // start ADC
////    ADC_StartConvert();

////    RS485_CTS_Write(0);

////    CYGlobalIntEnable;                      // enable interrupts


//=========================================================     application loop

    for(;;)
    {

        // Call function scheduler
        function_scheduler();

        //  Wait until the FF is set to 1
        while(FF_STATUS_Read() == 0);

        // Command a FF reset
        RESET_FF_Write(0x01);

        // Wait for FF to be reset
        while(FF_STATUS_Read() == 1);

        if(UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_SOFT_BUFF_OVER)
            UART_RS485_ClearRxBuffer();
    }
}



/* [] END OF FILE */