/* ========================================
 *
LE DUE CONFIGURAZIONI VANNO RISCRITTE IN FUNZIONE DELLA NUOVA SCHEDA ED CONFIGURABILI TRAMITE SOFTWARE
 * Configurazione Mano Sinistra
 *
 * CS0 = 8; CS1 = 9; CS2 = 10;    // Little
 * CS3 = 4; CS4 = 5; CS5 = 6;     // Ring
 * CS6 = 44; CS7 = 40; CS8 = 39;  // Medium
 * CS9 = 35; CSA = 34; CSB = 33;  // Index
 * CSC = 32; CSD = 30; CSE = 29;  // Thumb
 * CSF = 11; CSG = 15;            // Palm
 * 
 * Configurazione Mano Destra
 *
 * CS0 = 32; CS1 = 30; CS2 = 29;  // Little
 * CS3 = 35; CS4 = 34; CS5 = 33;  // Ring
 * CS6 = 44; CS7 = 40; CS8 = 39;  // Medium
 * CS9 = 4; CSA = 5; CSB = 6;     // Index
 * CSC = 8; CSD = 9; CSE = 10;    // Thumb
 * CSF = 11; CSG = 15;            // Palm
 *
* ========================================
*/
#include <project.h>
#include <stdlib.h>
#include <IMU_functions.h>
// FILE AGGIUNTI DA FIRMWARE MANO
#include <device.h>
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
#include <interruptions.h>
#include <command_processing.h>
////

//global variables declaration 
extern	uint8 Accel[N_IMU][6];
extern	uint8 Gyro[N_IMU][6];
int status;
uint8 flag = 0;  //if flag=? start usb uart
int soglia;
int k_imu;

int main()
{
    EEPROM_Start();
    // QUESTA LA RIMETTO DOPO memRecall();                                        // recall configuration
    // RS485
    CyDelay(100);
    FTDI_ENABLE_REG_Write(0x01);
    UART_RS485_Stop();                                  // stop UART
    UART_RS485_Start();                                 // start UART
    UART_RS485_Init();
    UART_RS485_ClearRxBuffer();
    UART_RS485_ClearTxBuffer();
    ISR_RS485_RX_StartEx(ISR_RS485_RX_ExInterrupt);     // RS485 isr function QUESTA VA SCRITTA

	//SPI module
	SPIM_1_Start();
	SPIM_1_Init();
	SPIM_1_Enable();
	SPIM_1_ClearRxBuffer();
	SPIM_1_ClearTxBuffer();
	SPIM_1_ClearFIFO();							
    CyDelay(10);
    
    // ADC
    ADC_Start();                            // start ADC
    ADC_StartConvert();

	// Init MPU9250 devices
	for (k_imu=0; k_imu<N_IMU; k_imu++) {	
	    Chip_Select_Write(k_imu);
	    CyDelay(10);
	    InitIMU();
	    CyDelay(10);
	}
    
    RS485_CTS_Write(0);
    
    //interrupt start
	CyGlobalIntEnable;                         

    isr_1_Start();
    isr_1_Enable();
        
    status = 0;
	CyDelay(30);
    
    for(;;){
    
    // Nel loop controllo se è arrivato qualcosa sulla seriale 
        
        if(UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_SOFT_BUFF_OVER) {UART_RS485_ClearRxBuffer();}
    
    // QUI DEVO AGGIUNGERE I COMANDI PER IL CONTROLLO DEL GUANTO    
        
   // Serial Communication Control
   // flag = UART_1_GetChar();
   //if(flag != 0u) {
	//		if(flag == '?') {
	//	    status = 1; //Enable Transfer Data Read from IMU
	//	}
	//	if (flag == '!') {
	//		status = 0; //Disable Transfer Data Read from IMU
	//	}
	//	if (flag == 'r') {
	//		status = 0;
	//	    GloveReset(); // Reset Glove
	//	}
      //  if (flag == 'i') { UART_1_PutChar(device_id); } //Send Device ID to computer 
        
     //   if (flag == 'h') { UART_1_PutString("Mano Sinistra"); }

    //}	
} // End While 
} // End Main