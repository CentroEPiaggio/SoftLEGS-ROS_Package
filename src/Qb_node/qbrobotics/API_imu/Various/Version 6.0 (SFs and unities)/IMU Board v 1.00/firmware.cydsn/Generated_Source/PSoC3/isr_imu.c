/*******************************************************************************
* File Name: isr_imu.c  
* Version 1.70
*
*  Description:
*   API for controlling the state of an interrupt.
*
*
*  Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/


#include <cydevice_trm.h>
#include <CyLib.h>
#include <isr_imu.h>


#if !defined(isr_imu__REMOVED) /* Check for removal by optimization */

/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START isr_imu_intc` */
#include <IMU_functions.h>
#include <command_processing.h>

extern uint8 N_IMU_Connected;
extern uint8 IMU_connected[N_IMU_MAX];
extern uint8 IMU_conf[N_IMU_MAX][3];
extern uint8 imu_send_flag;
extern int imus_data_size;
extern int single_imu_size[N_IMU_MAX];

extern uint8 Accel[N_IMU_MAX][6];
extern uint8 Gyro[N_IMU_MAX][6];
extern uint8 Mag[N_IMU_MAX][6];   
extern struct st_imu g_imu[N_IMU_MAX];

extern float scaleAccFactor; 
extern float scaleGyroFactor;
extern float scaleMagFactor;
/* `#END` */


/*******************************************************************************
* Function Name: isr_imu_Start
********************************************************************************
*
* Summary:
*  Set up the interrupt and enable it. This function disables the interrupt, 
*  sets the default interrupt vector, sets the priority from the value in the
*  Design Wide Resources Interrupt Editor, then enables the interrupt to the 
*  interrupt controller.
*
* Parameters:  
*   None
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_Start(void) 
{
    /* For all we know the interrupt is active. */
    isr_imu_Disable();

    /* Set the ISR to point to the isr_imu Interrupt. */
    isr_imu_SetVector(&isr_imu_Interrupt);

    /* Set the priority. */
    isr_imu_SetPriority((uint8)isr_imu_INTC_PRIOR_NUMBER);

    /* Enable it. */
    isr_imu_Enable();
}


/*******************************************************************************
* Function Name: isr_imu_StartEx
********************************************************************************
*
* Summary:
*  Sets up the interrupt and enables it. This function disables the interrupt,
*  sets the interrupt vector based on the address passed in, sets the priority 
*  from the value in the Design Wide Resources Interrupt Editor, then enables 
*  the interrupt to the interrupt controller.
*  
*  When defining ISR functions, the CY_ISR and CY_ISR_PROTO macros should be 
*  used to provide consistent definition across compilers:
*  
*  Function definition example:
*   CY_ISR(MyISR)
*   {
*   }
*   Function prototype example:
*   CY_ISR_PROTO(MyISR);
*
* Parameters:  
*   address: Address of the ISR to set in the interrupt vector table.
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_StartEx(cyisraddress address) 
{
    /* For all we know the interrupt is active. */
    isr_imu_Disable();

    /* Set the ISR to point to the isr_imu Interrupt. */
    isr_imu_SetVector(address);

    /* Set the priority. */
    isr_imu_SetPriority((uint8)isr_imu_INTC_PRIOR_NUMBER);

    /* Enable it. */
    isr_imu_Enable();
}


/*******************************************************************************
* Function Name: isr_imu_Stop
********************************************************************************
*
* Summary:
*   Disables and removes the interrupt.
*
* Parameters:  
*   None
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_Stop(void) 
{
    /* Disable this interrupt. */
    isr_imu_Disable();
}


/*******************************************************************************
* Function Name: isr_imu_Interrupt
********************************************************************************
*
* Summary:
*   The default Interrupt Service Routine for isr_imu.
*
*   Add custom code between the START and END comments to keep the next version
*   of this file from over-writing your code.
*
*   Note You may use either the default ISR by using this API, or you may define
*   your own separate ISR through ISR_StartEx().
*
* Parameters:  
*   None
*
* Return:
*   None
*
*******************************************************************************/
CY_ISR(isr_imu_Interrupt)
{
    #ifdef isr_imu_INTERRUPT_INTERRUPT_CALLBACK
        isr_imu_Interrupt_InterruptCallback();
    #endif /* isr_imu_INTERRUPT_INTERRUPT_CALLBACK */

    /*  Place your Interrupt code here. */
    /* `#START isr_imu_Interrupt` */
    uint8 k_imu;
    uint8 c = 1;
    uint8 k = 0;
    uint8 gl_c = 1;
    uint16 tmp = 0;

    // Packet: header + imu id(uint8) + imu flags(uint8) + crc  
    uint8 packet_data[350];
    uint8 single_packet[20];
    
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++){
        ChipSelector(IMU_connected[k_imu]);
        ReadIMU(IMU_connected[k_imu]);   
       
        tmp = Accel[IMU_connected[k_imu]][0];
        g_imu[k_imu].accel_value[0] = (int16)(tmp<<8 | Accel[IMU_connected[k_imu]][1]);
        tmp = Accel[IMU_connected[k_imu]][2];
        g_imu[k_imu].accel_value[1] = (int16)(tmp<<8 | Accel[IMU_connected[k_imu]][3]);
        tmp = Accel[IMU_connected[k_imu]][4];
        g_imu[k_imu].accel_value[2] = (int16)(tmp<<8 | Accel[IMU_connected[k_imu]][5]);
        
        tmp = Gyro[IMU_connected[k_imu]][0];
        g_imu[k_imu].gyro_value[0] = (int16)(tmp<<8 | Gyro[IMU_connected[k_imu]][1]);
        tmp = Gyro[IMU_connected[k_imu]][2];
        g_imu[k_imu].gyro_value[1] = (int16)(tmp<<8 | Gyro[IMU_connected[k_imu]][3]);
        tmp = Gyro[IMU_connected[k_imu]][4];
        g_imu[k_imu].gyro_value[2] = (int16)(tmp<<8 | Gyro[IMU_connected[k_imu]][5]);
        
        tmp = Mag[IMU_connected[k_imu]][0];
        g_imu[k_imu].mag_value[0] = (int16)(tmp<<8 | Mag[IMU_connected[k_imu]][1]);
        tmp = Mag[IMU_connected[k_imu]][2];
        g_imu[k_imu].mag_value[1] = (int16)(tmp<<8 | Mag[IMU_connected[k_imu]][3]);
        tmp = Mag[IMU_connected[k_imu]][4];
        g_imu[k_imu].mag_value[2] = (int16)(tmp<<8 | Mag[IMU_connected[k_imu]][5]);
    }
  
    
    if (imu_send_flag == 1){
        //Header package 
        packet_data[0] = CMD_GET_IMU_READINGS;
     
        for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++) 
        {	
            
            single_packet[0] = (uint8) 0x3A; //':';
            if (IMU_conf[IMU_connected[k_imu]][0]){
                *((int16 *) &single_packet[c]) = (int16) g_imu[k_imu].accel_value[0];
                *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].accel_value[1];
                *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].accel_value[2];
                c = c + 6;
            }
            if (IMU_conf[IMU_connected[k_imu]][1]){
                *((int16 *) &single_packet[c])   = (int16) g_imu[k_imu].gyro_value[0];
                *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].gyro_value[1];
                *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].gyro_value[2];
                c = c + 6;
            }
            if (IMU_conf[IMU_connected[k_imu]][2]){
                *((int16 *) &single_packet[c])   = (int16) g_imu[k_imu].mag_value[0];
                *((int16 *) &single_packet[c+2]) = (int16) g_imu[k_imu].mag_value[1];
                *((int16 *) &single_packet[c+4]) = (int16) g_imu[k_imu].mag_value[2];
                c = c + 6;
            }
            single_packet[single_imu_size[IMU_connected[k_imu]] - 1] = (uint8) 0x3A; //':';
            c = 1;
           
            
            for(k=0; k < single_imu_size[IMU_connected[k_imu]]; k++) {
                packet_data[gl_c + k] = (uint8) single_packet[k]; 
            }
            gl_c = gl_c + single_imu_size[IMU_connected[k_imu]];  
            
            memset(&single_packet, 0, sizeof(single_packet));     
            
        }

        // Calculate Checksum and send message to UART 
        packet_data[imus_data_size-1] = LCRChecksum (packet_data, imus_data_size-1);
        commWrite(packet_data, imus_data_size);
    
        imu_send_flag = 0;
    }
    
    /* `#END` */

    /* PSoC3 ES1, ES2 RTC ISR PATCH  */ 
    #if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
        #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (isr_imu__ES2_PATCH ))      
            isr_imu_ISR_PATCH();
        #endif /* CYDEV_CHIP_REVISION_USED */
    #endif /* (CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3) */
}


/*******************************************************************************
* Function Name: isr_imu_SetVector
********************************************************************************
*
* Summary:
*   Change the ISR vector for the Interrupt. Note calling isr_imu_Start
*   will override any effect this method would have had. To set the vector 
*   before the component has been started use isr_imu_StartEx instead.
* 
*   When defining ISR functions, the CY_ISR and CY_ISR_PROTO macros should be 
*   used to provide consistent definition across compilers:
*
*   Function definition example:
*   CY_ISR(MyISR)
*   {
*   }
*
*   Function prototype example:
*     CY_ISR_PROTO(MyISR);
*
* Parameters:
*   address: Address of the ISR to set in the interrupt vector table.
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_SetVector(cyisraddress address) 
{
    CY_SET_REG16(isr_imu_INTC_VECTOR, (uint16) address);
}


/*******************************************************************************
* Function Name: isr_imu_GetVector
********************************************************************************
*
* Summary:
*   Gets the "address" of the current ISR vector for the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   Address of the ISR in the interrupt vector table.
*
*******************************************************************************/
cyisraddress isr_imu_GetVector(void) 
{
    return (cyisraddress) CY_GET_REG16(isr_imu_INTC_VECTOR);
}


/*******************************************************************************
* Function Name: isr_imu_SetPriority
********************************************************************************
*
* Summary:
*   Sets the Priority of the Interrupt. 
*
*   Note calling isr_imu_Start or isr_imu_StartEx will 
*   override any effect this API would have had. This API should only be called
*   after isr_imu_Start or isr_imu_StartEx has been called. 
*   To set the initial priority for the component, use the Design-Wide Resources
*   Interrupt Editor.
*
*   Note This API has no effect on Non-maskable interrupt NMI).
*
* Parameters:
*   priority: Priority of the interrupt, 0 being the highest priority
*             PSoC 3 and PSoC 5LP: Priority is from 0 to 7.
*             PSoC 4: Priority is from 0 to 3.
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_SetPriority(uint8 priority) 
{
    *isr_imu_INTC_PRIOR = priority << 5;
}


/*******************************************************************************
* Function Name: isr_imu_GetPriority
********************************************************************************
*
* Summary:
*   Gets the Priority of the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   Priority of the interrupt, 0 being the highest priority
*    PSoC 3 and PSoC 5LP: Priority is from 0 to 7.
*    PSoC 4: Priority is from 0 to 3.
*
*******************************************************************************/
uint8 isr_imu_GetPriority(void) 
{
    uint8 priority;


    priority = *isr_imu_INTC_PRIOR >> 5;

    return priority;
}


/*******************************************************************************
* Function Name: isr_imu_Enable
********************************************************************************
*
* Summary:
*   Enables the interrupt to the interrupt controller. Do not call this function
*   unless ISR_Start() has been called or the functionality of the ISR_Start() 
*   function, which sets the vector and the priority, has been called.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_Enable(void) 
{
    /* Enable the general interrupt. */
    *isr_imu_INTC_SET_EN = isr_imu__INTC_MASK;
}


/*******************************************************************************
* Function Name: isr_imu_GetState
********************************************************************************
*
* Summary:
*   Gets the state (enabled, disabled) of the Interrupt.
*
* Parameters:
*   None
*
* Return:
*   1 if enabled, 0 if disabled.
*
*******************************************************************************/
uint8 isr_imu_GetState(void) 
{
    /* Get the state of the general interrupt. */
    return ((*isr_imu_INTC_SET_EN & (uint8)isr_imu__INTC_MASK) != 0u) ? 1u:0u;
}


/*******************************************************************************
* Function Name: isr_imu_Disable
********************************************************************************
*
* Summary:
*   Disables the Interrupt in the interrupt controller.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_Disable(void) 
{
    /* Disable the general interrupt. */
    *isr_imu_INTC_CLR_EN = isr_imu__INTC_MASK;
}


/*******************************************************************************
* Function Name: isr_imu_SetPending
********************************************************************************
*
* Summary:
*   Causes the Interrupt to enter the pending state, a software method of
*   generating the interrupt.
*
* Parameters:
*   None
*
* Return:
*   None
*
* Side Effects:
*   If interrupts are enabled and the interrupt is set up properly, the ISR is
*   entered (depending on the priority of this interrupt and other pending 
*   interrupts).
*
*******************************************************************************/
void isr_imu_SetPending(void) 
{
    *isr_imu_INTC_SET_PD = isr_imu__INTC_MASK;
}


/*******************************************************************************
* Function Name: isr_imu_ClearPending
********************************************************************************
*
* Summary:
*   Clears a pending interrupt in the interrupt controller.
*
*   Note Some interrupt sources are clear-on-read and require the block 
*   interrupt/status register to be read/cleared with the appropriate block API 
*   (GPIO, UART, and so on). Otherwise the ISR will continue to remain in 
*   pending state even though the interrupt itself is cleared using this API.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void isr_imu_ClearPending(void) 
{
    *isr_imu_INTC_CLR_PD = isr_imu__INTC_MASK;
}

#endif /* End check for removal by optimization */


/* [] END OF FILE */
