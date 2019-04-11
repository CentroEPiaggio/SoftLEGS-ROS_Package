/*******************************************************************************
* File Name: ISR_WATCHDOG.c  
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
#include <ISR_WATCHDOG.h>


#if !defined(ISR_WATCHDOG__REMOVED) /* Check for removal by optimization */

/*******************************************************************************
*  Place your includes, defines and code here 
********************************************************************************/
/* `#START ISR_WATCHDOG_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: ISR_WATCHDOG_Start
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
void ISR_WATCHDOG_Start(void) 
{
    /* For all we know the interrupt is active. */
    ISR_WATCHDOG_Disable();

    /* Set the ISR to point to the ISR_WATCHDOG Interrupt. */
    ISR_WATCHDOG_SetVector(&ISR_WATCHDOG_Interrupt);

    /* Set the priority. */
    ISR_WATCHDOG_SetPriority((uint8)ISR_WATCHDOG_INTC_PRIOR_NUMBER);

    /* Enable it. */
    ISR_WATCHDOG_Enable();
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_StartEx
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
void ISR_WATCHDOG_StartEx(cyisraddress address) 
{
    /* For all we know the interrupt is active. */
    ISR_WATCHDOG_Disable();

    /* Set the ISR to point to the ISR_WATCHDOG Interrupt. */
    ISR_WATCHDOG_SetVector(address);

    /* Set the priority. */
    ISR_WATCHDOG_SetPriority((uint8)ISR_WATCHDOG_INTC_PRIOR_NUMBER);

    /* Enable it. */
    ISR_WATCHDOG_Enable();
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_Stop
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
void ISR_WATCHDOG_Stop(void) 
{
    /* Disable this interrupt. */
    ISR_WATCHDOG_Disable();
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_Interrupt
********************************************************************************
*
* Summary:
*   The default Interrupt Service Routine for ISR_WATCHDOG.
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
CY_ISR(ISR_WATCHDOG_Interrupt)
{
    #ifdef ISR_WATCHDOG_INTERRUPT_INTERRUPT_CALLBACK
        ISR_WATCHDOG_Interrupt_InterruptCallback();
    #endif /* ISR_WATCHDOG_INTERRUPT_INTERRUPT_CALLBACK */

    /*  Place your Interrupt code here. */
    /* `#START ISR_WATCHDOG_Interrupt` */

    /* `#END` */

    /* PSoC3 ES1, ES2 RTC ISR PATCH  */ 
    #if(CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3)
        #if((CYDEV_CHIP_REVISION_USED <= CYDEV_CHIP_REVISION_3A_ES2) && (ISR_WATCHDOG__ES2_PATCH ))      
            ISR_WATCHDOG_ISR_PATCH();
        #endif /* CYDEV_CHIP_REVISION_USED */
    #endif /* (CYDEV_CHIP_FAMILY_USED == CYDEV_CHIP_FAMILY_PSOC3) */
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_SetVector
********************************************************************************
*
* Summary:
*   Change the ISR vector for the Interrupt. Note calling ISR_WATCHDOG_Start
*   will override any effect this method would have had. To set the vector 
*   before the component has been started use ISR_WATCHDOG_StartEx instead.
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
void ISR_WATCHDOG_SetVector(cyisraddress address) 
{
    CY_SET_REG16(ISR_WATCHDOG_INTC_VECTOR, (uint16) address);
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_GetVector
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
cyisraddress ISR_WATCHDOG_GetVector(void) 
{
    return (cyisraddress) CY_GET_REG16(ISR_WATCHDOG_INTC_VECTOR);
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_SetPriority
********************************************************************************
*
* Summary:
*   Sets the Priority of the Interrupt. 
*
*   Note calling ISR_WATCHDOG_Start or ISR_WATCHDOG_StartEx will 
*   override any effect this API would have had. This API should only be called
*   after ISR_WATCHDOG_Start or ISR_WATCHDOG_StartEx has been called. 
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
void ISR_WATCHDOG_SetPriority(uint8 priority) 
{
    *ISR_WATCHDOG_INTC_PRIOR = priority << 5;
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_GetPriority
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
uint8 ISR_WATCHDOG_GetPriority(void) 
{
    uint8 priority;


    priority = *ISR_WATCHDOG_INTC_PRIOR >> 5;

    return priority;
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_Enable
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
void ISR_WATCHDOG_Enable(void) 
{
    /* Enable the general interrupt. */
    *ISR_WATCHDOG_INTC_SET_EN = ISR_WATCHDOG__INTC_MASK;
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_GetState
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
uint8 ISR_WATCHDOG_GetState(void) 
{
    /* Get the state of the general interrupt. */
    return ((*ISR_WATCHDOG_INTC_SET_EN & (uint8)ISR_WATCHDOG__INTC_MASK) != 0u) ? 1u:0u;
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_Disable
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
void ISR_WATCHDOG_Disable(void) 
{
    /* Disable the general interrupt. */
    *ISR_WATCHDOG_INTC_CLR_EN = ISR_WATCHDOG__INTC_MASK;
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_SetPending
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
void ISR_WATCHDOG_SetPending(void) 
{
    *ISR_WATCHDOG_INTC_SET_PD = ISR_WATCHDOG__INTC_MASK;
}


/*******************************************************************************
* Function Name: ISR_WATCHDOG_ClearPending
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
void ISR_WATCHDOG_ClearPending(void) 
{
    *ISR_WATCHDOG_INTC_CLR_PD = ISR_WATCHDOG__INTC_MASK;
}

#endif /* End check for removal by optimization */


/* [] END OF FILE */
