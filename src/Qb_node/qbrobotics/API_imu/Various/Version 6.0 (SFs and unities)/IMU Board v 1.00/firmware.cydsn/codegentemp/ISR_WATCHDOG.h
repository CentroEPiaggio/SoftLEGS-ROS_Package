/*******************************************************************************
* File Name: ISR_WATCHDOG.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_ISR_ISR_WATCHDOG_H)
#define CY_ISR_ISR_WATCHDOG_H

#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void ISR_WATCHDOG_Start(void) ;
void ISR_WATCHDOG_StartEx(cyisraddress address) ;
void ISR_WATCHDOG_Stop(void) ;

CY_ISR_PROTO(ISR_WATCHDOG_Interrupt);

void ISR_WATCHDOG_SetVector(cyisraddress address) ;
cyisraddress ISR_WATCHDOG_GetVector(void) ;

void ISR_WATCHDOG_SetPriority(uint8 priority) ;
uint8 ISR_WATCHDOG_GetPriority(void) ;

void ISR_WATCHDOG_Enable(void) ;
uint8 ISR_WATCHDOG_GetState(void) ;
void ISR_WATCHDOG_Disable(void) ;

void ISR_WATCHDOG_SetPending(void) ;
void ISR_WATCHDOG_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the ISR_WATCHDOG ISR. */
#define ISR_WATCHDOG_INTC_VECTOR            ((reg16 *) ISR_WATCHDOG__INTC_VECT)

/* Address of the ISR_WATCHDOG ISR priority. */
#define ISR_WATCHDOG_INTC_PRIOR             ((reg8 *) ISR_WATCHDOG__INTC_PRIOR_REG)

/* Priority of the ISR_WATCHDOG interrupt. */
#define ISR_WATCHDOG_INTC_PRIOR_NUMBER      ISR_WATCHDOG__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable ISR_WATCHDOG interrupt. */
#define ISR_WATCHDOG_INTC_SET_EN            ((reg8 *) ISR_WATCHDOG__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the ISR_WATCHDOG interrupt. */
#define ISR_WATCHDOG_INTC_CLR_EN            ((reg8 *) ISR_WATCHDOG__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the ISR_WATCHDOG interrupt state to pending. */
#define ISR_WATCHDOG_INTC_SET_PD            ((reg8 *) ISR_WATCHDOG__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the ISR_WATCHDOG interrupt. */
#define ISR_WATCHDOG_INTC_CLR_PD            ((reg8 *) ISR_WATCHDOG__INTC_CLR_PD_REG)



#endif /* CY_ISR_ISR_WATCHDOG_H */


/* [] END OF FILE */
