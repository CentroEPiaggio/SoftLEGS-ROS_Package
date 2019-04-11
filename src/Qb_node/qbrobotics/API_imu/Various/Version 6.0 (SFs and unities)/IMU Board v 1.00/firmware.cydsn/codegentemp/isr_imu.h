/*******************************************************************************
* File Name: isr_imu.h
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

#if !defined(CY_ISR_isr_imu_H)
#define CY_ISR_isr_imu_H

#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void isr_imu_Start(void) ;
void isr_imu_StartEx(cyisraddress address) ;
void isr_imu_Stop(void) ;

CY_ISR_PROTO(isr_imu_Interrupt);

void isr_imu_SetVector(cyisraddress address) ;
cyisraddress isr_imu_GetVector(void) ;

void isr_imu_SetPriority(uint8 priority) ;
uint8 isr_imu_GetPriority(void) ;

void isr_imu_Enable(void) ;
uint8 isr_imu_GetState(void) ;
void isr_imu_Disable(void) ;

void isr_imu_SetPending(void) ;
void isr_imu_ClearPending(void) ;


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the isr_imu ISR. */
#define isr_imu_INTC_VECTOR            ((reg16 *) isr_imu__INTC_VECT)

/* Address of the isr_imu ISR priority. */
#define isr_imu_INTC_PRIOR             ((reg8 *) isr_imu__INTC_PRIOR_REG)

/* Priority of the isr_imu interrupt. */
#define isr_imu_INTC_PRIOR_NUMBER      isr_imu__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable isr_imu interrupt. */
#define isr_imu_INTC_SET_EN            ((reg8 *) isr_imu__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the isr_imu interrupt. */
#define isr_imu_INTC_CLR_EN            ((reg8 *) isr_imu__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the isr_imu interrupt state to pending. */
#define isr_imu_INTC_SET_PD            ((reg8 *) isr_imu__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the isr_imu interrupt. */
#define isr_imu_INTC_CLR_PD            ((reg8 *) isr_imu__INTC_CLR_PD_REG)



#endif /* CY_ISR_isr_imu_H */


/* [] END OF FILE */
