/*******************************************************************************
* File Name: isr_clock.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_isr_clock_H)
#define CY_CLOCK_isr_clock_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
* Conditional Compilation Parameters
***************************************/

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component cy_clock_v2_20 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

void isr_clock_Start(void) ;
void isr_clock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void isr_clock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void isr_clock_StandbyPower(uint8 state) ;
void isr_clock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 isr_clock_GetDividerRegister(void) ;
void isr_clock_SetModeRegister(uint8 modeBitMask) ;
void isr_clock_ClearModeRegister(uint8 modeBitMask) ;
uint8 isr_clock_GetModeRegister(void) ;
void isr_clock_SetSourceRegister(uint8 clkSource) ;
uint8 isr_clock_GetSourceRegister(void) ;
#if defined(isr_clock__CFG3)
void isr_clock_SetPhaseRegister(uint8 clkPhase) ;
uint8 isr_clock_GetPhaseRegister(void) ;
#endif /* defined(isr_clock__CFG3) */

#define isr_clock_Enable()                       isr_clock_Start()
#define isr_clock_Disable()                      isr_clock_Stop()
#define isr_clock_SetDivider(clkDivider)         isr_clock_SetDividerRegister(clkDivider, 1u)
#define isr_clock_SetDividerValue(clkDivider)    isr_clock_SetDividerRegister((clkDivider) - 1u, 1u)
#define isr_clock_SetMode(clkMode)               isr_clock_SetModeRegister(clkMode)
#define isr_clock_SetSource(clkSource)           isr_clock_SetSourceRegister(clkSource)
#if defined(isr_clock__CFG3)
#define isr_clock_SetPhase(clkPhase)             isr_clock_SetPhaseRegister(clkPhase)
#define isr_clock_SetPhaseValue(clkPhase)        isr_clock_SetPhaseRegister((clkPhase) + 1u)
#endif /* defined(isr_clock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define isr_clock_CLKEN              (* (reg8 *) isr_clock__PM_ACT_CFG)
#define isr_clock_CLKEN_PTR          ((reg8 *) isr_clock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define isr_clock_CLKSTBY            (* (reg8 *) isr_clock__PM_STBY_CFG)
#define isr_clock_CLKSTBY_PTR        ((reg8 *) isr_clock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define isr_clock_DIV_LSB            (* (reg8 *) isr_clock__CFG0)
#define isr_clock_DIV_LSB_PTR        ((reg8 *) isr_clock__CFG0)
#define isr_clock_DIV_PTR            ((reg16 *) isr_clock__CFG0)

/* Clock MSB divider configuration register. */
#define isr_clock_DIV_MSB            (* (reg8 *) isr_clock__CFG1)
#define isr_clock_DIV_MSB_PTR        ((reg8 *) isr_clock__CFG1)

/* Mode and source configuration register */
#define isr_clock_MOD_SRC            (* (reg8 *) isr_clock__CFG2)
#define isr_clock_MOD_SRC_PTR        ((reg8 *) isr_clock__CFG2)

#if defined(isr_clock__CFG3)
/* Analog clock phase configuration register */
#define isr_clock_PHASE              (* (reg8 *) isr_clock__CFG3)
#define isr_clock_PHASE_PTR          ((reg8 *) isr_clock__CFG3)
#endif /* defined(isr_clock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define isr_clock_CLKEN_MASK         isr_clock__PM_ACT_MSK
#define isr_clock_CLKSTBY_MASK       isr_clock__PM_STBY_MSK

/* CFG2 field masks */
#define isr_clock_SRC_SEL_MSK        isr_clock__CFG2_SRC_SEL_MASK
#define isr_clock_MODE_MASK          (~(isr_clock_SRC_SEL_MSK))

#if defined(isr_clock__CFG3)
/* CFG3 phase mask */
#define isr_clock_PHASE_MASK         isr_clock__CFG3_PHASE_DLY_MASK
#endif /* defined(isr_clock__CFG3) */

#endif /* CY_CLOCK_isr_clock_H */


/* [] END OF FILE */
