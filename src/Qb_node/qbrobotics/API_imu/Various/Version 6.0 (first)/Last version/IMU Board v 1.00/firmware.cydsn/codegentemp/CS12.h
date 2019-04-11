/*******************************************************************************
* File Name: CS12.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_CS12_H) /* Pins CS12_H */
#define CY_PINS_CS12_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS12_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS12_Write(uint8 value) ;
void    CS12_SetDriveMode(uint8 mode) ;
uint8   CS12_ReadDataReg(void) ;
uint8   CS12_Read(void) ;
void    CS12_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS12_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS12_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS12_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS12_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS12_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS12_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS12_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS12_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS12_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS12_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS12_MASK               CS12__MASK
#define CS12_SHIFT              CS12__SHIFT
#define CS12_WIDTH              1u

/* Interrupt constants */
#if defined(CS12__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS12_SetInterruptMode() function.
     *  @{
     */
        #define CS12_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS12_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS12_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS12_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS12_INTR_MASK      (0x01u)
#endif /* (CS12__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS12_PS                     (* (reg8 *) CS12__PS)
/* Data Register */
#define CS12_DR                     (* (reg8 *) CS12__DR)
/* Port Number */
#define CS12_PRT_NUM                (* (reg8 *) CS12__PRT) 
/* Connect to Analog Globals */                                                  
#define CS12_AG                     (* (reg8 *) CS12__AG)                       
/* Analog MUX bux enable */
#define CS12_AMUX                   (* (reg8 *) CS12__AMUX) 
/* Bidirectional Enable */                                                        
#define CS12_BIE                    (* (reg8 *) CS12__BIE)
/* Bit-mask for Aliased Register Access */
#define CS12_BIT_MASK               (* (reg8 *) CS12__BIT_MASK)
/* Bypass Enable */
#define CS12_BYP                    (* (reg8 *) CS12__BYP)
/* Port wide control signals */                                                   
#define CS12_CTL                    (* (reg8 *) CS12__CTL)
/* Drive Modes */
#define CS12_DM0                    (* (reg8 *) CS12__DM0) 
#define CS12_DM1                    (* (reg8 *) CS12__DM1)
#define CS12_DM2                    (* (reg8 *) CS12__DM2) 
/* Input Buffer Disable Override */
#define CS12_INP_DIS                (* (reg8 *) CS12__INP_DIS)
/* LCD Common or Segment Drive */
#define CS12_LCD_COM_SEG            (* (reg8 *) CS12__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS12_LCD_EN                 (* (reg8 *) CS12__LCD_EN)
/* Slew Rate Control */
#define CS12_SLW                    (* (reg8 *) CS12__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS12_PRTDSI__CAPS_SEL       (* (reg8 *) CS12__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS12_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS12__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS12_PRTDSI__OE_SEL0        (* (reg8 *) CS12__PRTDSI__OE_SEL0) 
#define CS12_PRTDSI__OE_SEL1        (* (reg8 *) CS12__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS12_PRTDSI__OUT_SEL0       (* (reg8 *) CS12__PRTDSI__OUT_SEL0) 
#define CS12_PRTDSI__OUT_SEL1       (* (reg8 *) CS12__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS12_PRTDSI__SYNC_OUT       (* (reg8 *) CS12__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS12__SIO_CFG)
    #define CS12_SIO_HYST_EN        (* (reg8 *) CS12__SIO_HYST_EN)
    #define CS12_SIO_REG_HIFREQ     (* (reg8 *) CS12__SIO_REG_HIFREQ)
    #define CS12_SIO_CFG            (* (reg8 *) CS12__SIO_CFG)
    #define CS12_SIO_DIFF           (* (reg8 *) CS12__SIO_DIFF)
#endif /* (CS12__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS12__INTSTAT)
    #define CS12_INTSTAT             (* (reg8 *) CS12__INTSTAT)
    #define CS12_SNAP                (* (reg8 *) CS12__SNAP)
    
	#define CS12_0_INTTYPE_REG 		(* (reg8 *) CS12__0__INTTYPE)
#endif /* (CS12__INTSTAT) */

#endif /* End Pins CS12_H */


/* [] END OF FILE */
