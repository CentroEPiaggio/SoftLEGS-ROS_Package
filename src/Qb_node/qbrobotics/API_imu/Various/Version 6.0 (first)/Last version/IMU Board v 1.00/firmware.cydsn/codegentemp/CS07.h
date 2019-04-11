/*******************************************************************************
* File Name: CS07.h  
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

#if !defined(CY_PINS_CS07_H) /* Pins CS07_H */
#define CY_PINS_CS07_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS07_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS07_Write(uint8 value) ;
void    CS07_SetDriveMode(uint8 mode) ;
uint8   CS07_ReadDataReg(void) ;
uint8   CS07_Read(void) ;
void    CS07_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS07_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS07_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS07_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS07_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS07_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS07_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS07_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS07_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS07_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS07_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS07_MASK               CS07__MASK
#define CS07_SHIFT              CS07__SHIFT
#define CS07_WIDTH              1u

/* Interrupt constants */
#if defined(CS07__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS07_SetInterruptMode() function.
     *  @{
     */
        #define CS07_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS07_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS07_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS07_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS07_INTR_MASK      (0x01u)
#endif /* (CS07__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS07_PS                     (* (reg8 *) CS07__PS)
/* Data Register */
#define CS07_DR                     (* (reg8 *) CS07__DR)
/* Port Number */
#define CS07_PRT_NUM                (* (reg8 *) CS07__PRT) 
/* Connect to Analog Globals */                                                  
#define CS07_AG                     (* (reg8 *) CS07__AG)                       
/* Analog MUX bux enable */
#define CS07_AMUX                   (* (reg8 *) CS07__AMUX) 
/* Bidirectional Enable */                                                        
#define CS07_BIE                    (* (reg8 *) CS07__BIE)
/* Bit-mask for Aliased Register Access */
#define CS07_BIT_MASK               (* (reg8 *) CS07__BIT_MASK)
/* Bypass Enable */
#define CS07_BYP                    (* (reg8 *) CS07__BYP)
/* Port wide control signals */                                                   
#define CS07_CTL                    (* (reg8 *) CS07__CTL)
/* Drive Modes */
#define CS07_DM0                    (* (reg8 *) CS07__DM0) 
#define CS07_DM1                    (* (reg8 *) CS07__DM1)
#define CS07_DM2                    (* (reg8 *) CS07__DM2) 
/* Input Buffer Disable Override */
#define CS07_INP_DIS                (* (reg8 *) CS07__INP_DIS)
/* LCD Common or Segment Drive */
#define CS07_LCD_COM_SEG            (* (reg8 *) CS07__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS07_LCD_EN                 (* (reg8 *) CS07__LCD_EN)
/* Slew Rate Control */
#define CS07_SLW                    (* (reg8 *) CS07__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS07_PRTDSI__CAPS_SEL       (* (reg8 *) CS07__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS07_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS07__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS07_PRTDSI__OE_SEL0        (* (reg8 *) CS07__PRTDSI__OE_SEL0) 
#define CS07_PRTDSI__OE_SEL1        (* (reg8 *) CS07__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS07_PRTDSI__OUT_SEL0       (* (reg8 *) CS07__PRTDSI__OUT_SEL0) 
#define CS07_PRTDSI__OUT_SEL1       (* (reg8 *) CS07__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS07_PRTDSI__SYNC_OUT       (* (reg8 *) CS07__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS07__SIO_CFG)
    #define CS07_SIO_HYST_EN        (* (reg8 *) CS07__SIO_HYST_EN)
    #define CS07_SIO_REG_HIFREQ     (* (reg8 *) CS07__SIO_REG_HIFREQ)
    #define CS07_SIO_CFG            (* (reg8 *) CS07__SIO_CFG)
    #define CS07_SIO_DIFF           (* (reg8 *) CS07__SIO_DIFF)
#endif /* (CS07__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS07__INTSTAT)
    #define CS07_INTSTAT             (* (reg8 *) CS07__INTSTAT)
    #define CS07_SNAP                (* (reg8 *) CS07__SNAP)
    
	#define CS07_0_INTTYPE_REG 		(* (reg8 *) CS07__0__INTTYPE)
#endif /* (CS07__INTSTAT) */

#endif /* End Pins CS07_H */


/* [] END OF FILE */
