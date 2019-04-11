/*******************************************************************************
* File Name: CS16.h  
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

#if !defined(CY_PINS_CS16_H) /* Pins CS16_H */
#define CY_PINS_CS16_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS16_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS16_Write(uint8 value) ;
void    CS16_SetDriveMode(uint8 mode) ;
uint8   CS16_ReadDataReg(void) ;
uint8   CS16_Read(void) ;
void    CS16_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS16_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS16_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS16_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS16_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS16_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS16_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS16_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS16_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS16_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS16_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS16_MASK               CS16__MASK
#define CS16_SHIFT              CS16__SHIFT
#define CS16_WIDTH              1u

/* Interrupt constants */
#if defined(CS16__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS16_SetInterruptMode() function.
     *  @{
     */
        #define CS16_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS16_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS16_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS16_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS16_INTR_MASK      (0x01u)
#endif /* (CS16__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS16_PS                     (* (reg8 *) CS16__PS)
/* Data Register */
#define CS16_DR                     (* (reg8 *) CS16__DR)
/* Port Number */
#define CS16_PRT_NUM                (* (reg8 *) CS16__PRT) 
/* Connect to Analog Globals */                                                  
#define CS16_AG                     (* (reg8 *) CS16__AG)                       
/* Analog MUX bux enable */
#define CS16_AMUX                   (* (reg8 *) CS16__AMUX) 
/* Bidirectional Enable */                                                        
#define CS16_BIE                    (* (reg8 *) CS16__BIE)
/* Bit-mask for Aliased Register Access */
#define CS16_BIT_MASK               (* (reg8 *) CS16__BIT_MASK)
/* Bypass Enable */
#define CS16_BYP                    (* (reg8 *) CS16__BYP)
/* Port wide control signals */                                                   
#define CS16_CTL                    (* (reg8 *) CS16__CTL)
/* Drive Modes */
#define CS16_DM0                    (* (reg8 *) CS16__DM0) 
#define CS16_DM1                    (* (reg8 *) CS16__DM1)
#define CS16_DM2                    (* (reg8 *) CS16__DM2) 
/* Input Buffer Disable Override */
#define CS16_INP_DIS                (* (reg8 *) CS16__INP_DIS)
/* LCD Common or Segment Drive */
#define CS16_LCD_COM_SEG            (* (reg8 *) CS16__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS16_LCD_EN                 (* (reg8 *) CS16__LCD_EN)
/* Slew Rate Control */
#define CS16_SLW                    (* (reg8 *) CS16__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS16_PRTDSI__CAPS_SEL       (* (reg8 *) CS16__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS16_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS16__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS16_PRTDSI__OE_SEL0        (* (reg8 *) CS16__PRTDSI__OE_SEL0) 
#define CS16_PRTDSI__OE_SEL1        (* (reg8 *) CS16__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS16_PRTDSI__OUT_SEL0       (* (reg8 *) CS16__PRTDSI__OUT_SEL0) 
#define CS16_PRTDSI__OUT_SEL1       (* (reg8 *) CS16__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS16_PRTDSI__SYNC_OUT       (* (reg8 *) CS16__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS16__SIO_CFG)
    #define CS16_SIO_HYST_EN        (* (reg8 *) CS16__SIO_HYST_EN)
    #define CS16_SIO_REG_HIFREQ     (* (reg8 *) CS16__SIO_REG_HIFREQ)
    #define CS16_SIO_CFG            (* (reg8 *) CS16__SIO_CFG)
    #define CS16_SIO_DIFF           (* (reg8 *) CS16__SIO_DIFF)
#endif /* (CS16__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS16__INTSTAT)
    #define CS16_INTSTAT             (* (reg8 *) CS16__INTSTAT)
    #define CS16_SNAP                (* (reg8 *) CS16__SNAP)
    
	#define CS16_0_INTTYPE_REG 		(* (reg8 *) CS16__0__INTTYPE)
#endif /* (CS16__INTSTAT) */

#endif /* End Pins CS16_H */


/* [] END OF FILE */
