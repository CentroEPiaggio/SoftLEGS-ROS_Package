/*******************************************************************************
* File Name: CS00.h  
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

#if !defined(CY_PINS_CS00_H) /* Pins CS00_H */
#define CY_PINS_CS00_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS00_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS00_Write(uint8 value) ;
void    CS00_SetDriveMode(uint8 mode) ;
uint8   CS00_ReadDataReg(void) ;
uint8   CS00_Read(void) ;
void    CS00_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS00_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS00_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS00_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS00_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS00_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS00_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS00_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS00_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS00_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS00_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS00_MASK               CS00__MASK
#define CS00_SHIFT              CS00__SHIFT
#define CS00_WIDTH              1u

/* Interrupt constants */
#if defined(CS00__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS00_SetInterruptMode() function.
     *  @{
     */
        #define CS00_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS00_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS00_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS00_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS00_INTR_MASK      (0x01u)
#endif /* (CS00__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS00_PS                     (* (reg8 *) CS00__PS)
/* Data Register */
#define CS00_DR                     (* (reg8 *) CS00__DR)
/* Port Number */
#define CS00_PRT_NUM                (* (reg8 *) CS00__PRT) 
/* Connect to Analog Globals */                                                  
#define CS00_AG                     (* (reg8 *) CS00__AG)                       
/* Analog MUX bux enable */
#define CS00_AMUX                   (* (reg8 *) CS00__AMUX) 
/* Bidirectional Enable */                                                        
#define CS00_BIE                    (* (reg8 *) CS00__BIE)
/* Bit-mask for Aliased Register Access */
#define CS00_BIT_MASK               (* (reg8 *) CS00__BIT_MASK)
/* Bypass Enable */
#define CS00_BYP                    (* (reg8 *) CS00__BYP)
/* Port wide control signals */                                                   
#define CS00_CTL                    (* (reg8 *) CS00__CTL)
/* Drive Modes */
#define CS00_DM0                    (* (reg8 *) CS00__DM0) 
#define CS00_DM1                    (* (reg8 *) CS00__DM1)
#define CS00_DM2                    (* (reg8 *) CS00__DM2) 
/* Input Buffer Disable Override */
#define CS00_INP_DIS                (* (reg8 *) CS00__INP_DIS)
/* LCD Common or Segment Drive */
#define CS00_LCD_COM_SEG            (* (reg8 *) CS00__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS00_LCD_EN                 (* (reg8 *) CS00__LCD_EN)
/* Slew Rate Control */
#define CS00_SLW                    (* (reg8 *) CS00__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS00_PRTDSI__CAPS_SEL       (* (reg8 *) CS00__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS00_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS00__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS00_PRTDSI__OE_SEL0        (* (reg8 *) CS00__PRTDSI__OE_SEL0) 
#define CS00_PRTDSI__OE_SEL1        (* (reg8 *) CS00__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS00_PRTDSI__OUT_SEL0       (* (reg8 *) CS00__PRTDSI__OUT_SEL0) 
#define CS00_PRTDSI__OUT_SEL1       (* (reg8 *) CS00__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS00_PRTDSI__SYNC_OUT       (* (reg8 *) CS00__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS00__SIO_CFG)
    #define CS00_SIO_HYST_EN        (* (reg8 *) CS00__SIO_HYST_EN)
    #define CS00_SIO_REG_HIFREQ     (* (reg8 *) CS00__SIO_REG_HIFREQ)
    #define CS00_SIO_CFG            (* (reg8 *) CS00__SIO_CFG)
    #define CS00_SIO_DIFF           (* (reg8 *) CS00__SIO_DIFF)
#endif /* (CS00__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS00__INTSTAT)
    #define CS00_INTSTAT             (* (reg8 *) CS00__INTSTAT)
    #define CS00_SNAP                (* (reg8 *) CS00__SNAP)
    
	#define CS00_0_INTTYPE_REG 		(* (reg8 *) CS00__0__INTTYPE)
#endif /* (CS00__INTSTAT) */

#endif /* End Pins CS00_H */


/* [] END OF FILE */
