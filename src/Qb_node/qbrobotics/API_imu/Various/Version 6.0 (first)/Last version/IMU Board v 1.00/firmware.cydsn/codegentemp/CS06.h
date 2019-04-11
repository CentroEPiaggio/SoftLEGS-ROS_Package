/*******************************************************************************
* File Name: CS06.h  
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

#if !defined(CY_PINS_CS06_H) /* Pins CS06_H */
#define CY_PINS_CS06_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS06_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS06_Write(uint8 value) ;
void    CS06_SetDriveMode(uint8 mode) ;
uint8   CS06_ReadDataReg(void) ;
uint8   CS06_Read(void) ;
void    CS06_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS06_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS06_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS06_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS06_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS06_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS06_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS06_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS06_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS06_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS06_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS06_MASK               CS06__MASK
#define CS06_SHIFT              CS06__SHIFT
#define CS06_WIDTH              1u

/* Interrupt constants */
#if defined(CS06__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS06_SetInterruptMode() function.
     *  @{
     */
        #define CS06_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS06_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS06_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS06_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS06_INTR_MASK      (0x01u)
#endif /* (CS06__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS06_PS                     (* (reg8 *) CS06__PS)
/* Data Register */
#define CS06_DR                     (* (reg8 *) CS06__DR)
/* Port Number */
#define CS06_PRT_NUM                (* (reg8 *) CS06__PRT) 
/* Connect to Analog Globals */                                                  
#define CS06_AG                     (* (reg8 *) CS06__AG)                       
/* Analog MUX bux enable */
#define CS06_AMUX                   (* (reg8 *) CS06__AMUX) 
/* Bidirectional Enable */                                                        
#define CS06_BIE                    (* (reg8 *) CS06__BIE)
/* Bit-mask for Aliased Register Access */
#define CS06_BIT_MASK               (* (reg8 *) CS06__BIT_MASK)
/* Bypass Enable */
#define CS06_BYP                    (* (reg8 *) CS06__BYP)
/* Port wide control signals */                                                   
#define CS06_CTL                    (* (reg8 *) CS06__CTL)
/* Drive Modes */
#define CS06_DM0                    (* (reg8 *) CS06__DM0) 
#define CS06_DM1                    (* (reg8 *) CS06__DM1)
#define CS06_DM2                    (* (reg8 *) CS06__DM2) 
/* Input Buffer Disable Override */
#define CS06_INP_DIS                (* (reg8 *) CS06__INP_DIS)
/* LCD Common or Segment Drive */
#define CS06_LCD_COM_SEG            (* (reg8 *) CS06__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS06_LCD_EN                 (* (reg8 *) CS06__LCD_EN)
/* Slew Rate Control */
#define CS06_SLW                    (* (reg8 *) CS06__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS06_PRTDSI__CAPS_SEL       (* (reg8 *) CS06__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS06_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS06__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS06_PRTDSI__OE_SEL0        (* (reg8 *) CS06__PRTDSI__OE_SEL0) 
#define CS06_PRTDSI__OE_SEL1        (* (reg8 *) CS06__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS06_PRTDSI__OUT_SEL0       (* (reg8 *) CS06__PRTDSI__OUT_SEL0) 
#define CS06_PRTDSI__OUT_SEL1       (* (reg8 *) CS06__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS06_PRTDSI__SYNC_OUT       (* (reg8 *) CS06__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS06__SIO_CFG)
    #define CS06_SIO_HYST_EN        (* (reg8 *) CS06__SIO_HYST_EN)
    #define CS06_SIO_REG_HIFREQ     (* (reg8 *) CS06__SIO_REG_HIFREQ)
    #define CS06_SIO_CFG            (* (reg8 *) CS06__SIO_CFG)
    #define CS06_SIO_DIFF           (* (reg8 *) CS06__SIO_DIFF)
#endif /* (CS06__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS06__INTSTAT)
    #define CS06_INTSTAT             (* (reg8 *) CS06__INTSTAT)
    #define CS06_SNAP                (* (reg8 *) CS06__SNAP)
    
	#define CS06_0_INTTYPE_REG 		(* (reg8 *) CS06__0__INTTYPE)
#endif /* (CS06__INTSTAT) */

#endif /* End Pins CS06_H */


/* [] END OF FILE */
