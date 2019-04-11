/*******************************************************************************
* File Name: CS05.h  
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

#if !defined(CY_PINS_CS05_H) /* Pins CS05_H */
#define CY_PINS_CS05_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS05_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS05_Write(uint8 value) ;
void    CS05_SetDriveMode(uint8 mode) ;
uint8   CS05_ReadDataReg(void) ;
uint8   CS05_Read(void) ;
void    CS05_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS05_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS05_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS05_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS05_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS05_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS05_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS05_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS05_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS05_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS05_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS05_MASK               CS05__MASK
#define CS05_SHIFT              CS05__SHIFT
#define CS05_WIDTH              1u

/* Interrupt constants */
#if defined(CS05__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS05_SetInterruptMode() function.
     *  @{
     */
        #define CS05_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS05_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS05_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS05_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS05_INTR_MASK      (0x01u)
#endif /* (CS05__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS05_PS                     (* (reg8 *) CS05__PS)
/* Data Register */
#define CS05_DR                     (* (reg8 *) CS05__DR)
/* Port Number */
#define CS05_PRT_NUM                (* (reg8 *) CS05__PRT) 
/* Connect to Analog Globals */                                                  
#define CS05_AG                     (* (reg8 *) CS05__AG)                       
/* Analog MUX bux enable */
#define CS05_AMUX                   (* (reg8 *) CS05__AMUX) 
/* Bidirectional Enable */                                                        
#define CS05_BIE                    (* (reg8 *) CS05__BIE)
/* Bit-mask for Aliased Register Access */
#define CS05_BIT_MASK               (* (reg8 *) CS05__BIT_MASK)
/* Bypass Enable */
#define CS05_BYP                    (* (reg8 *) CS05__BYP)
/* Port wide control signals */                                                   
#define CS05_CTL                    (* (reg8 *) CS05__CTL)
/* Drive Modes */
#define CS05_DM0                    (* (reg8 *) CS05__DM0) 
#define CS05_DM1                    (* (reg8 *) CS05__DM1)
#define CS05_DM2                    (* (reg8 *) CS05__DM2) 
/* Input Buffer Disable Override */
#define CS05_INP_DIS                (* (reg8 *) CS05__INP_DIS)
/* LCD Common or Segment Drive */
#define CS05_LCD_COM_SEG            (* (reg8 *) CS05__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS05_LCD_EN                 (* (reg8 *) CS05__LCD_EN)
/* Slew Rate Control */
#define CS05_SLW                    (* (reg8 *) CS05__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS05_PRTDSI__CAPS_SEL       (* (reg8 *) CS05__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS05_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS05__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS05_PRTDSI__OE_SEL0        (* (reg8 *) CS05__PRTDSI__OE_SEL0) 
#define CS05_PRTDSI__OE_SEL1        (* (reg8 *) CS05__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS05_PRTDSI__OUT_SEL0       (* (reg8 *) CS05__PRTDSI__OUT_SEL0) 
#define CS05_PRTDSI__OUT_SEL1       (* (reg8 *) CS05__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS05_PRTDSI__SYNC_OUT       (* (reg8 *) CS05__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS05__SIO_CFG)
    #define CS05_SIO_HYST_EN        (* (reg8 *) CS05__SIO_HYST_EN)
    #define CS05_SIO_REG_HIFREQ     (* (reg8 *) CS05__SIO_REG_HIFREQ)
    #define CS05_SIO_CFG            (* (reg8 *) CS05__SIO_CFG)
    #define CS05_SIO_DIFF           (* (reg8 *) CS05__SIO_DIFF)
#endif /* (CS05__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS05__INTSTAT)
    #define CS05_INTSTAT             (* (reg8 *) CS05__INTSTAT)
    #define CS05_SNAP                (* (reg8 *) CS05__SNAP)
    
	#define CS05_0_INTTYPE_REG 		(* (reg8 *) CS05__0__INTTYPE)
#endif /* (CS05__INTSTAT) */

#endif /* End Pins CS05_H */


/* [] END OF FILE */
