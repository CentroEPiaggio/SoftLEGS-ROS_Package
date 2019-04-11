/*******************************************************************************
* File Name: CS01.h  
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

#if !defined(CY_PINS_CS01_H) /* Pins CS01_H */
#define CY_PINS_CS01_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS01_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS01_Write(uint8 value) ;
void    CS01_SetDriveMode(uint8 mode) ;
uint8   CS01_ReadDataReg(void) ;
uint8   CS01_Read(void) ;
void    CS01_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS01_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS01_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS01_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS01_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS01_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS01_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS01_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS01_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS01_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS01_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS01_MASK               CS01__MASK
#define CS01_SHIFT              CS01__SHIFT
#define CS01_WIDTH              1u

/* Interrupt constants */
#if defined(CS01__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS01_SetInterruptMode() function.
     *  @{
     */
        #define CS01_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS01_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS01_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS01_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS01_INTR_MASK      (0x01u)
#endif /* (CS01__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS01_PS                     (* (reg8 *) CS01__PS)
/* Data Register */
#define CS01_DR                     (* (reg8 *) CS01__DR)
/* Port Number */
#define CS01_PRT_NUM                (* (reg8 *) CS01__PRT) 
/* Connect to Analog Globals */                                                  
#define CS01_AG                     (* (reg8 *) CS01__AG)                       
/* Analog MUX bux enable */
#define CS01_AMUX                   (* (reg8 *) CS01__AMUX) 
/* Bidirectional Enable */                                                        
#define CS01_BIE                    (* (reg8 *) CS01__BIE)
/* Bit-mask for Aliased Register Access */
#define CS01_BIT_MASK               (* (reg8 *) CS01__BIT_MASK)
/* Bypass Enable */
#define CS01_BYP                    (* (reg8 *) CS01__BYP)
/* Port wide control signals */                                                   
#define CS01_CTL                    (* (reg8 *) CS01__CTL)
/* Drive Modes */
#define CS01_DM0                    (* (reg8 *) CS01__DM0) 
#define CS01_DM1                    (* (reg8 *) CS01__DM1)
#define CS01_DM2                    (* (reg8 *) CS01__DM2) 
/* Input Buffer Disable Override */
#define CS01_INP_DIS                (* (reg8 *) CS01__INP_DIS)
/* LCD Common or Segment Drive */
#define CS01_LCD_COM_SEG            (* (reg8 *) CS01__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS01_LCD_EN                 (* (reg8 *) CS01__LCD_EN)
/* Slew Rate Control */
#define CS01_SLW                    (* (reg8 *) CS01__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS01_PRTDSI__CAPS_SEL       (* (reg8 *) CS01__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS01_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS01__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS01_PRTDSI__OE_SEL0        (* (reg8 *) CS01__PRTDSI__OE_SEL0) 
#define CS01_PRTDSI__OE_SEL1        (* (reg8 *) CS01__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS01_PRTDSI__OUT_SEL0       (* (reg8 *) CS01__PRTDSI__OUT_SEL0) 
#define CS01_PRTDSI__OUT_SEL1       (* (reg8 *) CS01__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS01_PRTDSI__SYNC_OUT       (* (reg8 *) CS01__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS01__SIO_CFG)
    #define CS01_SIO_HYST_EN        (* (reg8 *) CS01__SIO_HYST_EN)
    #define CS01_SIO_REG_HIFREQ     (* (reg8 *) CS01__SIO_REG_HIFREQ)
    #define CS01_SIO_CFG            (* (reg8 *) CS01__SIO_CFG)
    #define CS01_SIO_DIFF           (* (reg8 *) CS01__SIO_DIFF)
#endif /* (CS01__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS01__INTSTAT)
    #define CS01_INTSTAT             (* (reg8 *) CS01__INTSTAT)
    #define CS01_SNAP                (* (reg8 *) CS01__SNAP)
    
	#define CS01_0_INTTYPE_REG 		(* (reg8 *) CS01__0__INTTYPE)
#endif /* (CS01__INTSTAT) */

#endif /* End Pins CS01_H */


/* [] END OF FILE */
