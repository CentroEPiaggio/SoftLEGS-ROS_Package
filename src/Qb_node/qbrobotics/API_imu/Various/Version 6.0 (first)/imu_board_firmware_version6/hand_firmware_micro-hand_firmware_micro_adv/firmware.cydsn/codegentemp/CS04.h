/*******************************************************************************
* File Name: CS04.h  
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

#if !defined(CY_PINS_CS04_H) /* Pins CS04_H */
#define CY_PINS_CS04_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS04_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS04_Write(uint8 value) ;
void    CS04_SetDriveMode(uint8 mode) ;
uint8   CS04_ReadDataReg(void) ;
uint8   CS04_Read(void) ;
void    CS04_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS04_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS04_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS04_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS04_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS04_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS04_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS04_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS04_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS04_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS04_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS04_MASK               CS04__MASK
#define CS04_SHIFT              CS04__SHIFT
#define CS04_WIDTH              1u

/* Interrupt constants */
#if defined(CS04__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS04_SetInterruptMode() function.
     *  @{
     */
        #define CS04_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS04_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS04_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS04_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS04_INTR_MASK      (0x01u)
#endif /* (CS04__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS04_PS                     (* (reg8 *) CS04__PS)
/* Data Register */
#define CS04_DR                     (* (reg8 *) CS04__DR)
/* Port Number */
#define CS04_PRT_NUM                (* (reg8 *) CS04__PRT) 
/* Connect to Analog Globals */                                                  
#define CS04_AG                     (* (reg8 *) CS04__AG)                       
/* Analog MUX bux enable */
#define CS04_AMUX                   (* (reg8 *) CS04__AMUX) 
/* Bidirectional Enable */                                                        
#define CS04_BIE                    (* (reg8 *) CS04__BIE)
/* Bit-mask for Aliased Register Access */
#define CS04_BIT_MASK               (* (reg8 *) CS04__BIT_MASK)
/* Bypass Enable */
#define CS04_BYP                    (* (reg8 *) CS04__BYP)
/* Port wide control signals */                                                   
#define CS04_CTL                    (* (reg8 *) CS04__CTL)
/* Drive Modes */
#define CS04_DM0                    (* (reg8 *) CS04__DM0) 
#define CS04_DM1                    (* (reg8 *) CS04__DM1)
#define CS04_DM2                    (* (reg8 *) CS04__DM2) 
/* Input Buffer Disable Override */
#define CS04_INP_DIS                (* (reg8 *) CS04__INP_DIS)
/* LCD Common or Segment Drive */
#define CS04_LCD_COM_SEG            (* (reg8 *) CS04__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS04_LCD_EN                 (* (reg8 *) CS04__LCD_EN)
/* Slew Rate Control */
#define CS04_SLW                    (* (reg8 *) CS04__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS04_PRTDSI__CAPS_SEL       (* (reg8 *) CS04__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS04_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS04__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS04_PRTDSI__OE_SEL0        (* (reg8 *) CS04__PRTDSI__OE_SEL0) 
#define CS04_PRTDSI__OE_SEL1        (* (reg8 *) CS04__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS04_PRTDSI__OUT_SEL0       (* (reg8 *) CS04__PRTDSI__OUT_SEL0) 
#define CS04_PRTDSI__OUT_SEL1       (* (reg8 *) CS04__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS04_PRTDSI__SYNC_OUT       (* (reg8 *) CS04__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS04__SIO_CFG)
    #define CS04_SIO_HYST_EN        (* (reg8 *) CS04__SIO_HYST_EN)
    #define CS04_SIO_REG_HIFREQ     (* (reg8 *) CS04__SIO_REG_HIFREQ)
    #define CS04_SIO_CFG            (* (reg8 *) CS04__SIO_CFG)
    #define CS04_SIO_DIFF           (* (reg8 *) CS04__SIO_DIFF)
#endif /* (CS04__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS04__INTSTAT)
    #define CS04_INTSTAT             (* (reg8 *) CS04__INTSTAT)
    #define CS04_SNAP                (* (reg8 *) CS04__SNAP)
    
	#define CS04_0_INTTYPE_REG 		(* (reg8 *) CS04__0__INTTYPE)
#endif /* (CS04__INTSTAT) */

#endif /* End Pins CS04_H */


/* [] END OF FILE */
