/*******************************************************************************
* File Name: CS02.h  
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

#if !defined(CY_PINS_CS02_H) /* Pins CS02_H */
#define CY_PINS_CS02_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS02_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS02_Write(uint8 value) ;
void    CS02_SetDriveMode(uint8 mode) ;
uint8   CS02_ReadDataReg(void) ;
uint8   CS02_Read(void) ;
void    CS02_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS02_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS02_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS02_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS02_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS02_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS02_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS02_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS02_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS02_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS02_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS02_MASK               CS02__MASK
#define CS02_SHIFT              CS02__SHIFT
#define CS02_WIDTH              1u

/* Interrupt constants */
#if defined(CS02__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS02_SetInterruptMode() function.
     *  @{
     */
        #define CS02_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS02_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS02_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS02_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS02_INTR_MASK      (0x01u)
#endif /* (CS02__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS02_PS                     (* (reg8 *) CS02__PS)
/* Data Register */
#define CS02_DR                     (* (reg8 *) CS02__DR)
/* Port Number */
#define CS02_PRT_NUM                (* (reg8 *) CS02__PRT) 
/* Connect to Analog Globals */                                                  
#define CS02_AG                     (* (reg8 *) CS02__AG)                       
/* Analog MUX bux enable */
#define CS02_AMUX                   (* (reg8 *) CS02__AMUX) 
/* Bidirectional Enable */                                                        
#define CS02_BIE                    (* (reg8 *) CS02__BIE)
/* Bit-mask for Aliased Register Access */
#define CS02_BIT_MASK               (* (reg8 *) CS02__BIT_MASK)
/* Bypass Enable */
#define CS02_BYP                    (* (reg8 *) CS02__BYP)
/* Port wide control signals */                                                   
#define CS02_CTL                    (* (reg8 *) CS02__CTL)
/* Drive Modes */
#define CS02_DM0                    (* (reg8 *) CS02__DM0) 
#define CS02_DM1                    (* (reg8 *) CS02__DM1)
#define CS02_DM2                    (* (reg8 *) CS02__DM2) 
/* Input Buffer Disable Override */
#define CS02_INP_DIS                (* (reg8 *) CS02__INP_DIS)
/* LCD Common or Segment Drive */
#define CS02_LCD_COM_SEG            (* (reg8 *) CS02__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS02_LCD_EN                 (* (reg8 *) CS02__LCD_EN)
/* Slew Rate Control */
#define CS02_SLW                    (* (reg8 *) CS02__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS02_PRTDSI__CAPS_SEL       (* (reg8 *) CS02__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS02_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS02__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS02_PRTDSI__OE_SEL0        (* (reg8 *) CS02__PRTDSI__OE_SEL0) 
#define CS02_PRTDSI__OE_SEL1        (* (reg8 *) CS02__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS02_PRTDSI__OUT_SEL0       (* (reg8 *) CS02__PRTDSI__OUT_SEL0) 
#define CS02_PRTDSI__OUT_SEL1       (* (reg8 *) CS02__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS02_PRTDSI__SYNC_OUT       (* (reg8 *) CS02__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS02__SIO_CFG)
    #define CS02_SIO_HYST_EN        (* (reg8 *) CS02__SIO_HYST_EN)
    #define CS02_SIO_REG_HIFREQ     (* (reg8 *) CS02__SIO_REG_HIFREQ)
    #define CS02_SIO_CFG            (* (reg8 *) CS02__SIO_CFG)
    #define CS02_SIO_DIFF           (* (reg8 *) CS02__SIO_DIFF)
#endif /* (CS02__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS02__INTSTAT)
    #define CS02_INTSTAT             (* (reg8 *) CS02__INTSTAT)
    #define CS02_SNAP                (* (reg8 *) CS02__SNAP)
    
	#define CS02_0_INTTYPE_REG 		(* (reg8 *) CS02__0__INTTYPE)
#endif /* (CS02__INTSTAT) */

#endif /* End Pins CS02_H */


/* [] END OF FILE */
