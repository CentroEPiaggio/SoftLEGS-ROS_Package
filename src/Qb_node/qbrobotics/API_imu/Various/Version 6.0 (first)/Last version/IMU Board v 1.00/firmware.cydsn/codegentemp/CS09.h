/*******************************************************************************
* File Name: CS09.h  
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

#if !defined(CY_PINS_CS09_H) /* Pins CS09_H */
#define CY_PINS_CS09_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS09_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS09_Write(uint8 value) ;
void    CS09_SetDriveMode(uint8 mode) ;
uint8   CS09_ReadDataReg(void) ;
uint8   CS09_Read(void) ;
void    CS09_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS09_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS09_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS09_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS09_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS09_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS09_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS09_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS09_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS09_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS09_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS09_MASK               CS09__MASK
#define CS09_SHIFT              CS09__SHIFT
#define CS09_WIDTH              1u

/* Interrupt constants */
#if defined(CS09__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS09_SetInterruptMode() function.
     *  @{
     */
        #define CS09_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS09_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS09_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS09_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS09_INTR_MASK      (0x01u)
#endif /* (CS09__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS09_PS                     (* (reg8 *) CS09__PS)
/* Data Register */
#define CS09_DR                     (* (reg8 *) CS09__DR)
/* Port Number */
#define CS09_PRT_NUM                (* (reg8 *) CS09__PRT) 
/* Connect to Analog Globals */                                                  
#define CS09_AG                     (* (reg8 *) CS09__AG)                       
/* Analog MUX bux enable */
#define CS09_AMUX                   (* (reg8 *) CS09__AMUX) 
/* Bidirectional Enable */                                                        
#define CS09_BIE                    (* (reg8 *) CS09__BIE)
/* Bit-mask for Aliased Register Access */
#define CS09_BIT_MASK               (* (reg8 *) CS09__BIT_MASK)
/* Bypass Enable */
#define CS09_BYP                    (* (reg8 *) CS09__BYP)
/* Port wide control signals */                                                   
#define CS09_CTL                    (* (reg8 *) CS09__CTL)
/* Drive Modes */
#define CS09_DM0                    (* (reg8 *) CS09__DM0) 
#define CS09_DM1                    (* (reg8 *) CS09__DM1)
#define CS09_DM2                    (* (reg8 *) CS09__DM2) 
/* Input Buffer Disable Override */
#define CS09_INP_DIS                (* (reg8 *) CS09__INP_DIS)
/* LCD Common or Segment Drive */
#define CS09_LCD_COM_SEG            (* (reg8 *) CS09__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS09_LCD_EN                 (* (reg8 *) CS09__LCD_EN)
/* Slew Rate Control */
#define CS09_SLW                    (* (reg8 *) CS09__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS09_PRTDSI__CAPS_SEL       (* (reg8 *) CS09__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS09_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS09__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS09_PRTDSI__OE_SEL0        (* (reg8 *) CS09__PRTDSI__OE_SEL0) 
#define CS09_PRTDSI__OE_SEL1        (* (reg8 *) CS09__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS09_PRTDSI__OUT_SEL0       (* (reg8 *) CS09__PRTDSI__OUT_SEL0) 
#define CS09_PRTDSI__OUT_SEL1       (* (reg8 *) CS09__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS09_PRTDSI__SYNC_OUT       (* (reg8 *) CS09__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS09__SIO_CFG)
    #define CS09_SIO_HYST_EN        (* (reg8 *) CS09__SIO_HYST_EN)
    #define CS09_SIO_REG_HIFREQ     (* (reg8 *) CS09__SIO_REG_HIFREQ)
    #define CS09_SIO_CFG            (* (reg8 *) CS09__SIO_CFG)
    #define CS09_SIO_DIFF           (* (reg8 *) CS09__SIO_DIFF)
#endif /* (CS09__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS09__INTSTAT)
    #define CS09_INTSTAT             (* (reg8 *) CS09__INTSTAT)
    #define CS09_SNAP                (* (reg8 *) CS09__SNAP)
    
	#define CS09_0_INTTYPE_REG 		(* (reg8 *) CS09__0__INTTYPE)
#endif /* (CS09__INTSTAT) */

#endif /* End Pins CS09_H */


/* [] END OF FILE */
