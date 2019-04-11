/*******************************************************************************
* File Name: CS14.h  
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

#if !defined(CY_PINS_CS14_H) /* Pins CS14_H */
#define CY_PINS_CS14_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS14_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS14_Write(uint8 value) ;
void    CS14_SetDriveMode(uint8 mode) ;
uint8   CS14_ReadDataReg(void) ;
uint8   CS14_Read(void) ;
void    CS14_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS14_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS14_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS14_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS14_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS14_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS14_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS14_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS14_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS14_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS14_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS14_MASK               CS14__MASK
#define CS14_SHIFT              CS14__SHIFT
#define CS14_WIDTH              1u

/* Interrupt constants */
#if defined(CS14__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS14_SetInterruptMode() function.
     *  @{
     */
        #define CS14_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS14_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS14_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS14_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS14_INTR_MASK      (0x01u)
#endif /* (CS14__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS14_PS                     (* (reg8 *) CS14__PS)
/* Data Register */
#define CS14_DR                     (* (reg8 *) CS14__DR)
/* Port Number */
#define CS14_PRT_NUM                (* (reg8 *) CS14__PRT) 
/* Connect to Analog Globals */                                                  
#define CS14_AG                     (* (reg8 *) CS14__AG)                       
/* Analog MUX bux enable */
#define CS14_AMUX                   (* (reg8 *) CS14__AMUX) 
/* Bidirectional Enable */                                                        
#define CS14_BIE                    (* (reg8 *) CS14__BIE)
/* Bit-mask for Aliased Register Access */
#define CS14_BIT_MASK               (* (reg8 *) CS14__BIT_MASK)
/* Bypass Enable */
#define CS14_BYP                    (* (reg8 *) CS14__BYP)
/* Port wide control signals */                                                   
#define CS14_CTL                    (* (reg8 *) CS14__CTL)
/* Drive Modes */
#define CS14_DM0                    (* (reg8 *) CS14__DM0) 
#define CS14_DM1                    (* (reg8 *) CS14__DM1)
#define CS14_DM2                    (* (reg8 *) CS14__DM2) 
/* Input Buffer Disable Override */
#define CS14_INP_DIS                (* (reg8 *) CS14__INP_DIS)
/* LCD Common or Segment Drive */
#define CS14_LCD_COM_SEG            (* (reg8 *) CS14__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS14_LCD_EN                 (* (reg8 *) CS14__LCD_EN)
/* Slew Rate Control */
#define CS14_SLW                    (* (reg8 *) CS14__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS14_PRTDSI__CAPS_SEL       (* (reg8 *) CS14__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS14_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS14__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS14_PRTDSI__OE_SEL0        (* (reg8 *) CS14__PRTDSI__OE_SEL0) 
#define CS14_PRTDSI__OE_SEL1        (* (reg8 *) CS14__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS14_PRTDSI__OUT_SEL0       (* (reg8 *) CS14__PRTDSI__OUT_SEL0) 
#define CS14_PRTDSI__OUT_SEL1       (* (reg8 *) CS14__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS14_PRTDSI__SYNC_OUT       (* (reg8 *) CS14__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS14__SIO_CFG)
    #define CS14_SIO_HYST_EN        (* (reg8 *) CS14__SIO_HYST_EN)
    #define CS14_SIO_REG_HIFREQ     (* (reg8 *) CS14__SIO_REG_HIFREQ)
    #define CS14_SIO_CFG            (* (reg8 *) CS14__SIO_CFG)
    #define CS14_SIO_DIFF           (* (reg8 *) CS14__SIO_DIFF)
#endif /* (CS14__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS14__INTSTAT)
    #define CS14_INTSTAT             (* (reg8 *) CS14__INTSTAT)
    #define CS14_SNAP                (* (reg8 *) CS14__SNAP)
    
	#define CS14_0_INTTYPE_REG 		(* (reg8 *) CS14__0__INTTYPE)
#endif /* (CS14__INTSTAT) */

#endif /* End Pins CS14_H */


/* [] END OF FILE */
