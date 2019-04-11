/*******************************************************************************
* File Name: CS13.h  
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

#if !defined(CY_PINS_CS13_H) /* Pins CS13_H */
#define CY_PINS_CS13_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS13_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS13_Write(uint8 value) ;
void    CS13_SetDriveMode(uint8 mode) ;
uint8   CS13_ReadDataReg(void) ;
uint8   CS13_Read(void) ;
void    CS13_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS13_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS13_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS13_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS13_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS13_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS13_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS13_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS13_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS13_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS13_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS13_MASK               CS13__MASK
#define CS13_SHIFT              CS13__SHIFT
#define CS13_WIDTH              1u

/* Interrupt constants */
#if defined(CS13__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS13_SetInterruptMode() function.
     *  @{
     */
        #define CS13_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS13_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS13_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS13_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS13_INTR_MASK      (0x01u)
#endif /* (CS13__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS13_PS                     (* (reg8 *) CS13__PS)
/* Data Register */
#define CS13_DR                     (* (reg8 *) CS13__DR)
/* Port Number */
#define CS13_PRT_NUM                (* (reg8 *) CS13__PRT) 
/* Connect to Analog Globals */                                                  
#define CS13_AG                     (* (reg8 *) CS13__AG)                       
/* Analog MUX bux enable */
#define CS13_AMUX                   (* (reg8 *) CS13__AMUX) 
/* Bidirectional Enable */                                                        
#define CS13_BIE                    (* (reg8 *) CS13__BIE)
/* Bit-mask for Aliased Register Access */
#define CS13_BIT_MASK               (* (reg8 *) CS13__BIT_MASK)
/* Bypass Enable */
#define CS13_BYP                    (* (reg8 *) CS13__BYP)
/* Port wide control signals */                                                   
#define CS13_CTL                    (* (reg8 *) CS13__CTL)
/* Drive Modes */
#define CS13_DM0                    (* (reg8 *) CS13__DM0) 
#define CS13_DM1                    (* (reg8 *) CS13__DM1)
#define CS13_DM2                    (* (reg8 *) CS13__DM2) 
/* Input Buffer Disable Override */
#define CS13_INP_DIS                (* (reg8 *) CS13__INP_DIS)
/* LCD Common or Segment Drive */
#define CS13_LCD_COM_SEG            (* (reg8 *) CS13__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS13_LCD_EN                 (* (reg8 *) CS13__LCD_EN)
/* Slew Rate Control */
#define CS13_SLW                    (* (reg8 *) CS13__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS13_PRTDSI__CAPS_SEL       (* (reg8 *) CS13__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS13_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS13__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS13_PRTDSI__OE_SEL0        (* (reg8 *) CS13__PRTDSI__OE_SEL0) 
#define CS13_PRTDSI__OE_SEL1        (* (reg8 *) CS13__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS13_PRTDSI__OUT_SEL0       (* (reg8 *) CS13__PRTDSI__OUT_SEL0) 
#define CS13_PRTDSI__OUT_SEL1       (* (reg8 *) CS13__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS13_PRTDSI__SYNC_OUT       (* (reg8 *) CS13__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS13__SIO_CFG)
    #define CS13_SIO_HYST_EN        (* (reg8 *) CS13__SIO_HYST_EN)
    #define CS13_SIO_REG_HIFREQ     (* (reg8 *) CS13__SIO_REG_HIFREQ)
    #define CS13_SIO_CFG            (* (reg8 *) CS13__SIO_CFG)
    #define CS13_SIO_DIFF           (* (reg8 *) CS13__SIO_DIFF)
#endif /* (CS13__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS13__INTSTAT)
    #define CS13_INTSTAT             (* (reg8 *) CS13__INTSTAT)
    #define CS13_SNAP                (* (reg8 *) CS13__SNAP)
    
	#define CS13_0_INTTYPE_REG 		(* (reg8 *) CS13__0__INTTYPE)
#endif /* (CS13__INTSTAT) */

#endif /* End Pins CS13_H */


/* [] END OF FILE */
