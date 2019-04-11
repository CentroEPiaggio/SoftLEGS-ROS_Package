/*******************************************************************************
* File Name: CS11.h  
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

#if !defined(CY_PINS_CS11_H) /* Pins CS11_H */
#define CY_PINS_CS11_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS11_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    CS11_Write(uint8 value) ;
void    CS11_SetDriveMode(uint8 mode) ;
uint8   CS11_ReadDataReg(void) ;
uint8   CS11_Read(void) ;
void    CS11_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   CS11_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the CS11_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define CS11_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define CS11_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define CS11_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define CS11_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define CS11_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define CS11_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define CS11_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define CS11_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define CS11_MASK               CS11__MASK
#define CS11_SHIFT              CS11__SHIFT
#define CS11_WIDTH              1u

/* Interrupt constants */
#if defined(CS11__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in CS11_SetInterruptMode() function.
     *  @{
     */
        #define CS11_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define CS11_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define CS11_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define CS11_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define CS11_INTR_MASK      (0x01u)
#endif /* (CS11__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS11_PS                     (* (reg8 *) CS11__PS)
/* Data Register */
#define CS11_DR                     (* (reg8 *) CS11__DR)
/* Port Number */
#define CS11_PRT_NUM                (* (reg8 *) CS11__PRT) 
/* Connect to Analog Globals */                                                  
#define CS11_AG                     (* (reg8 *) CS11__AG)                       
/* Analog MUX bux enable */
#define CS11_AMUX                   (* (reg8 *) CS11__AMUX) 
/* Bidirectional Enable */                                                        
#define CS11_BIE                    (* (reg8 *) CS11__BIE)
/* Bit-mask for Aliased Register Access */
#define CS11_BIT_MASK               (* (reg8 *) CS11__BIT_MASK)
/* Bypass Enable */
#define CS11_BYP                    (* (reg8 *) CS11__BYP)
/* Port wide control signals */                                                   
#define CS11_CTL                    (* (reg8 *) CS11__CTL)
/* Drive Modes */
#define CS11_DM0                    (* (reg8 *) CS11__DM0) 
#define CS11_DM1                    (* (reg8 *) CS11__DM1)
#define CS11_DM2                    (* (reg8 *) CS11__DM2) 
/* Input Buffer Disable Override */
#define CS11_INP_DIS                (* (reg8 *) CS11__INP_DIS)
/* LCD Common or Segment Drive */
#define CS11_LCD_COM_SEG            (* (reg8 *) CS11__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS11_LCD_EN                 (* (reg8 *) CS11__LCD_EN)
/* Slew Rate Control */
#define CS11_SLW                    (* (reg8 *) CS11__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS11_PRTDSI__CAPS_SEL       (* (reg8 *) CS11__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS11_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS11__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS11_PRTDSI__OE_SEL0        (* (reg8 *) CS11__PRTDSI__OE_SEL0) 
#define CS11_PRTDSI__OE_SEL1        (* (reg8 *) CS11__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS11_PRTDSI__OUT_SEL0       (* (reg8 *) CS11__PRTDSI__OUT_SEL0) 
#define CS11_PRTDSI__OUT_SEL1       (* (reg8 *) CS11__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS11_PRTDSI__SYNC_OUT       (* (reg8 *) CS11__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(CS11__SIO_CFG)
    #define CS11_SIO_HYST_EN        (* (reg8 *) CS11__SIO_HYST_EN)
    #define CS11_SIO_REG_HIFREQ     (* (reg8 *) CS11__SIO_REG_HIFREQ)
    #define CS11_SIO_CFG            (* (reg8 *) CS11__SIO_CFG)
    #define CS11_SIO_DIFF           (* (reg8 *) CS11__SIO_DIFF)
#endif /* (CS11__SIO_CFG) */

/* Interrupt Registers */
#if defined(CS11__INTSTAT)
    #define CS11_INTSTAT             (* (reg8 *) CS11__INTSTAT)
    #define CS11_SNAP                (* (reg8 *) CS11__SNAP)
    
	#define CS11_0_INTTYPE_REG 		(* (reg8 *) CS11__0__INTTYPE)
#endif /* (CS11__INTSTAT) */

#endif /* End Pins CS11_H */


/* [] END OF FILE */
