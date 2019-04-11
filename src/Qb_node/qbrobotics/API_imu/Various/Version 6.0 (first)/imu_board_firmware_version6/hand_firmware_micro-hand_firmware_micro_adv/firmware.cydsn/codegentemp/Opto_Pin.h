/*******************************************************************************
* File Name: Opto_Pin.h  
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

#if !defined(CY_PINS_Opto_Pin_H) /* Pins Opto_Pin_H */
#define CY_PINS_Opto_Pin_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Opto_Pin_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    Opto_Pin_Write(uint8 value) ;
void    Opto_Pin_SetDriveMode(uint8 mode) ;
uint8   Opto_Pin_ReadDataReg(void) ;
uint8   Opto_Pin_Read(void) ;
void    Opto_Pin_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   Opto_Pin_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the Opto_Pin_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define Opto_Pin_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define Opto_Pin_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define Opto_Pin_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define Opto_Pin_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define Opto_Pin_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define Opto_Pin_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define Opto_Pin_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define Opto_Pin_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define Opto_Pin_MASK               Opto_Pin__MASK
#define Opto_Pin_SHIFT              Opto_Pin__SHIFT
#define Opto_Pin_WIDTH              1u

/* Interrupt constants */
#if defined(Opto_Pin__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Opto_Pin_SetInterruptMode() function.
     *  @{
     */
        #define Opto_Pin_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define Opto_Pin_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define Opto_Pin_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define Opto_Pin_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define Opto_Pin_INTR_MASK      (0x01u)
#endif /* (Opto_Pin__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Opto_Pin_PS                     (* (reg8 *) Opto_Pin__PS)
/* Data Register */
#define Opto_Pin_DR                     (* (reg8 *) Opto_Pin__DR)
/* Port Number */
#define Opto_Pin_PRT_NUM                (* (reg8 *) Opto_Pin__PRT) 
/* Connect to Analog Globals */                                                  
#define Opto_Pin_AG                     (* (reg8 *) Opto_Pin__AG)                       
/* Analog MUX bux enable */
#define Opto_Pin_AMUX                   (* (reg8 *) Opto_Pin__AMUX) 
/* Bidirectional Enable */                                                        
#define Opto_Pin_BIE                    (* (reg8 *) Opto_Pin__BIE)
/* Bit-mask for Aliased Register Access */
#define Opto_Pin_BIT_MASK               (* (reg8 *) Opto_Pin__BIT_MASK)
/* Bypass Enable */
#define Opto_Pin_BYP                    (* (reg8 *) Opto_Pin__BYP)
/* Port wide control signals */                                                   
#define Opto_Pin_CTL                    (* (reg8 *) Opto_Pin__CTL)
/* Drive Modes */
#define Opto_Pin_DM0                    (* (reg8 *) Opto_Pin__DM0) 
#define Opto_Pin_DM1                    (* (reg8 *) Opto_Pin__DM1)
#define Opto_Pin_DM2                    (* (reg8 *) Opto_Pin__DM2) 
/* Input Buffer Disable Override */
#define Opto_Pin_INP_DIS                (* (reg8 *) Opto_Pin__INP_DIS)
/* LCD Common or Segment Drive */
#define Opto_Pin_LCD_COM_SEG            (* (reg8 *) Opto_Pin__LCD_COM_SEG)
/* Enable Segment LCD */
#define Opto_Pin_LCD_EN                 (* (reg8 *) Opto_Pin__LCD_EN)
/* Slew Rate Control */
#define Opto_Pin_SLW                    (* (reg8 *) Opto_Pin__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Opto_Pin_PRTDSI__CAPS_SEL       (* (reg8 *) Opto_Pin__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Opto_Pin_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Opto_Pin__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Opto_Pin_PRTDSI__OE_SEL0        (* (reg8 *) Opto_Pin__PRTDSI__OE_SEL0) 
#define Opto_Pin_PRTDSI__OE_SEL1        (* (reg8 *) Opto_Pin__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Opto_Pin_PRTDSI__OUT_SEL0       (* (reg8 *) Opto_Pin__PRTDSI__OUT_SEL0) 
#define Opto_Pin_PRTDSI__OUT_SEL1       (* (reg8 *) Opto_Pin__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Opto_Pin_PRTDSI__SYNC_OUT       (* (reg8 *) Opto_Pin__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(Opto_Pin__SIO_CFG)
    #define Opto_Pin_SIO_HYST_EN        (* (reg8 *) Opto_Pin__SIO_HYST_EN)
    #define Opto_Pin_SIO_REG_HIFREQ     (* (reg8 *) Opto_Pin__SIO_REG_HIFREQ)
    #define Opto_Pin_SIO_CFG            (* (reg8 *) Opto_Pin__SIO_CFG)
    #define Opto_Pin_SIO_DIFF           (* (reg8 *) Opto_Pin__SIO_DIFF)
#endif /* (Opto_Pin__SIO_CFG) */

/* Interrupt Registers */
#if defined(Opto_Pin__INTSTAT)
    #define Opto_Pin_INTSTAT             (* (reg8 *) Opto_Pin__INTSTAT)
    #define Opto_Pin_SNAP                (* (reg8 *) Opto_Pin__SNAP)
    
	#define Opto_Pin_0_INTTYPE_REG 		(* (reg8 *) Opto_Pin__0__INTTYPE)
#endif /* (Opto_Pin__INTSTAT) */

#endif /* End Pins Opto_Pin_H */


/* [] END OF FILE */
