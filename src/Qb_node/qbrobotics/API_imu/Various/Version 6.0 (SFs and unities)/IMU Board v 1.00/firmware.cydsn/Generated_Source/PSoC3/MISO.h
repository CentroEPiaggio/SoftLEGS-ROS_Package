/*******************************************************************************
* File Name: MISO.h  
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

#if !defined(CY_PINS_MISO_H) /* Pins MISO_H */
#define CY_PINS_MISO_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "MISO_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    MISO_Write(uint8 value) ;
void    MISO_SetDriveMode(uint8 mode) ;
uint8   MISO_ReadDataReg(void) ;
uint8   MISO_Read(void) ;
void    MISO_SetInterruptMode(uint16 position, uint16 mode) ;
uint8   MISO_ClearInterrupt(void) ;
/** @} general */

/***************************************
*           API Constants        
***************************************/

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the MISO_SetDriveMode() function.
     *  @{
     */
        /* Drive Modes */
        #define MISO_DM_ALG_HIZ         PIN_DM_ALG_HIZ   /**< \brief High Impedance Analog   */
        #define MISO_DM_DIG_HIZ         PIN_DM_DIG_HIZ   /**< \brief High Impedance Digital  */
        #define MISO_DM_RES_UP          PIN_DM_RES_UP    /**< \brief Resistive Pull Up       */
        #define MISO_DM_RES_DWN         PIN_DM_RES_DWN   /**< \brief Resistive Pull Down     */
        #define MISO_DM_OD_LO           PIN_DM_OD_LO     /**< \brief Open Drain, Drives Low  */
        #define MISO_DM_OD_HI           PIN_DM_OD_HI     /**< \brief Open Drain, Drives High */
        #define MISO_DM_STRONG          PIN_DM_STRONG    /**< \brief Strong Drive            */
        #define MISO_DM_RES_UPDWN       PIN_DM_RES_UPDWN /**< \brief Resistive Pull Up/Down  */
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define MISO_MASK               MISO__MASK
#define MISO_SHIFT              MISO__SHIFT
#define MISO_WIDTH              1u

/* Interrupt constants */
#if defined(MISO__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in MISO_SetInterruptMode() function.
     *  @{
     */
        #define MISO_INTR_NONE      (uint16)(0x0000u)   /**< \brief Disabled             */
        #define MISO_INTR_RISING    (uint16)(0x0001u)   /**< \brief Rising edge trigger  */
        #define MISO_INTR_FALLING   (uint16)(0x0002u)   /**< \brief Falling edge trigger */
        #define MISO_INTR_BOTH      (uint16)(0x0003u)   /**< \brief Both edge trigger    */
        /** @} intrMode */
/** @} group_constants */

    #define MISO_INTR_MASK      (0x01u)
#endif /* (MISO__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MISO_PS                     (* (reg8 *) MISO__PS)
/* Data Register */
#define MISO_DR                     (* (reg8 *) MISO__DR)
/* Port Number */
#define MISO_PRT_NUM                (* (reg8 *) MISO__PRT) 
/* Connect to Analog Globals */                                                  
#define MISO_AG                     (* (reg8 *) MISO__AG)                       
/* Analog MUX bux enable */
#define MISO_AMUX                   (* (reg8 *) MISO__AMUX) 
/* Bidirectional Enable */                                                        
#define MISO_BIE                    (* (reg8 *) MISO__BIE)
/* Bit-mask for Aliased Register Access */
#define MISO_BIT_MASK               (* (reg8 *) MISO__BIT_MASK)
/* Bypass Enable */
#define MISO_BYP                    (* (reg8 *) MISO__BYP)
/* Port wide control signals */                                                   
#define MISO_CTL                    (* (reg8 *) MISO__CTL)
/* Drive Modes */
#define MISO_DM0                    (* (reg8 *) MISO__DM0) 
#define MISO_DM1                    (* (reg8 *) MISO__DM1)
#define MISO_DM2                    (* (reg8 *) MISO__DM2) 
/* Input Buffer Disable Override */
#define MISO_INP_DIS                (* (reg8 *) MISO__INP_DIS)
/* LCD Common or Segment Drive */
#define MISO_LCD_COM_SEG            (* (reg8 *) MISO__LCD_COM_SEG)
/* Enable Segment LCD */
#define MISO_LCD_EN                 (* (reg8 *) MISO__LCD_EN)
/* Slew Rate Control */
#define MISO_SLW                    (* (reg8 *) MISO__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define MISO_PRTDSI__CAPS_SEL       (* (reg8 *) MISO__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define MISO_PRTDSI__DBL_SYNC_IN    (* (reg8 *) MISO__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define MISO_PRTDSI__OE_SEL0        (* (reg8 *) MISO__PRTDSI__OE_SEL0) 
#define MISO_PRTDSI__OE_SEL1        (* (reg8 *) MISO__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define MISO_PRTDSI__OUT_SEL0       (* (reg8 *) MISO__PRTDSI__OUT_SEL0) 
#define MISO_PRTDSI__OUT_SEL1       (* (reg8 *) MISO__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define MISO_PRTDSI__SYNC_OUT       (* (reg8 *) MISO__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(MISO__SIO_CFG)
    #define MISO_SIO_HYST_EN        (* (reg8 *) MISO__SIO_HYST_EN)
    #define MISO_SIO_REG_HIFREQ     (* (reg8 *) MISO__SIO_REG_HIFREQ)
    #define MISO_SIO_CFG            (* (reg8 *) MISO__SIO_CFG)
    #define MISO_SIO_DIFF           (* (reg8 *) MISO__SIO_DIFF)
#endif /* (MISO__SIO_CFG) */

/* Interrupt Registers */
#if defined(MISO__INTSTAT)
    #define MISO_INTSTAT             (* (reg8 *) MISO__INTSTAT)
    #define MISO_SNAP                (* (reg8 *) MISO__SNAP)
    
	#define MISO_0_INTTYPE_REG 		(* (reg8 *) MISO__0__INTTYPE)
#endif /* (MISO__INTSTAT) */

#endif /* End Pins MISO_H */


/* [] END OF FILE */
