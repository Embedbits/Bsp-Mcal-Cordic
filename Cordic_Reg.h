/**
 * \author Mr.Nobody
 * \file Cordic_Reg.h
 * \ingroup Cordic
 * \brief Cordic module Reg component header file.
 *
 * This component contains the functionality for register operations. It is
 * private to the module (the register access is not shared with other
 * modules). The CORDIC peripheral has a single instance with three registers
 * (CORDIC_CSR, CORDIC_WDATA, CORDIC_RDATA, RM0481 / RM0517 CORDIC registers).
 * The functions are available on the device lines that have the peripheral
 * (STM32H543, H553, H562, H563, H573, H5E4, H5E5, H5F4, H5F5).
 *
 */

#ifndef CORDIC_REG_CORDIC_REG_H
#define CORDIC_REG_CORDIC_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Cordic_Types.h"                   /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief List of the CORDIC registers used by the module */
typedef enum
{
    CORDIC_REG_CSR   = 0u, /**< CORDIC control and status register */
    CORDIC_REG_WDATA     , /**< CORDIC argument register           */
    CORDIC_REG_RDATA     , /**< CORDIC result register             */
    CORDIC_REG_CNT         /**< Count of available registers       */
}   cordic_RegId_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* ---------------------------- Generic accessors --------------------------- */

void                    Cordic_Reg_Set_RegBit          ( cordic_RegId_t regId, uint32_t bitMask );
void                    Cordic_Reg_Reset_RegBit        ( cordic_RegId_t regId, uint32_t bitMask );
uint32_t                Cordic_Reg_Get_RegBit          ( cordic_RegId_t regId, uint32_t bitMask );

void                    Cordic_Reg_Set_RegVal          ( cordic_RegId_t regId, uint32_t regMask, uint32_t regValue );
uint32_t                Cordic_Reg_Get_RegVal          ( cordic_RegId_t regId, uint32_t regMask );

/* ------------------------------ Data registers ---------------------------- */

void                    Cordic_Reg_Set_Data            ( uint32_t argData );
uint32_t                Cordic_Reg_Get_Data            ( void );

uint32_t                Cordic_Reg_Get_WdataAddr       ( void );
uint32_t                Cordic_Reg_Get_RdataAddr       ( void );

/* ----------------------------- Register protocol -------------------------- */

cordic_FlagState_t      Cordic_Reg_Get_ResultReady     ( void );
cordic_RequestState_t   Cordic_Reg_Wait_ResultReady    ( void );

#ifdef __cplusplus
}
#endif

#endif /* CORDIC_REG_CORDIC_REG_H */
