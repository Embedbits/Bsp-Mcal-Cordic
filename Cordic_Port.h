/**
 * \author Mr.Nobody
 * \file Cordic_Port.h
 * \ingroup Cordic
 * \brief Cordic module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef CORDIC_CORDIC_PORT_H
#define CORDIC_CORDIC_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Cordic_Types.h"                   /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

cordic_ModuleVersion_t      Cordic_Get_ModuleVersion        ( void );

cordic_RequestState_t       Cordic_Init                     ( const cordic_Config_t * const cordicConfig );
cordic_RequestState_t       Cordic_Deinit                   ( void );
void                        Cordic_Task                     ( void );

cordic_RequestState_t       Cordic_Get_DefaultConfig        ( cordic_Config_t * const cordicConfig );

/* ------------------------------- Calculation ------------------------------- */

cordic_RequestState_t       Cordic_Set_Calculate            ( const cordic_Arguments_t * const arguments,
                                                              cordic_Results_t * const results );
cordic_RequestState_t       Cordic_Set_CalculateArray       ( const cordic_ArrayRequest_t * const request );
cordic_RequestState_t       Cordic_Get_State                ( cordic_State_t * const state );

/* -------------------------------- Settings --------------------------------- */

cordic_RequestState_t       Cordic_Set_Function             ( cordic_Function_t function );
cordic_RequestState_t       Cordic_Get_Function             ( cordic_Function_t * const function );

cordic_RequestState_t       Cordic_Set_Precision            ( cordic_Precision_t precision );
cordic_RequestState_t       Cordic_Get_Precision            ( cordic_Precision_t * const precision );

cordic_RequestState_t       Cordic_Set_Scale                ( cordic_Scale_t scale );
cordic_RequestState_t       Cordic_Get_Scale                ( cordic_Scale_t * const scale );

cordic_RequestState_t       Cordic_Set_Format               ( cordic_Format_t format );
cordic_RequestState_t       Cordic_Get_Format               ( cordic_Format_t * const format );

/* ------------------------------- Conversion -------------------------------- */

cordic_RequestState_t       Cordic_Get_FromFloat            ( float value, cordic_Q31_t * const qValue );
cordic_RequestState_t       Cordic_Get_ToFloat              ( cordic_Q31_t qValue, float * const value );
cordic_RequestState_t       Cordic_Get_FromFloatQ15         ( float value, cordic_Q15_t * const qValue );
cordic_RequestState_t       Cordic_Get_ToFloatQ15           ( cordic_Q15_t qValue, float * const value );

/* ------------------------------- Interrupts -------------------------------- */

cordic_RequestState_t       Cordic_Set_IrqPriority          ( cordic_IrqPrio_t irqPrio );
cordic_RequestState_t       Cordic_Get_IrqPriority          ( cordic_IrqPrio_t * const irqPrio );

#ifdef __cplusplus
}
#endif

#endif /* CORDIC_CORDIC_PORT_H */
