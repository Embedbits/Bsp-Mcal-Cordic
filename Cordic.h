/**
 * \author Mr.Nobody
 * \file Cordic.h
 * \ingroup Cordic
 * \brief Cordic module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef CORDIC_CORDIC_H
#define CORDIC_CORDIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Cordic_Types.h"                   /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Type used to store the number of values (arguments or results) of a calculation */
typedef uint32_t cordic_ValueCnt_t;


/** \brief Type used to store the number of 32-bit data words of an array transfer */
typedef uint32_t cordic_WordCnt_t;


/** \brief Closed range of a primary or secondary argument in the q1.31 format */
typedef struct
{
    cordic_Q31_t Min; /**< Smallest accepted argument */
    cordic_Q31_t Max; /**< Largest accepted argument  */
}   cordic_ArgRange_t;


/**
 * \brief Properties of one function that the module needs to drive the hardware, indexed by
 *        \ref cordic_Function_t (the item value is the CORDIC_CSR.FUNC encoding)
 */
typedef struct
{
    cordic_ValueCnt_t           ArgCnt;      /**< Number of the arguments (CORDIC_CSR.NARGS + 1)               */
    cordic_ValueCnt_t           ResCnt;      /**< Number of the results (CORDIC_CSR.NRES + 1)                  */
    cordic_Scale_t              ScaleMin;    /**< Smallest valid scaling exponent                              */
    cordic_Scale_t              ScaleMax;    /**< Largest valid scaling exponent                               */
    cordic_ArgRange_t           Arg1Range;   /**< Range of the primary argument for every valid scale          */
    cordic_ArgRange_t           Arg2Range;   /**< Range of the secondary argument, if the function has it      */
    const cordic_ArgRange_t *   Arg1ByScale; /**< Ranges of the primary argument per scale (index = scale - ScaleMin),
                                                  CORDIC_NULL_PTR if \ref Arg1Range holds for every scale      */
}   cordic_FunctionProps_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

#ifdef __cplusplus
}
#endif

#endif /* CORDIC_CORDIC_H */
