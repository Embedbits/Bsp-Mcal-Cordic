/**
 * \author Mr.Nobody
 * \file Cordic_Types.h
 * \ingroup Cordic
 * \brief Cordic module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through the Port file. The interface is the same
 * for all families; the lists below contain what the CORDIC coprocessor of the
 * device offers (STM32H5: STM32H543, H553, H562, H563, H573 and the H5E / H5F
 * lines have it, H503, H523 and H533 do not).
 *
 */

/**
 * \defgroup Cordic Cordic
 * \brief Cordic module
 */

#ifndef CORDIC_CORDIC_TYPES_H
#define CORDIC_CORDIC_TYPES_H
/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Nvic_Port.h"                      /* NVIC Port module definitions   */
#include "Gpdma_Port.h"                     /* GPDMA Port module definitions  */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define CORDIC_NULL_PTR                     ( ( void* ) 0u )

/** Largest q1.31 value, 1.0 - 2^-31 (the modulus 1.0 of \ref CORDIC_FUNC_COSINE and \ref CORDIC_FUNC_SINE) */
#define CORDIC_Q31_ONE                      ( 0x7FFFFFFF )

/** Smallest q1.31 value, -1.0 */
#define CORDIC_Q31_MINUS_ONE                ( -0x7FFFFFFF - 1 )

/** Largest q1.15 value, 1.0 - 2^-15 */
#define CORDIC_Q15_ONE                      ( 0x7FFF )

/** Smallest q1.15 value, -1.0 */
#define CORDIC_Q15_MINUS_ONE                ( -0x7FFF - 1 )

/** Count of bits of one q1.15 value, the shift between the two values packed in one 32-bit word */
#define CORDIC_Q15_BITS                     ( 16u )

/** Mask of the lower (first) q1.15 value of a 32-bit word with two packed values */
#define CORDIC_Q15_MASK                     ( 0xFFFFu )

/* ========================== EXPORTED MACROS =============================== */

/** Packs two q1.15 values into one 32-bit data word of the 16-bit format (FIRST in the lower half-word) */
#define CORDIC_Q15_PACK( FIRST, SECOND ) \
            ( (cordic_Q31_t)( ( (uint32_t)(FIRST) & CORDIC_Q15_MASK ) | \
                              ( (uint32_t)(SECOND) << CORDIC_Q15_BITS )   ) )

/** Extracts the first (lower half-word) q1.15 value of a 32-bit data word of the 16-bit format */
#define CORDIC_Q15_FIRST( WORD ) \
            ( (cordic_Q15_t)( (uint32_t)(WORD) & CORDIC_Q15_MASK ) )

/** Extracts the second (upper half-word) q1.15 value of a 32-bit data word of the 16-bit format */
#define CORDIC_Q15_SECOND( WORD ) \
            ( (cordic_Q15_t)( (uint32_t)(WORD) >> CORDIC_Q15_BITS ) )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t cordic_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t cordic_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t cordic_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    cordic_MajorVersion_t Major; /**< Major version */
    cordic_MinorVersion_t Minor; /**< Minor version */
    cordic_PatchVersion_t Patch; /**< Patch version */
}   cordic_ModuleVersion_t;


/** \brief Enumeration used to signal request processing state */
typedef enum
{
    CORDIC_REQUEST_ERROR = 0u, /**< Processing request failed  */
    CORDIC_REQUEST_OK          /**< Processing request succeed */
}   cordic_RequestState_t;


/** \brief Function status enumeration */
typedef enum
{
    CORDIC_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    CORDIC_FUNCTION_ACTIVE         /**< Function status is active   */
}   cordic_FunctionState_t;


/** \brief Flag states enumeration */
typedef enum
{
    CORDIC_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    CORDIC_FLAG_ACTIVE         /**< Active flag state   */
}   cordic_FlagState_t;


/**
 * \brief Fixed-point type with 1 sign bit and 31 fractional bits (q1.31)
 *
 * Range -1.0 (0x80000000) to 1.0 - 2^-31 (0x7FFFFFFF). Angles are normalized to [-1, 1) as a fraction of pi.
 */
typedef int32_t cordic_Q31_t;


/**
 * \brief Fixed-point type with 1 sign bit and 15 fractional bits (q1.15)
 *
 * Range -1.0 (0x8000) to 1.0 - 2^-15 (0x7FFF). Two values are packed in one 32-bit data word.
 */
typedef int16_t cordic_Q15_t;


/** \brief Type used to store the number of calculations of an array request */
typedef uint32_t cordic_Count_t;


/** \brief Interrupt priority type definition (forwarded to Nvic) */
typedef nvic_IrqPrio_t cordic_IrqPrio_t;


/** \brief DMA controller identification type (forwarded to Gpdma) */
typedef gpdma_PeriphId_t cordic_DmaPeriphId_t;


/** \brief DMA channel identification type (forwarded to Gpdma) */
typedef gpdma_ChannelId_t cordic_DmaChannelId_t;


/** \brief DMA channel priority type (forwarded to Gpdma) */
typedef gpdma_Priority_t cordic_DmaPriority_t;


/**
 * \brief Function evaluated by the coprocessor (CORDIC_CSR.FUNC)
 *
 * Argument 1 / 2 and result 1 / 2 of every function, with the ranges of the reference manual:
 * - Cosine: angle / pi and modulus m in [0, 1]; m * cos and m * sin.
 * - Sine: angle / pi and modulus m in [0, 1]; m * sin and m * cos.
 * - Phase: x and y; atan2( y, x ) / pi and the modulus sqrt( x^2 + y^2 ) (saturated to 1).
 * - Modulus: x and y; sqrt( x^2 + y^2 ) (saturated to 1) and atan2( y, x ) / pi.
 * - Arctangent: x * 2^-n (scale n = 0 .. 7); 2^-n * atan( x ) / pi, no second argument or result.
 * - Hyperbolic cosine: x * 2^-1 in [-0.559, 0.559] (scale 1); 2^-1 * cosh x and 2^-1 * sinh x.
 * - Hyperbolic sine: x * 2^-1 in [-0.559, 0.559] (scale 1); 2^-1 * sinh x and 2^-1 * cosh x.
 * - Arctangent hyperbolic: x * 2^-1 in [-0.403, 0.403] (scale 1); 2^-1 * atanh x, no second result.
 * - Natural logarithm: x * 2^-n (scale n = 1 .. 4); 2^-(n+1) * ln x, no second result.
 * - Square root: x * 2^-n (scale n = 0 .. 2); 2^-n * sqrt( x ), no second result.
 */
typedef enum
{
    CORDIC_FUNC_COSINE                = 0u, /**< Cosine (and sine as the second result)                   */
    CORDIC_FUNC_SINE                  = 1u, /**< Sine (and cosine as the second result)                   */
    CORDIC_FUNC_PHASE                 = 2u, /**< Phase atan2( y, x ) (and the modulus)                    */
    CORDIC_FUNC_MODULUS               = 3u, /**< Modulus of the vector [x, y] (and the phase)             */
    CORDIC_FUNC_ARCTANGENT            = 4u, /**< Arctangent                                               */
    CORDIC_FUNC_HYPERBOLIC_COSINE     = 5u, /**< Hyperbolic cosine (and hyperbolic sine)                  */
    CORDIC_FUNC_HYPERBOLIC_SINE       = 6u, /**< Hyperbolic sine (and hyperbolic cosine)                  */
    CORDIC_FUNC_ARCTANGENT_HYPERBOLIC = 7u, /**< Hyperbolic arctangent                                    */
    CORDIC_FUNC_NATURAL_LOGARITHM     = 8u, /**< Natural logarithm                                        */
    CORDIC_FUNC_SQUARE_ROOT           = 9u, /**< Square root                                              */
    CORDIC_FUNC_CNT                         /**< Count of available functions                             */
}   cordic_Function_t;


/** \brief Format of the arguments and the results (CORDIC_CSR.ARGSIZE / RESSIZE) */
typedef enum
{
    CORDIC_FORMAT_Q31 = 0u, /**< 32-bit values q1.31, two data words per calculation for two arguments / results */
    CORDIC_FORMAT_Q15 = 1u, /**< 16-bit values q1.15, both values packed in one data word                         */
    CORDIC_FORMAT_CNT       /**< Count of available formats                                                       */
}   cordic_Format_t;


/**
 * \brief Precision of the calculation (CORDIC_CSR.PRECISION): the number of iterations divided by 4
 *
 * Every step adds 4 iterations (one clock cycle) and about 1 to 4 bits of accuracy, see the reference
 * manual (precision against the number of iterations). The recommended range for most functions is 3 to 6.
 */
typedef enum
{
    CORDIC_PRECISION_1  =  1u, /**<  4 iterations, 1 clock cycle   */
    CORDIC_PRECISION_2  =  2u, /**<  8 iterations, 2 clock cycles  */
    CORDIC_PRECISION_3  =  3u, /**< 12 iterations, 3 clock cycles  */
    CORDIC_PRECISION_4  =  4u, /**< 16 iterations, 4 clock cycles  */
    CORDIC_PRECISION_5  =  5u, /**< 20 iterations, 5 clock cycles  */
    CORDIC_PRECISION_6  =  6u, /**< 24 iterations, 6 clock cycles  */
    CORDIC_PRECISION_7  =  7u, /**< 28 iterations, 7 clock cycles  */
    CORDIC_PRECISION_8  =  8u, /**< 32 iterations, 8 clock cycles  */
    CORDIC_PRECISION_9  =  9u, /**< 36 iterations, 9 clock cycles  */
    CORDIC_PRECISION_10 = 10u, /**< 40 iterations, 10 clock cycles */
    CORDIC_PRECISION_11 = 11u, /**< 44 iterations, 11 clock cycles */
    CORDIC_PRECISION_12 = 12u, /**< 48 iterations, 12 clock cycles */
    CORDIC_PRECISION_13 = 13u, /**< 52 iterations, 13 clock cycles */
    CORDIC_PRECISION_14 = 14u, /**< 56 iterations, 14 clock cycles */
    CORDIC_PRECISION_15 = 15u, /**< 60 iterations, 15 clock cycles */
    CORDIC_PRECISION_CNT       /**< Count of available precision items (the value 0 is reserved) */
}   cordic_Precision_t;


/**
 * \brief Scaling exponent n of the calculation (CORDIC_CSR.SCALE)
 *
 * The arguments are multiplied by 2^-n before the calculation and the results of the functions that
 * define it are multiplied by 2^n after it, by the caller. The valid exponents depend on the function:
 * 0 for the cosine, the sine, the phase and the modulus, 0 .. 7 for the arctangent, 1 for the hyperbolic
 * functions, 1 .. 4 for the natural logarithm and 0 .. 2 for the square root. The combination is checked
 * when a calculation is started.
 */
typedef enum
{
    CORDIC_SCALE_0 = 0u, /**< Scaling exponent 0 */
    CORDIC_SCALE_1 = 1u, /**< Scaling exponent 1 */
    CORDIC_SCALE_2 = 2u, /**< Scaling exponent 2 */
    CORDIC_SCALE_3 = 3u, /**< Scaling exponent 3 */
    CORDIC_SCALE_4 = 4u, /**< Scaling exponent 4 */
    CORDIC_SCALE_5 = 5u, /**< Scaling exponent 5 */
    CORDIC_SCALE_6 = 6u, /**< Scaling exponent 6 */
    CORDIC_SCALE_7 = 7u, /**< Scaling exponent 7 */
    CORDIC_SCALE_CNT     /**< Count of available scaling exponents */
}   cordic_Scale_t;


/** \brief Data transfer handling style of the array calculation */
typedef enum
{
    CORDIC_XFER_MODE_POLL = 0u, /**< The array is calculated by \ref Cordic_Task, a few calculations per call */
    CORDIC_XFER_MODE_ISR  = 1u, /**< The array is calculated in the CORDIC interrupt                           */
    CORDIC_XFER_MODE_DMA  = 2u, /**< The array is moved by two Gpdma channels, no CPU load                      */
    CORDIC_XFER_MODE_CNT        /**< Count of available transfer modes                                          */
}   cordic_XferMode_t;


/** \brief State of the array calculation of the module */
typedef enum
{
    CORDIC_STATE_IDLE  = 0u, /**< No array calculation is running                                  */
    CORDIC_STATE_BUSY  = 1u, /**< An array calculation is running, the configuration is locked      */
    CORDIC_STATE_ERROR = 2u, /**< The last array calculation ended with an error (see the callback) */
    CORDIC_STATE_CNT         /**< Count of available states                                         */
}   cordic_State_t;


/** \brief List of the transfer errors, reported as a bit mask to the error callback */
typedef enum
{
    CORDIC_XFER_ERROR_NONE    = 0x00u, /**< No error                                         */
    CORDIC_XFER_ERROR_DMA     = 0x01u, /**< Error of a Gpdma channel                         */
    CORDIC_XFER_ERROR_TIMEOUT = 0x02u, /**< The result was not ready in the expected time    */
    CORDIC_XFER_ERROR_ALL     = 0x03u  /**< All possible errors                              */
}   cordic_XferError_t;


/** \brief Callback reporting the end of an array calculation (all results are stored) */
typedef void ( cordic_XferCompleteCallback_t )( void );


/** \brief Callback reporting that the half of the results of an array calculation is stored */
typedef void ( cordic_XferHalfCallback_t )( void );


/** \brief Callback reporting an error of an array calculation, the mask is made of \ref cordic_XferError_t items */
typedef void ( cordic_XferErrorCallback_t )( cordic_XferError_t xferError );


/**
 * \brief Arguments of a single calculation
 *
 * In \ref CORDIC_FORMAT_Q31 both values are q1.31 numbers. In \ref CORDIC_FORMAT_Q15 both values are q1.15
 * numbers sign extended to 32 bits. Value2 is ignored by the functions with one argument.
 */
typedef struct
{
    cordic_Q31_t Value1; /**< Primary argument (angle, x coordinate or the value)                 */
    cordic_Q31_t Value2; /**< Secondary argument (modulus or y coordinate), if the function has it */
}   cordic_Arguments_t;


/**
 * \brief Results of a single calculation
 *
 * The format of the values is the one of \ref cordic_Arguments_t. Value2 is zero for the functions
 * with one result.
 */
typedef struct
{
    cordic_Q31_t Value1; /**< Primary result                                  */
    cordic_Q31_t Value2; /**< Secondary result, zero if the function has none */
}   cordic_Results_t;


/**
 * \brief Request of an array calculation (see \ref Cordic_Set_CalculateArray)
 *
 * The arrays hold 32-bit data words, the layout per calculation is the one of the data registers: in
 * \ref CORDIC_FORMAT_Q31 one word per value (the primary value first), in \ref CORDIC_FORMAT_Q15 one word
 * with both values packed (see \ref CORDIC_Q15_PACK). The input array has Count calculations of one or
 * two arguments, the output array Count calculations of one or two results, the numbers follow the function.
 */
typedef struct
{
    const cordic_Q31_t * Input;  /**< Input array, 4 byte aligned in the DMA mode  */
    cordic_Q31_t       * Output; /**< Output array, 4 byte aligned in the DMA mode */
    cordic_Count_t       Count;  /**< Number of the calculations (at least 1)      */
}   cordic_ArrayRequest_t;


/** \brief Type used to store the Cordic module configuration (see \ref Cordic_Init) */
typedef struct
{
    cordic_Function_t               Function;             /**< Function of the calculation                    */
    cordic_Format_t                 Format;               /**< Format of the arguments and the results        */
    cordic_Precision_t              Precision;            /**< Precision (number of iterations / 4)           */
    cordic_Scale_t                  Scale;                /**< Scaling exponent, valid for the function       */
    cordic_XferMode_t               XferMode;             /**< Transfer mode of the array calculation         */
    cordic_IrqPrio_t                IrqPriority;          /**< CORDIC interrupt priority (ISR mode)           */
    cordic_DmaPeriphId_t            DmaPeriphId;          /**< Gpdma controller of both channels (DMA mode)   */
    cordic_DmaChannelId_t           DmaWriteChannelId;    /**< Gpdma channel feeding the arguments (DMA mode) */
    cordic_DmaChannelId_t           DmaReadChannelId;     /**< Gpdma channel taking the results (DMA mode)    */
    cordic_DmaPriority_t            DmaWritePriority;     /**< Priority of the argument channel (DMA mode)    */
    cordic_DmaPriority_t            DmaReadPriority;      /**< Priority of the result channel (DMA mode)      */
    cordic_XferCompleteCallback_t * XferCompleteCallback; /**< End of an array, may be CORDIC_NULL_PTR        */
    cordic_XferHalfCallback_t *     XferHalfCallback;     /**< Half of an array, may be CORDIC_NULL_PTR       */
    cordic_XferErrorCallback_t *    XferErrorCallback;    /**< Error of an array, may be CORDIC_NULL_PTR      */
}   cordic_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */


#endif /* CORDIC_CORDIC_TYPES_H */
