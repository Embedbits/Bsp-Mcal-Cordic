/**
 * \author Mr.Nobody
 * \file Cordic.c
 * \ingroup Cordic
 * \brief Cordic module common functionality
 *
 * # Introduction #
 * The CORDIC coprocessor evaluates the trigonometric and hyperbolic functions, the natural logarithm and
 * the square root in the fixed-point format in a few clock cycles (RM0481 / RM0517 CORDIC chapter). The
 * module selects the function, the precision, the scaling exponent and the format (q1.31 or q1.15), starts
 * a single calculation and returns the results, and converts the application values from and to the
 * fixed-point format with a range check.
 *
 * # Single calculation #
 * \ref Cordic_Set_Calculate checks the module state, the function / scale combination and the arguments
 * against the ranges of the reference manual before a register is written, writes the arguments to
 * CORDIC_WDATA (this starts the calculation), waits (bounded loop) for CORDIC_CSR.RRDY and reads the
 * results from CORDIC_RDATA. The number of the data words follows the function and the format
 * (CORDIC_CSR.NARGS / NRES are programmed per function in the 32-bit format and stay 0 in the 16-bit
 * format, where both values are packed in one word).
 *
 * # Array calculation #
 * \ref Cordic_Set_CalculateArray calculates an array of the inputs into an array of the outputs with the
 * function, precision, scale and format set at that moment (the configuration is locked until the end).
 * The data layout is the one of the data registers: one 32-bit word per value in the q1.31 format, both
 * values packed in one word in the q1.15 format. The transfer mode of the configuration selects who moves
 * the data:
 * - \ref CORDIC_XFER_MODE_POLL (POLL): the request is only stored, \ref Cordic_Task calculates a few calculations
 *   per call (write of the arguments, bounded wait for CORDIC_CSR.RRDY, read of the results).
 * - \ref CORDIC_XFER_MODE_ISR (ISR): CORDIC_CSR.IEN is set, the CORDIC interrupt reads the results of a calculation and
 *   writes the arguments of the next one.
 * - \ref CORDIC_XFER_MODE_DMA (DMA): two Gpdma channels (memory to CORDIC_WDATA with the request
 *   CORDIC_WRITE, CORDIC_RDATA to memory with the request CORDIC_READ) are configured and
 *   CORDIC_CSR.DMAREN / DMAWEN are set, the CPU is not used; the transfer complete event of the result
 *   channel ends the request. The channels are released (Gpdma_Deinit) when the next request starts or in
 *   \ref Cordic_Deinit. The elements of the arrays are not checked (no range check as in the single
 *   calculation).
 * The end, the half and an error of the request are reported by the callbacks of the configuration.
 *
 * # Device differences (STM32H5) #
 * STM32H503, STM32H523 and STM32H533 have no CORDIC peripheral. On those lines every public function
 * checks the CMSIS instance macro (defined(CORDIC)) and returns CORDIC_REQUEST_ERROR
 * (Cordic_Get_ModuleVersion still returns the version, Cordic_Task only returns).
 *
 * # Scale #
 * The scaling is done by the caller (arguments multiplied by 2^-n before, the results multiplied by the
 * factor of the function after the calculation). The module checks that the exponent is valid for the
 * function when a calculation is started, a setter does not check it, so that the function and the
 * scale can be changed one after the other.
 *
 */
/* ============================== INCLUDES ================================== */
#include "Cordic.h"                         /* Self include                   */
#include "Cordic_Port.h"                    /* Own port file include          */
#include "Cordic_Types.h"                   /* Module types definitions       */
#include "Cordic_Reg.h"                     /* Register access                */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Gpdma_Port.h"                     /* DMA handler functionality      */
#include "Stm32.h"                          /* CORDIC registers definition    */
/* ============================== TYPEDEFS ================================== */

#if defined(CORDIC)

/** \brief Settings of the coprocessor that are written to CORDIC_CSR */
typedef struct
{
    cordic_Function_t  Function;  /**< Function of the calculation   */
    cordic_Format_t    Format;    /**< Format of arguments / results */
    cordic_Precision_t Precision; /**< Precision                     */
    cordic_Scale_t     Scale;     /**< Scaling exponent              */
}   cordic_Settings_t;


/** \brief Direction of a DMA channel of the array calculation */
typedef enum
{
    CORDIC_DMA_DIR_WRITE = 0u, /**< Channel moving the arguments from the memory to CORDIC_WDATA */
    CORDIC_DMA_DIR_READ      , /**< Channel moving the results from CORDIC_RDATA to the memory   */
    CORDIC_DMA_DIR_CNT         /**< Count of the DMA channels of the array calculation            */
}   cordic_DmaDir_t;


/** \brief Runtime data of the module */
typedef struct
{
    cordic_FunctionState_t          InitState;                          /**< Cordic_Init succeeded                  */
    cordic_Settings_t               Settings;                           /**< Settings written to the peripheral     */
    cordic_XferMode_t               XferMode;                           /**< Transfer mode of the array calculation */
    cordic_IrqPrio_t                IrqPriority;                        /**< Configured interrupt priority          */
    cordic_DmaPeriphId_t            DmaPeriphId;                        /**< Gpdma controller of both channels      */
    cordic_DmaChannelId_t           DmaChannelId[ CORDIC_DMA_DIR_CNT ]; /**< Gpdma channel of every direction       */
    cordic_DmaPriority_t            DmaPriority[ CORDIC_DMA_DIR_CNT ];  /**< Priority of the channel of a direction */
    cordic_FunctionState_t          DmaConfigured;                      /**< Gpdma channels hold a configuration    */
    cordic_XferCompleteCallback_t * XferCompleteCallback;               /**< End of an array calculation            */
    cordic_XferHalfCallback_t *     XferHalfCallback;                   /**< Half of an array calculation           */
    cordic_XferErrorCallback_t *    XferErrorCallback;                  /**< Error of an array calculation          */
    volatile cordic_State_t         State;                              /**< State of the array calculation         */
    volatile cordic_XferError_t     XferError;                          /**< Error mask of the last array           */
    const cordic_Q31_t *            ArrayInput;                         /**< Input array of the request             */
    cordic_Q31_t *                  ArrayOutput;                        /**< Output array of the request            */
    cordic_Count_t                  ArrayCount;                         /**< Number of the calculations             */
    volatile cordic_Count_t         ArrayDone;                          /**< Calculations with the results stored   */
    cordic_WordCnt_t                ArrayInWords;                       /**< Data words per calculation, arguments  */
    cordic_WordCnt_t                ArrayOutWords;                      /**< Data words per calculation, results    */
    cordic_FunctionState_t          HalfReported;                       /**< The half callback was called           */
}   cordic_RuntimeData_t;

#endif /* CORDIC */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define CORDIC_VERSION_MAJOR             ( 1u )

/** Value of minor version of SW module */
#define CORDIC_VERSION_MINOR             ( 0u )

/** Value of patch version of SW module */
#define CORDIC_VERSION_PATCH             ( 0u )

/** Default interrupt priority used by \ref Cordic_Get_DefaultConfig */
#define CORDIC_DEFAULT_IRQ_PRIO          ( 0u )

#if defined(CORDIC)

/** Number of values of a calculation with one argument / result */
#define CORDIC_VALUE_CNT_ONE             ( 1u )

/** Number of values of a calculation with two arguments / results */
#define CORDIC_VALUE_CNT_TWO             ( 2u )

/** CORDIC_CSR bits written by the module for the function, precision, scale and the data formats */
#define CORDIC_CSR_SETTINGS_MASK         ( CORDIC_CSR_FUNC      | \
                                           CORDIC_CSR_PRECISION | \
                                           CORDIC_CSR_SCALE     | \
                                           CORDIC_CSR_ARGSIZE   | \
                                           CORDIC_CSR_RESSIZE   | \
                                           CORDIC_CSR_NARGS     | \
                                           CORDIC_CSR_NRES        )

/** Number of data words of a calculation in the 16-bit format (both values packed in one word) */
#define CORDIC_WORD_CNT_PACKED           ( 1u )

/** Size of a data word in bytes */
#define CORDIC_WORD_SIZE                 ( 4u )

/** Mask of the address bits that shall be 0 for a word aligned buffer */
#define CORDIC_WORD_ALIGN_MASK           ( 3u )

/** Largest number of the calculations of an array request (the word counts stay below 2^32) */
#define CORDIC_ARRAY_COUNT_MAX           ( 0x7FFFFFFFu )

/** Number of the calculations served by one call of \ref Cordic_Task in the POLL mode */
#define CORDIC_TASK_CALC_CNT             ( 8u )

/** Burst length of the DMA transfers (one data word per request of the coprocessor) */
#define CORDIC_DMA_BURST_LEN             ( 1u )

/** CORDIC_CSR bits that start the interrupt or the DMA requests of the array calculation */
#define CORDIC_CSR_ARRAY_MASK            ( CORDIC_CSR_IEN    | \
                                           CORDIC_CSR_DMAREN | \
                                           CORDIC_CSR_DMAWEN   )

/** Factor between a q1.15 value and the q1.31 value of the same number (2^16) */
#define CORDIC_Q15_TO_Q31_FACTOR         ( 65536 )

/** Value 2^31 of the q1.31 format as an integer (one in the q1.31 format) */
#define CORDIC_Q31_UNIT                  ( 2147483648LL )

/** Divisor of the parts per million used by the range tables */
#define CORDIC_PPM_DIVISOR               ( 1000000LL )

/** Converts a number given in parts per million (0.559 = 559000) to the q1.31 format at compile time */
#define CORDIC_Q31_FROM_PPM( PPM ) \
            ( (cordic_Q31_t)( ( (int64_t)( PPM ) * CORDIC_Q31_UNIT ) / CORDIC_PPM_DIVISOR ) )

/** Range of an argument that accepts every q1.31 value */
#define CORDIC_ARG_RANGE_FULL \
            { .Min = CORDIC_Q31_MINUS_ONE, .Max = CORDIC_Q31_ONE }

/** Range of an argument that accepts the non-negative q1.31 values (the modulus) */
#define CORDIC_ARG_RANGE_POSITIVE \
            { .Min = 0, .Max = CORDIC_Q31_ONE }

/** Range of an argument that the function does not use */
#define CORDIC_ARG_RANGE_NONE \
            { .Min = 0, .Max = 0 }

/** Range of an argument symmetric around zero, the limit is given in parts per million */
#define CORDIC_ARG_RANGE_SYMMETRIC( LIMIT_PPM ) \
            { .Min = -CORDIC_Q31_FROM_PPM( LIMIT_PPM ), .Max = CORDIC_Q31_FROM_PPM( LIMIT_PPM ) }

/** Range of an argument given by the limits in parts per million */
#define CORDIC_ARG_RANGE_PPM( MIN_PPM, MAX_PPM ) \
            { .Min = CORDIC_Q31_FROM_PPM( MIN_PPM ), .Max = CORDIC_Q31_FROM_PPM( MAX_PPM ) }

/** Limit of the argument of the hyperbolic cosine and sine (x * 2^-1), RM0481 Tables 170 and 171 */
#define CORDIC_HYPERBOLIC_LIMIT_PPM      ( 559000 )

/** Limit of the argument of the hyperbolic arctangent (x * 2^-1), RM0481 Table 172 */
#define CORDIC_ATANH_LIMIT_PPM           ( 403000 )

/** Natural logarithm, scale 1: ARG1 range 0.0535 .. 0.5, RM0481 Table 174 */
#define CORDIC_LN_SCALE1_MIN_PPM         ( 53500 )
#define CORDIC_LN_SCALE1_MAX_PPM         ( 500000 )

/** Natural logarithm, scale 2: ARG1 range 0.25 .. 0.75, RM0481 Table 174 */
#define CORDIC_LN_SCALE2_MIN_PPM         ( 250000 )
#define CORDIC_LN_SCALE2_MAX_PPM         ( 750000 )

/** Natural logarithm, scale 3: ARG1 range 0.375 .. 0.875, RM0481 Table 174 */
#define CORDIC_LN_SCALE3_MIN_PPM         ( 375000 )
#define CORDIC_LN_SCALE3_MAX_PPM         ( 875000 )

/** Natural logarithm, scale 4: ARG1 range 0.4375 .. 0.584, RM0481 Table 174 */
#define CORDIC_LN_SCALE4_MIN_PPM         ( 437500 )
#define CORDIC_LN_SCALE4_MAX_PPM         ( 584000 )

/** Square root, scale 0: ARG1 range 0.027 .. 0.75, RM0481 Table 176 */
#define CORDIC_SQRT_SCALE0_MIN_PPM       ( 27000 )
#define CORDIC_SQRT_SCALE0_MAX_PPM       ( 750000 )

/** Square root, scale 1: ARG1 range 0.375 .. 0.875, RM0481 Table 176 */
#define CORDIC_SQRT_SCALE1_MIN_PPM       ( 375000 )
#define CORDIC_SQRT_SCALE1_MAX_PPM       ( 875000 )

/** Square root, scale 2: ARG1 range 0.4375 .. 0.585, RM0481 Table 176 */
#define CORDIC_SQRT_SCALE2_MIN_PPM       ( 437500 )
#define CORDIC_SQRT_SCALE2_MAX_PPM       ( 585000 )

#endif /* CORDIC */

/** Scale factor between a float number and a q1.31 value (2^31) */
#define CORDIC_Q31_SCALE_FLOAT           ( 2147483648.0f )

/** Scale factor between a float number and a q1.15 value (2^15) */
#define CORDIC_Q15_SCALE_FLOAT           ( 32768.0f )

/** The float number 1.0, the exclusive upper limit of the convertible numbers */
#define CORDIC_FLOAT_ONE                 ( 1.0f )

/** The float number -1.0, the inclusive lower limit of the convertible numbers */
#define CORDIC_FLOAT_MINUS_ONE           ( -1.0f )

/** The float number 0.5 used to round to the nearest value */
#define CORDIC_FLOAT_HALF                ( 0.5f )

/** The float number 0.0 */
#define CORDIC_FLOAT_ZERO                ( 0.0f )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(CORDIC)

/** \brief Primary argument ranges of the natural logarithm, indexed by scale - 1 (RM0481 Table 174) */
static const cordic_ArgRange_t cordic_LnArg1Range[] =
{
    CORDIC_ARG_RANGE_PPM( CORDIC_LN_SCALE1_MIN_PPM, CORDIC_LN_SCALE1_MAX_PPM ), /* Scale 1 */
    CORDIC_ARG_RANGE_PPM( CORDIC_LN_SCALE2_MIN_PPM, CORDIC_LN_SCALE2_MAX_PPM ), /* Scale 2 */
    CORDIC_ARG_RANGE_PPM( CORDIC_LN_SCALE3_MIN_PPM, CORDIC_LN_SCALE3_MAX_PPM ), /* Scale 3 */
    CORDIC_ARG_RANGE_PPM( CORDIC_LN_SCALE4_MIN_PPM, CORDIC_LN_SCALE4_MAX_PPM ), /* Scale 4 */
};

/** \brief Primary argument ranges of the square root, indexed by scale (RM0481 Table 176) */
static const cordic_ArgRange_t cordic_SqrtArg1Range[] =
{
    CORDIC_ARG_RANGE_PPM( CORDIC_SQRT_SCALE0_MIN_PPM, CORDIC_SQRT_SCALE0_MAX_PPM ), /* Scale 0 */
    CORDIC_ARG_RANGE_PPM( CORDIC_SQRT_SCALE1_MIN_PPM, CORDIC_SQRT_SCALE1_MAX_PPM ), /* Scale 1 */
    CORDIC_ARG_RANGE_PPM( CORDIC_SQRT_SCALE2_MIN_PPM, CORDIC_SQRT_SCALE2_MAX_PPM ), /* Scale 2 */
};

/** \brief Properties of the functions (RM0481 Table 164 and the function tables), indexed by cordic_Function_t */
static const cordic_FunctionProps_t cordic_FunctionProps[ CORDIC_FUNC_CNT ] =
{
    [ CORDIC_FUNC_COSINE ] = { .ArgCnt      = CORDIC_VALUE_CNT_TWO,
                               .ResCnt      = CORDIC_VALUE_CNT_TWO,
                               .ScaleMin    = CORDIC_SCALE_0,
                               .ScaleMax    = CORDIC_SCALE_0,
                               .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                               .Arg2Range   = CORDIC_ARG_RANGE_POSITIVE,
                               .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_SINE ] = { .ArgCnt      = CORDIC_VALUE_CNT_TWO,
                             .ResCnt      = CORDIC_VALUE_CNT_TWO,
                             .ScaleMin    = CORDIC_SCALE_0,
                             .ScaleMax    = CORDIC_SCALE_0,
                             .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                             .Arg2Range   = CORDIC_ARG_RANGE_POSITIVE,
                             .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_PHASE ] = { .ArgCnt      = CORDIC_VALUE_CNT_TWO,
                              .ResCnt      = CORDIC_VALUE_CNT_TWO,
                              .ScaleMin    = CORDIC_SCALE_0,
                              .ScaleMax    = CORDIC_SCALE_0,
                              .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                              .Arg2Range   = CORDIC_ARG_RANGE_FULL,
                              .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_MODULUS ] = { .ArgCnt      = CORDIC_VALUE_CNT_TWO,
                                .ResCnt      = CORDIC_VALUE_CNT_TWO,
                                .ScaleMin    = CORDIC_SCALE_0,
                                .ScaleMax    = CORDIC_SCALE_0,
                                .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                                .Arg2Range   = CORDIC_ARG_RANGE_FULL,
                                .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_ARCTANGENT ] = { .ArgCnt      = CORDIC_VALUE_CNT_ONE,
                                   .ResCnt      = CORDIC_VALUE_CNT_ONE,
                                   .ScaleMin    = CORDIC_SCALE_0,
                                   .ScaleMax    = CORDIC_SCALE_7,
                                   .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                                   .Arg2Range   = CORDIC_ARG_RANGE_NONE,
                                   .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_HYPERBOLIC_COSINE ] = { .ArgCnt      = CORDIC_VALUE_CNT_ONE,
                                          .ResCnt      = CORDIC_VALUE_CNT_TWO,
                                          .ScaleMin    = CORDIC_SCALE_1,
                                          .ScaleMax    = CORDIC_SCALE_1,
                                          .Arg1Range   = CORDIC_ARG_RANGE_SYMMETRIC( CORDIC_HYPERBOLIC_LIMIT_PPM ),
                                          .Arg2Range   = CORDIC_ARG_RANGE_NONE,
                                          .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_HYPERBOLIC_SINE ] = { .ArgCnt      = CORDIC_VALUE_CNT_ONE,
                                        .ResCnt      = CORDIC_VALUE_CNT_TWO,
                                        .ScaleMin    = CORDIC_SCALE_1,
                                        .ScaleMax    = CORDIC_SCALE_1,
                                        .Arg1Range   = CORDIC_ARG_RANGE_SYMMETRIC( CORDIC_HYPERBOLIC_LIMIT_PPM ),
                                        .Arg2Range   = CORDIC_ARG_RANGE_NONE,
                                        .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_ARCTANGENT_HYPERBOLIC ] = { .ArgCnt      = CORDIC_VALUE_CNT_ONE,
                                              .ResCnt      = CORDIC_VALUE_CNT_ONE,
                                              .ScaleMin    = CORDIC_SCALE_1,
                                              .ScaleMax    = CORDIC_SCALE_1,
                                              .Arg1Range   = CORDIC_ARG_RANGE_SYMMETRIC( CORDIC_ATANH_LIMIT_PPM ),
                                              .Arg2Range   = CORDIC_ARG_RANGE_NONE,
                                              .Arg1ByScale = CORDIC_NULL_PTR              },
    [ CORDIC_FUNC_NATURAL_LOGARITHM ] = { .ArgCnt      = CORDIC_VALUE_CNT_ONE,
                                          .ResCnt      = CORDIC_VALUE_CNT_ONE,
                                          .ScaleMin    = CORDIC_SCALE_1,
                                          .ScaleMax    = CORDIC_SCALE_4,
                                          .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                                          .Arg2Range   = CORDIC_ARG_RANGE_NONE,
                                          .Arg1ByScale = cordic_LnArg1Range           },
    [ CORDIC_FUNC_SQUARE_ROOT ] = { .ArgCnt      = CORDIC_VALUE_CNT_ONE,
                                    .ResCnt      = CORDIC_VALUE_CNT_ONE,
                                    .ScaleMin    = CORDIC_SCALE_0,
                                    .ScaleMax    = CORDIC_SCALE_2,
                                    .Arg1Range   = CORDIC_ARG_RANGE_FULL,
                                    .Arg2Range   = CORDIC_ARG_RANGE_NONE,
                                    .Arg1ByScale = cordic_SqrtArg1Range         },
};

/** \brief Runtime data of the module */
static cordic_RuntimeData_t cordic_Runtime =
{
    .InitState            = CORDIC_FUNCTION_INACTIVE,
    .Settings             = { .Function  = CORDIC_FUNC_COSINE,
                              .Format    = CORDIC_FORMAT_Q31,
                              .Precision = CORDIC_PRECISION_6,
                              .Scale     = CORDIC_SCALE_0      },
    .XferMode             = CORDIC_XFER_MODE_POLL,
    .IrqPriority          = CORDIC_DEFAULT_IRQ_PRIO,
    .DmaPeriphId          = GPDMA_PERIPH_1,
    .DmaChannelId         = { [ CORDIC_DMA_DIR_WRITE ] = GPDMA_CHANNEL_0,
                              [ CORDIC_DMA_DIR_READ  ] = GPDMA_CHANNEL_1   },
    .DmaPriority          = { [ CORDIC_DMA_DIR_WRITE ] = GPDMA_PRIORITY_LOW,
                              [ CORDIC_DMA_DIR_READ  ] = GPDMA_PRIORITY_LOW },
    .DmaConfigured        = CORDIC_FUNCTION_INACTIVE,
    .XferCompleteCallback = CORDIC_NULL_PTR,
    .XferHalfCallback     = CORDIC_NULL_PTR,
    .XferErrorCallback    = CORDIC_NULL_PTR,
    .State                = CORDIC_STATE_IDLE,
    .XferError            = CORDIC_XFER_ERROR_NONE,
    .ArrayInput           = CORDIC_NULL_PTR,
    .ArrayOutput          = CORDIC_NULL_PTR,
    .ArrayCount           = 0u,
    .ArrayDone            = 0u,
    .ArrayInWords         = 0u,
    .ArrayOutWords        = 0u,
    .HalfReported         = CORDIC_FUNCTION_INACTIVE,
};

/** \brief Transfer lists of the Gpdma channels (static, a Gpdma requirement), indexed by cordic_DmaDir_t */
static volatile gpdma_XferList_t cordic_DmaXferList[ CORDIC_DMA_DIR_CNT ];

/* ======================== FORWARD DECLARATIONS ============================ */

static const cordic_FunctionProps_t * Cordic_Get_FunctionProps    ( cordic_Function_t function );
static cordic_RequestState_t          Cordic_Check_Settings       ( const cordic_Settings_t * const settings );
static cordic_RequestState_t          Cordic_Check_Available      ( void );
static cordic_RequestState_t          Cordic_Check_Initialized    ( void );
static cordic_RequestState_t          Cordic_Check_Range          ( cordic_Q31_t value,
                                                                    const cordic_ArgRange_t * const range );
static cordic_RequestState_t          Cordic_Get_Normalized       ( cordic_Format_t format,
                                                                    cordic_Q31_t value,
                                                                    cordic_Q31_t * const normalized );
static cordic_RequestState_t          Cordic_Check_Arguments      ( const cordic_Settings_t * const settings,
                                                                    const cordic_Arguments_t * const arguments );
static cordic_RequestState_t          Cordic_Write_Settings       ( const cordic_Settings_t * const settings );
static cordic_RequestState_t          Cordic_Set_Settings         ( const cordic_Settings_t * const settings );
static cordic_RequestState_t          Cordic_Reset_Peripheral     ( void );
static cordic_RequestState_t          Cordic_Release_Peripheral   ( void );
static cordic_RequestState_t          Cordic_Check_DmaConfig      ( const cordic_Config_t * const cordicConfig );
static cordic_RequestState_t          Cordic_Check_Config         ( const cordic_Config_t * const cordicConfig );
static cordic_RequestState_t          Cordic_Start_Peripheral     ( const cordic_Settings_t * const settings );
static void                           Cordic_Write_Arguments      ( const cordic_Settings_t * const settings,
                                                                    const cordic_Arguments_t * const arguments );
static void                           Cordic_Read_Results         ( const cordic_Settings_t * const settings,
                                                                    cordic_Results_t * const results );
static cordic_RequestState_t          Cordic_Run_Calculation      ( const cordic_Arguments_t * const arguments,
                                                                    cordic_Results_t * const results );
static cordic_RequestState_t          Cordic_Start_Irq            ( void );
static cordic_RequestState_t          Cordic_Get_WordCnt          ( const cordic_Settings_t * const settings,
                                                                    cordic_WordCnt_t * const inWords,
                                                                    cordic_WordCnt_t * const outWords );
static cordic_RequestState_t          Cordic_Check_DmaRequest     ( const cordic_ArrayRequest_t * const request,
                                                                    cordic_WordCnt_t inWords,
                                                                    cordic_WordCnt_t outWords );
static cordic_RequestState_t          Cordic_Check_Request        ( const cordic_ArrayRequest_t * const request,
                                                                    cordic_WordCnt_t * const inWords,
                                                                    cordic_WordCnt_t * const outWords );
static void                           Cordic_Array_Write_Next     ( void );
static void                           Cordic_Array_Read_Next      ( void );
static void                           Cordic_Array_Report_Half    ( void );
static void                           Cordic_Array_Finish         ( cordic_XferError_t xferError );
static void                           Cordic_Array_Poll           ( void );
static cordic_RequestState_t          Cordic_Release_Dma          ( void );
static cordic_RequestState_t          Cordic_Init_DmaChannel      ( cordic_DmaDir_t dmaDir,
                                                                    uint32_t memAddr,
                                                                    cordic_WordCnt_t wordCnt );
static cordic_RequestState_t          Cordic_Start_Dma            ( const cordic_ArrayRequest_t * const request );
static cordic_RequestState_t          Cordic_Start_Array          ( const cordic_ArrayRequest_t * const request );
static cordic_RequestState_t          Cordic_Run_Array            ( const cordic_ArrayRequest_t * const request );
static void                           Cordic_IsrHandler           ( void );
static void                           Cordic_Dma_ReadCompleteIsr  ( void );
static void                           Cordic_Dma_ReadHalfIsr      ( void );
static void                           Cordic_Dma_ErrorIsr         ( gpdma_ErrorMaskId_t errMask );

#endif /* CORDIC */

/* ========================== PRIVATE FUNCTIONS ============================= */

#if defined(CORDIC)

/**
 * \brief Gets the properties of a function.
 *
 * \param function [in] : Function identification.
 *
 * \return Pointer to the properties, CORDIC_NULL_PTR for an invalid function.
 */
static const cordic_FunctionProps_t * Cordic_Get_FunctionProps( cordic_Function_t function )
{
    const cordic_FunctionProps_t * retProps = CORDIC_NULL_PTR;

    if( CORDIC_FUNC_CNT > function )
    {
        retProps = &cordic_FunctionProps[ function ];
    }
    else
    {
        /* Invalid function, no properties */
        retProps = CORDIC_NULL_PTR;
    }

    return ( retProps );
}


/**
 * \brief Checks the settings: every item is in its list and the scale is valid for the function.
 *
 * \param settings [in] : Settings to check.
 *
 * \return CORDIC_REQUEST_OK when the settings can be written, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_Settings( const cordic_Settings_t * const settings )
{
    cordic_RequestState_t          retState = CORDIC_REQUEST_ERROR;
    const cordic_FunctionProps_t * props    = CORDIC_NULL_PTR;

    if( ( CORDIC_FORMAT_CNT    >  settings->Format    ) &&
        ( CORDIC_PRECISION_1   <= settings->Precision ) &&
        ( CORDIC_PRECISION_CNT >  settings->Precision ) &&
        ( CORDIC_SCALE_CNT     >  settings->Scale     )    )
    {
        props = Cordic_Get_FunctionProps( settings->Function );
    }
    else
    {
        /* Invalid format, precision or scale, the function properties stay unavailable */
        props = CORDIC_NULL_PTR;
    }

    if( CORDIC_NULL_PTR != props )
    {
        if( ( props->ScaleMin <= settings->Scale ) &&
            ( props->ScaleMax >= settings->Scale )    )
        {
            retState = CORDIC_REQUEST_OK;
        }
        else
        {
            /* The scale is not valid for the function */
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid settings */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that the module is initialized.
 *
 * \return CORDIC_REQUEST_OK when Cordic_Init succeeded, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_Initialized( void )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

    if( CORDIC_FUNCTION_ACTIVE == cordic_Runtime.InitState )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* The module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that the module is initialized and that no array calculation is running.
 *
 * \return CORDIC_REQUEST_OK when the configuration may be changed or a calculation started,
 *         CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_Available( void )
{
    cordic_RequestState_t retState = Cordic_Check_Initialized();

    if( CORDIC_REQUEST_OK == retState )
    {
        const cordic_State_t moduleState = cordic_Runtime.State;

        if( CORDIC_STATE_BUSY != moduleState )
        {
            retState = CORDIC_REQUEST_OK;
        }
        else
        {
            /* An array calculation is running */
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that a q1.31 value is inside of a closed range.
 *
 * \param value [in] : Value to check.
 * \param range [in] : Range of the accepted values.
 *
 * \return CORDIC_REQUEST_OK when the value is inside of the range, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_Range( cordic_Q31_t value, const cordic_ArgRange_t * const range )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

    if( ( range->Min <= value ) &&
        ( range->Max >= value )    )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* The value is outside of the range */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Converts an argument of the configured format to the q1.31 value of the same number.
 *
 * A q1.15 value must be inside of the q1.15 range, then it is shifted to the q1.31 format.
 *
 * \param format     [in]  : Format of the argument.
 * \param value      [in]  : Argument value (q1.31, or q1.15 sign extended).
 * \param normalized [out] : The same number in the q1.31 format.
 *
 * \return CORDIC_REQUEST_OK when the value fits the format, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Get_Normalized( cordic_Format_t format,
                                                    cordic_Q31_t value,
                                                    cordic_Q31_t * const normalized )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_OK;

    if( CORDIC_FORMAT_Q15 == format )
    {
        if( ( CORDIC_Q15_MINUS_ONE <= value ) &&
            ( CORDIC_Q15_ONE       >= value )    )
        {
            *normalized = ( value * CORDIC_Q15_TO_Q31_FACTOR );
        }
        else
        {
            /* The value does not fit the 16-bit format */
            *normalized = 0;
            retState    = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        *normalized = value;
    }

    return ( retState );
}


/**
 * \brief Checks the arguments of a calculation against the ranges of the function and the scale.
 *
 * \param settings  [in] : Settings of the calculation.
 * \param arguments [in] : Arguments to check, the secondary one only for the functions that have it.
 *
 * \return CORDIC_REQUEST_OK when the arguments are valid, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_Arguments( const cordic_Settings_t * const settings,
                                                     const cordic_Arguments_t * const arguments )
{
    cordic_RequestState_t          retState = CORDIC_REQUEST_ERROR;
    const cordic_FunctionProps_t * props    = Cordic_Get_FunctionProps( settings->Function );
    cordic_Q31_t                   value1   = 0;
    cordic_Q31_t                   value2   = 0;

    if( CORDIC_NULL_PTR != props )
    {
        retState = Cordic_Get_Normalized( settings->Format, arguments->Value1, &value1 );
    }
    else
    {
        /* Invalid function */
        retState = CORDIC_REQUEST_ERROR;
    }

    if( ( CORDIC_REQUEST_OK    == retState      ) &&
        ( CORDIC_VALUE_CNT_TWO == props->ArgCnt )    )
    {
        retState = Cordic_Get_Normalized( settings->Format, arguments->Value2, &value2 );
    }
    else
    {
        /* The secondary argument is not used or the primary one was rejected */
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        const cordic_ArgRange_t * range1 = &props->Arg1Range;

        if( CORDIC_NULL_PTR != props->Arg1ByScale )
        {
            range1 = &props->Arg1ByScale[ (uint32_t)settings->Scale - (uint32_t)props->ScaleMin ];
        }
        else
        {
            /* The range of the primary argument does not depend on the scale */
        }

        retState = Cordic_Check_Range( value1, range1 );
    }
    else
    {
        /* An argument does not fit the format */
    }

    if( ( CORDIC_REQUEST_OK    == retState      ) &&
        ( CORDIC_VALUE_CNT_TWO == props->ArgCnt )    )
    {
        retState = Cordic_Check_Range( value2, &props->Arg2Range );
    }
    else
    {
        /* The secondary argument is not used or an argument was rejected */
    }

    return ( retState );
}


/**
 * \brief Writes the settings to CORDIC_CSR and verifies them by the read-back.
 *
 * The enable bits of the interrupt and of the DMA channels are not touched.
 *
 * \param settings [in] : Settings to write, already checked.
 *
 * \return CORDIC_REQUEST_OK when the register holds the settings, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Write_Settings( const cordic_Settings_t * const settings )
{
    cordic_RequestState_t          retState = CORDIC_REQUEST_ERROR;
    const cordic_FunctionProps_t * props    = Cordic_Get_FunctionProps( settings->Function );

    if( CORDIC_NULL_PTR != props )
    {
        uint32_t csrValue = ( (uint32_t)settings->Function  << CORDIC_CSR_FUNC_Pos      ) |
                            ( (uint32_t)settings->Precision << CORDIC_CSR_PRECISION_Pos ) |
                            ( (uint32_t)settings->Scale     << CORDIC_CSR_SCALE_Pos     );

        if( CORDIC_FORMAT_Q15 == settings->Format )
        {
            csrValue |= ( CORDIC_CSR_ARGSIZE | CORDIC_CSR_RESSIZE );
        }
        else
        {
            if( CORDIC_VALUE_CNT_TWO == props->ArgCnt )
            {
                csrValue |= CORDIC_CSR_NARGS;
            }
            else
            {
                /* One argument, one data word */
            }

            if( CORDIC_VALUE_CNT_TWO == props->ResCnt )
            {
                csrValue |= CORDIC_CSR_NRES;
            }
            else
            {
                /* One result, one data word */
            }
        }

        Cordic_Reg_Set_RegVal( CORDIC_REG_CSR, CORDIC_CSR_SETTINGS_MASK, csrValue );

        const uint32_t readBack = Cordic_Reg_Get_RegVal( CORDIC_REG_CSR, CORDIC_CSR_SETTINGS_MASK );

        if( csrValue == readBack )
        {
            retState = CORDIC_REQUEST_OK;
        }
        else
        {
            /* The register does not hold the settings */
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid function */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Writes new settings to the peripheral and, when verified, stores them as the actual ones.
 *
 * \param settings [in] : New settings, items checked by the caller.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR otherwise (the stored settings are unchanged).
 */
static cordic_RequestState_t Cordic_Set_Settings( const cordic_Settings_t * const settings )
{
    const cordic_RequestState_t retState = Cordic_Write_Settings( settings );

    if( CORDIC_REQUEST_OK == retState )
    {
        cordic_Runtime.Settings = *settings;
    }
    else
    {
        /* The register does not hold the settings, the stored ones stay */
    }

    return ( retState );
}


/**
 * \brief Resets the CORDIC peripheral through Rcc (reset pulse, the registers get the reset values).
 *
 * \return CORDIC_REQUEST_OK when the reset was activated and released, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Reset_Peripheral( void )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;
    rcc_RequestState_t    rccState = Rcc_Set_ResetActive( RCC_PERIPH_CORDIC );

    if( RCC_REQUEST_OK == rccState )
    {
        rccState = Rcc_Set_ResetInactive( RCC_PERIPH_CORDIC );
    }
    else
    {
        /* The reset could not be activated */
    }

    if( RCC_REQUEST_OK == rccState )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases the peripheral: the Gpdma channels and the interrupt are released, reset pulse and the
 *        peripheral clock off, the module is not initialized.
 *
 * \return CORDIC_REQUEST_OK when every step succeeded, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Release_Peripheral( void )
{
    cordic_RequestState_t       retState  = Cordic_Release_Dma();
    const nvic_RequestState_t   nvicState = Nvic_Set_PeriphIrq_Inactive( NVIC_PERIPH_IRQ_CORDIC );
    const cordic_RequestState_t resetState = Cordic_Reset_Peripheral();
    const rcc_RequestState_t    rccState  = Rcc_Set_PeriphInactive( RCC_PERIPH_CORDIC );

    cordic_Runtime.InitState = CORDIC_FUNCTION_INACTIVE;
    cordic_Runtime.State     = CORDIC_STATE_IDLE;

    if( ( CORDIC_REQUEST_OK == retState   ) &&
        ( NVIC_REQUEST_OK   == nvicState  ) &&
        ( CORDIC_REQUEST_OK == resetState ) &&
        ( RCC_REQUEST_OK    == rccState   )    )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the DMA items of the configuration: valid and different Gpdma channels.
 *
 * \param cordicConfig [in] : Configuration to check, not NULL.
 *
 * \return CORDIC_REQUEST_OK when the Gpdma controller, the channels and the priorities are valid,
 *         CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_DmaConfig( const cordic_Config_t * const cordicConfig )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

    if( ( GPDMA_PERIPH_CNT              >  cordicConfig->DmaPeriphId        ) &&
        ( GPDMA_CHANNEL_CNT             >  cordicConfig->DmaWriteChannelId  ) &&
        ( GPDMA_CHANNEL_CNT             >  cordicConfig->DmaReadChannelId   ) &&
        ( GPDMA_PRIORITY_CNT            >  cordicConfig->DmaWritePriority   ) &&
        ( GPDMA_PRIORITY_CNT            >  cordicConfig->DmaReadPriority    ) &&
        ( cordicConfig->DmaReadChannelId != cordicConfig->DmaWriteChannelId )    )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* The DMA mode needs two different valid Gpdma channels */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks the configuration of \ref Cordic_Init: the transfer mode, the settings, the DMA items of the
 *        DMA mode and that the module is not initialized.
 *
 * \param cordicConfig [in] : Configuration to check, not NULL.
 *
 * \return CORDIC_REQUEST_OK when the module can be initialized with the configuration, CORDIC_REQUEST_ERROR
 *         otherwise.
 */
static cordic_RequestState_t Cordic_Check_Config( const cordic_Config_t * const cordicConfig )
{
    cordic_RequestState_t   retState = CORDIC_REQUEST_ERROR;
    const cordic_Settings_t settings = { .Function  = cordicConfig->Function,
                                         .Format    = cordicConfig->Format,
                                         .Precision = cordicConfig->Precision,
                                         .Scale     = cordicConfig->Scale     };

    if( ( CORDIC_XFER_MODE_CNT     >  cordicConfig->XferMode ) &&
        ( CORDIC_FUNCTION_INACTIVE == cordic_Runtime.InitState )    )
    {
        retState = Cordic_Check_Settings( &settings );
    }
    else
    {
        /* Invalid transfer mode or the module is already initialized */
        retState = CORDIC_REQUEST_ERROR;
    }

    if( ( CORDIC_REQUEST_OK    == retState               ) &&
        ( CORDIC_XFER_MODE_DMA == cordicConfig->XferMode )    )
    {
        retState = Cordic_Check_DmaConfig( cordicConfig );
    }
    else
    {
        /* Not the DMA mode or the configuration is already rejected */
    }

    return ( retState );
}


/**
 * \brief Starts the peripheral: enables its clock, resets it and writes the settings (verified by read-back).
 *
 * \param settings [in] : Settings to write, already checked.
 *
 * \return CORDIC_REQUEST_OK when the peripheral holds the settings, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Start_Peripheral( const cordic_Settings_t * const settings )
{
    cordic_RequestState_t    retState = CORDIC_REQUEST_ERROR;
    const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( RCC_PERIPH_CORDIC );

    if( RCC_REQUEST_OK == rccState )
    {
        retState = Cordic_Reset_Peripheral();
    }
    else
    {
        /* The peripheral clock could not be enabled */
        retState = CORDIC_REQUEST_ERROR;
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Set_Settings( settings );
    }
    else
    {
        /* The peripheral could not be reset */
    }

    return ( retState );
}


/**
 * \brief Writes the arguments to CORDIC_WDATA, the last write starts the calculation.
 *
 * \param settings  [in] : Settings of the calculation (format and function).
 * \param arguments [in] : Arguments, already checked.
 */
static void Cordic_Write_Arguments( const cordic_Settings_t * const settings,
                                    const cordic_Arguments_t * const arguments )
{
    const cordic_FunctionProps_t * const props = Cordic_Get_FunctionProps( settings->Function );

    if( CORDIC_NULL_PTR != props )
    {
        if( CORDIC_FORMAT_Q15 == settings->Format )
        {
            cordic_Q31_t secondValue = 0;

            if( CORDIC_VALUE_CNT_TWO == props->ArgCnt )
            {
                secondValue = arguments->Value2;
            }
            else
            {
                /* The secondary argument is not used, its half-word stays 0 */
            }

            Cordic_Reg_Set_Data( (uint32_t)CORDIC_Q15_PACK( arguments->Value1, secondValue ) );
        }
        else
        {
            Cordic_Reg_Set_Data( (uint32_t)arguments->Value1 );

            if( CORDIC_VALUE_CNT_TWO == props->ArgCnt )
            {
                Cordic_Reg_Set_Data( (uint32_t)arguments->Value2 );
            }
            else
            {
                /* One argument, one data word */
            }
        }
    }
    else
    {
        /* Invalid function, nothing is written */
    }
}


/**
 * \brief Reads the results from CORDIC_RDATA, the number of the reads follows CORDIC_CSR.NRES.
 *
 * \param settings [in]  : Settings of the calculation (format and function).
 * \param results  [out] : Results, the secondary one is 0 for the functions with one result.
 */
static void Cordic_Read_Results( const cordic_Settings_t * const settings, cordic_Results_t * const results )
{
    const cordic_FunctionProps_t * const props = Cordic_Get_FunctionProps( settings->Function );

    results->Value1 = 0;
    results->Value2 = 0;

    if( CORDIC_NULL_PTR != props )
    {
        if( CORDIC_FORMAT_Q15 == settings->Format )
        {
            const uint32_t dataWord = Cordic_Reg_Get_Data();

            results->Value1 = (cordic_Q31_t)CORDIC_Q15_FIRST( dataWord );

            if( CORDIC_VALUE_CNT_TWO == props->ResCnt )
            {
                results->Value2 = (cordic_Q31_t)CORDIC_Q15_SECOND( dataWord );
            }
            else
            {
                /* One result, the upper half-word is not a result */
            }
        }
        else
        {
            results->Value1 = (cordic_Q31_t)Cordic_Reg_Get_Data();

            if( CORDIC_VALUE_CNT_TWO == props->ResCnt )
            {
                results->Value2 = (cordic_Q31_t)Cordic_Reg_Get_Data();
            }
            else
            {
                /* One result, one data word */
            }
        }
    }
    else
    {
        /* Invalid function, no result */
    }
}


/**
 * \brief Runs a single calculation: checks, writes the arguments, waits for the result and reads it.
 *
 * A result that is not ready in time resets the peripheral and writes the settings again.
 *
 * \param arguments [in]  : Arguments, not NULL.
 * \param results   [out] : Results, not NULL.
 *
 * \return CORDIC_REQUEST_OK when the results were read, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Run_Calculation( const cordic_Arguments_t * const arguments,
                                                     cordic_Results_t * const results )
{
    cordic_RequestState_t retState = Cordic_Check_Available();

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Check_Settings( &cordic_Runtime.Settings );
    }
    else
    {
        /* The module is not initialized or an array calculation is running */
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Check_Arguments( &cordic_Runtime.Settings, arguments );
    }
    else
    {
        /* The settings are not a valid combination */
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        Cordic_Write_Arguments( &cordic_Runtime.Settings, arguments );

        retState = Cordic_Reg_Wait_ResultReady();

        if( CORDIC_REQUEST_OK == retState )
        {
            Cordic_Read_Results( &cordic_Runtime.Settings, results );
        }
        else
        {
            /* The result is not ready: the peripheral state is unknown, reset it */
            (void)Cordic_Reset_Peripheral();
            (void)Cordic_Write_Settings( &cordic_Runtime.Settings );
        }
    }
    else
    {
        /* The arguments are not valid, nothing is written */
    }

    return ( retState );
}

/**
 * \brief Configures the CORDIC interrupt in Nvic (ISR transfer mode): handler, priority and enable.
 *
 * \return CORDIC_REQUEST_OK when the interrupt is enabled, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Start_Irq( void )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;
    nvic_RequestState_t   nvicState = Nvic_Set_PeriphIrq_Handler( NVIC_PERIPH_IRQ_CORDIC, Cordic_IsrHandler );

    if( NVIC_REQUEST_OK == nvicState )
    {
        nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_CORDIC, cordic_Runtime.IrqPriority );
    }
    else
    {
        /* The handler registration failed */
    }

    if( NVIC_REQUEST_OK == nvicState )
    {
        nvicState = Nvic_Set_PeriphIrq_Active( NVIC_PERIPH_IRQ_CORDIC );
    }
    else
    {
        /* The handler registration or the priority failed */
    }

    if( NVIC_REQUEST_OK == nvicState )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Gets the number of the data words of one calculation, the arguments and the results.
 *
 * In the 16-bit format both values are packed in one word, in the 32-bit format every value is one word.
 *
 * \param settings [in]  : Settings of the calculation.
 * \param inWords  [out] : Data words of the arguments of one calculation.
 * \param outWords [out] : Data words of the results of one calculation.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid function.
 */
static cordic_RequestState_t Cordic_Get_WordCnt( const cordic_Settings_t * const settings,
                                                 cordic_WordCnt_t * const inWords,
                                                 cordic_WordCnt_t * const outWords )
{
    cordic_RequestState_t          retState = CORDIC_REQUEST_ERROR;
    const cordic_FunctionProps_t * props    = Cordic_Get_FunctionProps( settings->Function );

    *inWords  = 0u;
    *outWords = 0u;

    if( CORDIC_NULL_PTR != props )
    {
        if( CORDIC_FORMAT_Q15 == settings->Format )
        {
            *inWords  = CORDIC_WORD_CNT_PACKED;
            *outWords = CORDIC_WORD_CNT_PACKED;
        }
        else
        {
            *inWords  = props->ArgCnt;
            *outWords = props->ResCnt;
        }

        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Invalid function */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks an array request for the DMA transfer mode: the blocks fit the Gpdma block size and the
 *        arrays are word aligned.
 *
 * \param request  [in] : Array request, not NULL.
 * \param inWords  [in] : Data words of the arguments of one calculation.
 * \param outWords [in] : Data words of the results of one calculation.
 *
 * \return CORDIC_REQUEST_OK when the DMA can move the arrays, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_DmaRequest( const cordic_ArrayRequest_t * const request,
                                                      cordic_WordCnt_t inWords,
                                                      cordic_WordCnt_t outWords )
{
    cordic_RequestState_t retState    = CORDIC_REQUEST_ERROR;
    const cordic_Count_t  maxCountIn  = ( GPDMA_MAX_BLOCK_LEN / ( inWords  * CORDIC_WORD_SIZE ) );
    const cordic_Count_t  maxCountOut = ( GPDMA_MAX_BLOCK_LEN / ( outWords * CORDIC_WORD_SIZE ) );
    const uintptr_t       inputAddr   = (uintptr_t)request->Input;
    const uintptr_t       outputAddr  = (uintptr_t)request->Output;

    if( ( maxCountIn  >= request->Count                           ) &&
        ( maxCountOut >= request->Count                           ) &&
        ( 0u          == ( inputAddr  & CORDIC_WORD_ALIGN_MASK )  ) &&
        ( 0u          == ( outputAddr & CORDIC_WORD_ALIGN_MASK )  )    )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* The array is too long for one Gpdma block or it is not word aligned */
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks an array request and gets the data word counts of one calculation.
 *
 * \param request  [in]  : Array request, not NULL.
 * \param inWords  [out] : Data words of the arguments of one calculation.
 * \param outWords [out] : Data words of the results of one calculation.
 *
 * \return CORDIC_REQUEST_OK when the request can be started, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Check_Request( const cordic_ArrayRequest_t * const request,
                                                   cordic_WordCnt_t * const inWords,
                                                   cordic_WordCnt_t * const outWords )
{
    cordic_RequestState_t retState = Cordic_Check_Available();

    *inWords  = 0u;
    *outWords = 0u;

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Check_Settings( &cordic_Runtime.Settings );
    }
    else
    {
        /* The module is not initialized or an array calculation is running */
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Get_WordCnt( &cordic_Runtime.Settings, inWords, outWords );
    }
    else
    {
        /* The settings are not a valid combination */
    }

    if( ( CORDIC_REQUEST_OK      == retState        ) &&
        ( CORDIC_NULL_PTR        != request->Input  ) &&
        ( CORDIC_NULL_PTR        != request->Output ) &&
        ( 0u                     <  request->Count  ) &&
        ( CORDIC_ARRAY_COUNT_MAX >= request->Count  )    )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Rejected state or settings, or an invalid array / count */
        retState = CORDIC_REQUEST_ERROR;
    }

    if( ( CORDIC_REQUEST_OK    == retState                   ) &&
        ( CORDIC_XFER_MODE_DMA == cordic_Runtime.XferMode    )    )
    {
        retState = Cordic_Check_DmaRequest( request, *inWords, *outWords );
    }
    else
    {
        /* Not the DMA mode or the request is already rejected */
    }

    return ( retState );
}


/**
 * \brief Writes the arguments of the calculation number ArrayDone to CORDIC_WDATA (starts the calculation).
 */
static void Cordic_Array_Write_Next( void )
{
    const cordic_WordCnt_t offset = ( cordic_Runtime.ArrayDone * cordic_Runtime.ArrayInWords );

    for( cordic_WordCnt_t wordIdx = 0u; cordic_Runtime.ArrayInWords > wordIdx; wordIdx++ )
    {
        Cordic_Reg_Set_Data( (uint32_t)cordic_Runtime.ArrayInput[ offset + wordIdx ] );
    }
}


/**
 * \brief Reads the results of the calculation number ArrayDone from CORDIC_RDATA, reports the half of the
 *        request and finishes the request after the last calculation.
 */
static void Cordic_Array_Read_Next( void )
{
    const cordic_WordCnt_t offset = ( cordic_Runtime.ArrayDone * cordic_Runtime.ArrayOutWords );

    for( cordic_WordCnt_t wordIdx = 0u; cordic_Runtime.ArrayOutWords > wordIdx; wordIdx++ )
    {
        cordic_Runtime.ArrayOutput[ offset + wordIdx ] = (cordic_Q31_t)Cordic_Reg_Get_Data();
    }

    cordic_Runtime.ArrayDone = ( cordic_Runtime.ArrayDone + 1u );

    Cordic_Array_Report_Half();

    if( cordic_Runtime.ArrayCount <= cordic_Runtime.ArrayDone )
    {
        Cordic_Array_Finish( CORDIC_XFER_ERROR_NONE );
    }
    else
    {
        /* The request continues with the next calculation */
    }
}


/**
 * \brief Calls the half callback once, when at least the half of the calculations of the request is done.
 */
static void Cordic_Array_Report_Half( void )
{
    const cordic_Count_t doneCnt   = cordic_Runtime.ArrayDone;
    const cordic_Count_t remainCnt = ( cordic_Runtime.ArrayCount - doneCnt );

    if( ( CORDIC_FUNCTION_INACTIVE == cordic_Runtime.HalfReported ) &&
        ( remainCnt                <= doneCnt                     )    )
    {
        cordic_Runtime.HalfReported = CORDIC_FUNCTION_ACTIVE;

        if( CORDIC_NULL_PTR != cordic_Runtime.XferHalfCallback )
        {
            cordic_Runtime.XferHalfCallback();
        }
        else
        {
            /* No half callback configured */
        }
    }
    else
    {
        /* The half is not reached yet or it was reported */
    }
}


/**
 * \brief Ends the array request: the interrupt and the DMA requests of the coprocessor are disabled, an error
 *        stops the Gpdma channels and resets the peripheral (settings written again), the state is set and the
 *        callback is called.
 *
 * \param xferError [in] : CORDIC_XFER_ERROR_NONE for the end without an error, the error mask otherwise.
 */
static void Cordic_Array_Finish( cordic_XferError_t xferError )
{
    Cordic_Reg_Reset_RegBit( CORDIC_REG_CSR, CORDIC_CSR_ARRAY_MASK );

    if( CORDIC_XFER_ERROR_NONE == xferError )
    {
        cordic_Runtime.XferError = CORDIC_XFER_ERROR_NONE;
        cordic_Runtime.State     = CORDIC_STATE_IDLE;

        if( CORDIC_NULL_PTR != cordic_Runtime.XferCompleteCallback )
        {
            cordic_Runtime.XferCompleteCallback();
        }
        else
        {
            /* No complete callback configured */
        }
    }
    else
    {
        if( CORDIC_FUNCTION_ACTIVE == cordic_Runtime.DmaConfigured )
        {
            (void)Gpdma_Set_ChannelInactive( cordic_Runtime.DmaPeriphId,
                                             cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_WRITE ] );
            (void)Gpdma_Set_ChannelInactive( cordic_Runtime.DmaPeriphId,
                                             cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_READ ] );
        }
        else
        {
            /* No Gpdma channel is configured */
        }

        /* The state of the peripheral is unknown (a result may be pending): reset it */
        (void)Cordic_Reset_Peripheral();
        (void)Cordic_Write_Settings( &cordic_Runtime.Settings );

        cordic_Runtime.XferError = xferError;
        cordic_Runtime.State     = CORDIC_STATE_ERROR;

        if( CORDIC_NULL_PTR != cordic_Runtime.XferErrorCallback )
        {
            cordic_Runtime.XferErrorCallback( xferError );
        }
        else
        {
            /* No error callback configured */
        }
    }
}


/**
 * \brief Serves the running array request in the POLL transfer mode: a few calculations per call, every one
 *        writes the arguments, waits (bounded) for the result and reads it.
 */
static void Cordic_Array_Poll( void )
{
    for( uint32_t calcIdx = 0u; CORDIC_TASK_CALC_CNT > calcIdx; calcIdx++ )
    {
        const cordic_State_t moduleState = cordic_Runtime.State;

        if( CORDIC_STATE_BUSY == moduleState )
        {
            Cordic_Array_Write_Next();

            const cordic_RequestState_t waitState = Cordic_Reg_Wait_ResultReady();

            if( CORDIC_REQUEST_OK == waitState )
            {
                Cordic_Array_Read_Next();
            }
            else
            {
                Cordic_Array_Finish( CORDIC_XFER_ERROR_TIMEOUT );
            }
        }
        else
        {
            /* No array calculation is running */
        }
    }
}


/**
 * \brief Releases the Gpdma channels of the array calculation (if configured): the channels are stopped and
 *        deinitialized and their interrupts disabled.
 *
 * \return CORDIC_REQUEST_OK when nothing was configured or the channels were released, CORDIC_REQUEST_ERROR
 *         otherwise.
 */
static cordic_RequestState_t Cordic_Release_Dma( void )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_OK;

    if( CORDIC_FUNCTION_ACTIVE == cordic_Runtime.DmaConfigured )
    {
        const gpdma_PeriphId_t  periphId  = cordic_Runtime.DmaPeriphId;
        const gpdma_ChannelId_t writeCh   = cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_WRITE ];
        const gpdma_ChannelId_t readCh    = cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_READ ];
        const gpdma_RequestState_t writeState = Gpdma_Deinit( periphId, writeCh );
        const gpdma_RequestState_t readState  = Gpdma_Deinit( periphId, readCh );
        const gpdma_RequestState_t writeIrq   = Gpdma_Set_InterruptInactive( periphId, writeCh );
        const gpdma_RequestState_t readIrq    = Gpdma_Set_InterruptInactive( periphId, readCh );

        cordic_Runtime.DmaConfigured = CORDIC_FUNCTION_INACTIVE;

        if( ( GPDMA_REQUEST_OK == writeState ) &&
            ( GPDMA_REQUEST_OK == readState  ) &&
            ( GPDMA_REQUEST_OK == writeIrq   ) &&
            ( GPDMA_REQUEST_OK == readIrq    )    )
        {
            retState = CORDIC_REQUEST_OK;
        }
        else
        {
            /* A channel could not be released */
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* No Gpdma channel is configured */
    }

    return ( retState );
}


/**
 * \brief Configures one Gpdma channel of the array calculation and enables its interrupt in Nvic.
 *
 * The write channel moves the words from the memory to CORDIC_WDATA (request CORDIC_WRITE), the read channel
 * the words from CORDIC_RDATA to the memory (request CORDIC_READ); one word per request, the whole array is
 * one block. Only the result channel reports the end (and the half, if a half callback is configured); both
 * channels report the errors.
 *
 * \param dmaDir  [in] : Direction of the channel.
 * \param memAddr [in] : Bus address of the array in the memory.
 * \param wordCnt [in] : Number of the data words of the block.
 *
 * \return CORDIC_REQUEST_OK when the channel is configured, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Init_DmaChannel( cordic_DmaDir_t dmaDir,
                                                     uint32_t memAddr,
                                                     cordic_WordCnt_t wordCnt )
{
    cordic_RequestState_t  retState   = CORDIC_REQUEST_ERROR;
    gpdma_TransferConfig_t xferConfig = { 0u };
    gpdma_ConfigStruct_t   dmaConfig  = { 0u };
    gpdma_RequestState_t   dmaState   = Gpdma_Get_DefaultConfig( &dmaConfig );

    xferConfig.XferListExecMode        = GPDMA_XFER_LIST_EXEC_ONCE;
    xferConfig.EventMode               = GPDMA_TRANSFER_EVENT_BLOCK;
    xferConfig.TriggerType             = GPDMA_TRG_NOT_USED;
    xferConfig.TriggerMode             = GPDMA_TRIGGER_BLOCK;
    xferConfig.RequestMode             = GPDMA_PERIPH_REQ_SINGLE;
    xferConfig.BlockSize               = (gpdma_BlockSize_t)( wordCnt * CORDIC_WORD_SIZE );
    xferConfig.BlockRepetitionCount    = 0u;
    xferConfig.SourceDataSize          = GPDMA_DATA_SIZE_32BITS;
    xferConfig.SourceBurstLength       = CORDIC_DMA_BURST_LEN;
    xferConfig.SourcePortId            = GPDMA_PORT_DEFAULT;
    xferConfig.SourceDataOp            = GPDMA_SRC_DATA_PRESERVE;
    xferConfig.DestinationDataSize     = GPDMA_DATA_SIZE_32BITS;
    xferConfig.DestinationBurstLength  = CORDIC_DMA_BURST_LEN;
    xferConfig.DestinationPortId       = GPDMA_PORT_DEFAULT;
    xferConfig.DestinationDataOp       = GPDMA_DEST_DATA_PRESERVE;

    dmaConfig.PeriphId           = cordic_Runtime.DmaPeriphId;
    dmaConfig.ChannelId          = cordic_Runtime.DmaChannelId[ dmaDir ];
    dmaConfig.ChannelPrio        = cordic_Runtime.DmaPriority[ dmaDir ];
    dmaConfig.TransferExecMode   = GPDMA_XFER_EXEC_BLOCKING;
    dmaConfig.TransferConfig     = &xferConfig;
    dmaConfig.TransfersCount     = 1u;
    dmaConfig.XferListAccessMode = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
    dmaConfig.XferList           = &cordic_DmaXferList[ dmaDir ];
    dmaConfig.TransferLockState  = GPDMA_TRANSFER_LIST_LOCKED;
    dmaConfig.ErrorIsr           = Cordic_Dma_ErrorIsr;
    dmaConfig.ErrorMask          = GPDMA_ERROR_ALL;

    if( CORDIC_DMA_DIR_WRITE == dmaDir )
    {
        xferConfig.Direction           = GPDMA_DIR_MEMORY_TO_PERIPH;
        xferConfig.RequestSource       = GPDMA_REQ_CORDIC_WRITE;
        xferConfig.SourceAddr          = (gpdma_SrcAddr_t)memAddr;
        xferConfig.SourceAddrMode      = GPDMA_ADDR_INCREMENT;
        xferConfig.DestinationAddr     = (gpdma_DstAddr_t)Cordic_Reg_Get_WdataAddr();
        xferConfig.DestinationAddrMode = GPDMA_ADDR_STATIC;
    }
    else
    {
        xferConfig.Direction           = GPDMA_DIR_PERIPH_TO_MEMORY;
        xferConfig.RequestSource       = GPDMA_REQ_CORDIC_READ;
        xferConfig.SourceAddr          = (gpdma_SrcAddr_t)Cordic_Reg_Get_RdataAddr();
        xferConfig.SourceAddrMode      = GPDMA_ADDR_STATIC;
        xferConfig.DestinationAddr     = (gpdma_DstAddr_t)memAddr;
        xferConfig.DestinationAddrMode = GPDMA_ADDR_INCREMENT;

        dmaConfig.TransferCompleteIsr  = Cordic_Dma_ReadCompleteIsr;

        if( CORDIC_NULL_PTR != cordic_Runtime.XferHalfCallback )
        {
            dmaConfig.HalfTransferIsr  = Cordic_Dma_ReadHalfIsr;
        }
        else
        {
            /* No half callback, the half transfer interrupt stays disabled */
        }
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Init( &dmaConfig );
    }
    else
    {
        /* The default configuration is not available */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        dmaState = Gpdma_Set_InterruptActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
    }
    else
    {
        /* The channel could not be configured */
    }

    if( GPDMA_REQUEST_OK == dmaState )
    {
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        retState = CORDIC_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts the array request in the DMA transfer mode: configures both Gpdma channels, activates them
 *        (result channel first) and enables the DMA requests of the coprocessor (CORDIC_CSR.DMAREN, DMAWEN).
 *
 * The channels of the previous request are released first. A failure releases the channels again.
 *
 * \param request [in] : Array request, checked, not NULL.
 *
 * \return CORDIC_REQUEST_OK when the transfer runs, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Start_Dma( const cordic_ArrayRequest_t * const request )
{
    cordic_RequestState_t retState = Cordic_Release_Dma();
    const uint32_t        inAddr   = (uint32_t)(uintptr_t)request->Input;
    const uint32_t        outAddr  = (uint32_t)(uintptr_t)request->Output;

    if( CORDIC_REQUEST_OK == retState )
    {
        cordic_Runtime.DmaConfigured = CORDIC_FUNCTION_ACTIVE;

        retState = Cordic_Init_DmaChannel( CORDIC_DMA_DIR_WRITE,
                                           inAddr,
                                           ( request->Count * cordic_Runtime.ArrayInWords ) );
    }
    else
    {
        /* The channels of the previous request could not be released */
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Init_DmaChannel( CORDIC_DMA_DIR_READ,
                                           outAddr,
                                           ( request->Count * cordic_Runtime.ArrayOutWords ) );
    }
    else
    {
        /* The write channel could not be configured */
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        const gpdma_PeriphId_t     periphId   = cordic_Runtime.DmaPeriphId;
        const gpdma_ChannelId_t    writeCh    = cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_WRITE ];
        const gpdma_ChannelId_t    readCh     = cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_READ ];
        const gpdma_RequestState_t readState  = Gpdma_Set_ChannelActive( periphId, readCh );
        const gpdma_RequestState_t writeState = Gpdma_Set_ChannelActive( periphId, writeCh );

        if( ( GPDMA_REQUEST_OK == readState  ) &&
            ( GPDMA_REQUEST_OK == writeState )    )
        {
            Cordic_Reg_Set_RegBit( CORDIC_REG_CSR, CORDIC_CSR_DMAREN );
            Cordic_Reg_Set_RegBit( CORDIC_REG_CSR, CORDIC_CSR_DMAWEN );
        }
        else
        {
            /* A channel could not be activated */
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The read channel could not be configured */
    }

    if( CORDIC_REQUEST_OK != retState )
    {
        (void)Cordic_Release_Dma();
    }
    else
    {
        /* The transfer runs */
    }

    return ( retState );
}


/**
 * \brief Starts a checked array request in the configured transfer mode.
 *
 * The request is stored and the module is busy: the POLL mode waits for \ref Cordic_Task, the ISR mode enables the
 * interrupt and writes the first arguments, the DMA mode starts the Gpdma channels. A failure disables the
 * requests again and the module is idle.
 *
 * \param request [in] : Array request, checked, not NULL.
 *
 * \return CORDIC_REQUEST_OK when the request runs, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Start_Array( const cordic_ArrayRequest_t * const request )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_OK;

    cordic_Runtime.ArrayInput   = request->Input;
    cordic_Runtime.ArrayOutput  = request->Output;
    cordic_Runtime.ArrayCount   = request->Count;
    cordic_Runtime.ArrayDone    = 0u;
    cordic_Runtime.HalfReported = CORDIC_FUNCTION_INACTIVE;
    cordic_Runtime.XferError    = CORDIC_XFER_ERROR_NONE;
    cordic_Runtime.State        = CORDIC_STATE_BUSY;

    if( CORDIC_XFER_MODE_ISR == cordic_Runtime.XferMode )
    {
        Cordic_Reg_Set_RegBit( CORDIC_REG_CSR, CORDIC_CSR_IEN );
        Cordic_Array_Write_Next();
    }
    else if( CORDIC_XFER_MODE_DMA == cordic_Runtime.XferMode )
    {
        retState = Cordic_Start_Dma( request );
    }
    else
    {
        /* The POLL mode: the calculations are served by Cordic_Task */
    }

    if( CORDIC_REQUEST_OK != retState )
    {
        Cordic_Reg_Reset_RegBit( CORDIC_REG_CSR, CORDIC_CSR_ARRAY_MASK );
        cordic_Runtime.State = CORDIC_STATE_IDLE;
    }
    else
    {
        /* The request runs */
    }

    return ( retState );
}


/**
 * \brief Checks and starts an array request.
 *
 * \param request [in] : Array request, not NULL.
 *
 * \return CORDIC_REQUEST_OK when the request runs, CORDIC_REQUEST_ERROR otherwise.
 */
static cordic_RequestState_t Cordic_Run_Array( const cordic_ArrayRequest_t * const request )
{
    cordic_WordCnt_t      inWords  = 0u;
    cordic_WordCnt_t      outWords = 0u;
    cordic_RequestState_t retState = Cordic_Check_Request( request, &inWords, &outWords );

    if( CORDIC_REQUEST_OK == retState )
    {
        cordic_Runtime.ArrayInWords  = inWords;
        cordic_Runtime.ArrayOutWords = outWords;

        retState = Cordic_Start_Array( request );
    }
    else
    {
        /* The request is rejected, nothing is changed */
    }

    return ( retState );
}


/**
 * \brief CORDIC interrupt handler (ISR transfer mode): reads the results of the finished calculation and
 *        writes the arguments of the next one. A request that is not running is ignored.
 */
static void Cordic_IsrHandler( void )
{
    const cordic_State_t     moduleState = cordic_Runtime.State;
    const cordic_FlagState_t readyState  = Cordic_Reg_Get_ResultReady();

    if( ( CORDIC_STATE_BUSY    == moduleState              ) &&
        ( CORDIC_XFER_MODE_ISR == cordic_Runtime.XferMode  ) &&
        ( CORDIC_FLAG_ACTIVE   == readyState               )    )
    {
        Cordic_Array_Read_Next();

        const cordic_State_t nextState = cordic_Runtime.State;

        if( CORDIC_STATE_BUSY == nextState )
        {
            Cordic_Array_Write_Next();
        }
        else
        {
            /* The request is finished */
        }
    }
    else
    {
        /* No array calculation waits for a result */
    }
}


/**
 * \brief Gpdma callback, transfer complete of the result channel: all results are stored, the request ends.
 */
static void Cordic_Dma_ReadCompleteIsr( void )
{
    const cordic_State_t moduleState = cordic_Runtime.State;

    if( ( CORDIC_STATE_BUSY    == moduleState             ) &&
        ( CORDIC_XFER_MODE_DMA == cordic_Runtime.XferMode )    )
    {
        cordic_Runtime.ArrayDone = cordic_Runtime.ArrayCount;
        Cordic_Array_Report_Half();
        Cordic_Array_Finish( CORDIC_XFER_ERROR_NONE );
    }
    else
    {
        /* No array calculation runs */
    }
}


/**
 * \brief Gpdma callback, half transfer of the result channel: the half callback is called.
 */
static void Cordic_Dma_ReadHalfIsr( void )
{
    const cordic_State_t moduleState = cordic_Runtime.State;

    if( ( CORDIC_STATE_BUSY    == moduleState             ) &&
        ( CORDIC_XFER_MODE_DMA == cordic_Runtime.XferMode )    )
    {
        cordic_Runtime.HalfReported = CORDIC_FUNCTION_ACTIVE;

        if( CORDIC_NULL_PTR != cordic_Runtime.XferHalfCallback )
        {
            cordic_Runtime.XferHalfCallback();
        }
        else
        {
            /* No half callback configured */
        }
    }
    else
    {
        /* No array calculation runs */
    }
}


/**
 * \brief Gpdma callback, error of a channel: the request ends with CORDIC_XFER_ERROR_DMA.
 *
 * \param errMask [in] : Gpdma error mask, not used (the error is reported as one DMA error).
 */
static void Cordic_Dma_ErrorIsr( gpdma_ErrorMaskId_t errMask )
{
    const cordic_State_t moduleState = cordic_Runtime.State;

    (void)errMask;

    if( CORDIC_STATE_BUSY == moduleState )
    {
        Cordic_Array_Finish( CORDIC_XFER_ERROR_DMA );
    }
    else
    {
        /* No array calculation runs, a second error of the same request */
    }
}

#endif /* CORDIC */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
cordic_ModuleVersion_t Cordic_Get_ModuleVersion( void )
{
    cordic_ModuleVersion_t retVersion;

    retVersion.Major = CORDIC_VERSION_MAJOR;
    retVersion.Minor = CORDIC_VERSION_MINOR;
    retVersion.Patch = CORDIC_VERSION_PATCH;

    return ( retVersion );
}


/**
 * \brief Fills the configuration structure with the default values.
 *
 * Default configuration: cosine (and sine) in the q1.31 format, precision 6, scale 0, POLL transfer mode,
 * Gpdma1 channel 0 (arguments) and channel 1 (results) at low priority, interrupt priority 0, no callback.
 *
 * \param cordicConfig [out] : Pointer to store the default configuration. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter or on a device
 *         line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_DefaultConfig( cordic_Config_t * const cordicConfig )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( CORDIC_NULL_PTR != cordicConfig )
    {
        cordicConfig->Function             = CORDIC_FUNC_COSINE;
        cordicConfig->Format               = CORDIC_FORMAT_Q31;
        cordicConfig->Precision            = CORDIC_PRECISION_6;
        cordicConfig->Scale                = CORDIC_SCALE_0;
        cordicConfig->XferMode             = CORDIC_XFER_MODE_POLL;
        cordicConfig->IrqPriority          = CORDIC_DEFAULT_IRQ_PRIO;
        cordicConfig->DmaPeriphId          = GPDMA_PERIPH_1;
        cordicConfig->DmaWriteChannelId    = GPDMA_CHANNEL_0;
        cordicConfig->DmaReadChannelId     = GPDMA_CHANNEL_1;
        cordicConfig->DmaWritePriority     = GPDMA_PRIORITY_LOW;
        cordicConfig->DmaReadPriority      = GPDMA_PRIORITY_LOW;
        cordicConfig->XferCompleteCallback = CORDIC_NULL_PTR;
        cordicConfig->XferHalfCallback     = CORDIC_NULL_PTR;
        cordicConfig->XferErrorCallback    = CORDIC_NULL_PTR;
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)cordicConfig;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Initializes the Cordic module.
 *
 * Enables the peripheral clock (Rcc), resets the peripheral (Rcc reset pulse) and writes the function,
 * the precision, the scale and the format to CORDIC_CSR; the register is read back and on a difference
 * the module is deinitialized and an error is returned. In the ISR transfer mode the CORDIC interrupt
 * (handler, priority, enable) is configured in Nvic; the DMA transfer mode only stores the Gpdma channels,
 * they are configured by every array request.
 *
 * \pre   The configuration shall not be NULL, all its items shall be in their lists, the scale shall
 *        be valid for the function and the module shall not be initialized; otherwise CORDIC_REQUEST_ERROR
 *        and nothing changes.
 *
 * \param cordicConfig [in] : Pointer to the configuration, see \ref cordic_Config_t. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK when the module is initialized, CORDIC_REQUEST_ERROR otherwise (also on a
 *         device line without the CORDIC peripheral and when the module is already initialized).
 */
cordic_RequestState_t Cordic_Init( const cordic_Config_t * const cordicConfig )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( CORDIC_NULL_PTR != cordicConfig )
    {
        retState = Cordic_Check_Config( cordicConfig );
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = CORDIC_REQUEST_ERROR;
    }

    if( CORDIC_REQUEST_OK == retState )
    {
        const cordic_Settings_t settings = { .Function  = cordicConfig->Function,
                                             .Format    = cordicConfig->Format,
                                             .Precision = cordicConfig->Precision,
                                             .Scale     = cordicConfig->Scale     };

        cordic_Runtime.XferMode                                = cordicConfig->XferMode;
        cordic_Runtime.IrqPriority                             = cordicConfig->IrqPriority;
        cordic_Runtime.DmaPeriphId                             = cordicConfig->DmaPeriphId;
        cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_WRITE ]    = cordicConfig->DmaWriteChannelId;
        cordic_Runtime.DmaChannelId[ CORDIC_DMA_DIR_READ ]     = cordicConfig->DmaReadChannelId;
        cordic_Runtime.DmaPriority[ CORDIC_DMA_DIR_WRITE ]     = cordicConfig->DmaWritePriority;
        cordic_Runtime.DmaPriority[ CORDIC_DMA_DIR_READ ]      = cordicConfig->DmaReadPriority;
        cordic_Runtime.DmaConfigured                           = CORDIC_FUNCTION_INACTIVE;
        cordic_Runtime.XferCompleteCallback                    = cordicConfig->XferCompleteCallback;
        cordic_Runtime.XferHalfCallback                        = cordicConfig->XferHalfCallback;
        cordic_Runtime.XferErrorCallback                       = cordicConfig->XferErrorCallback;
        cordic_Runtime.XferError                               = CORDIC_XFER_ERROR_NONE;

        retState = Cordic_Start_Peripheral( &settings );

        if( ( CORDIC_REQUEST_OK     == retState                ) &&
            ( CORDIC_XFER_MODE_ISR  == cordicConfig->XferMode  )    )
        {
            retState = Cordic_Start_Irq();
        }
        else
        {
            /* The peripheral could not be started or the interrupt is not used */
        }

        if( CORDIC_REQUEST_OK == retState )
        {
            cordic_Runtime.State     = CORDIC_STATE_IDLE;
            cordic_Runtime.InitState = CORDIC_FUNCTION_ACTIVE;
        }
        else
        {
            /* Initialization failed, the module is deinitialized */
            (void)Cordic_Release_Peripheral();
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* Invalid configuration, nothing is configured */
    }
#else
    (void)cordicConfig;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Deinitializes the Cordic module: a running array calculation is stopped without a callback, the
 *        Gpdma channels and the interrupt are released, the peripheral is reset and its clock disabled.
 *
 * \return CORDIC_REQUEST_OK when the module was deinitialized, CORDIC_REQUEST_ERROR when it was not
 *         initialized, when a step failed and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Deinit( void )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( CORDIC_REQUEST_OK == retState )
    {
        retState = Cordic_Release_Peripheral();
    }
    else
    {
        /* The module is not initialized, nothing to release */
    }
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Main task of the Cordic module.
 *
 * Serves a running array calculation in the POLL transfer mode: a few calculations per call (write of the
 * arguments, bounded wait for the result, read of the results); the end, the half and an error are reported
 * by the callbacks. The ISR and the DMA transfer modes and the single calculation need no periodic work.
 */
void Cordic_Task( void )
{
#if defined(CORDIC)
    const cordic_State_t moduleState = cordic_Runtime.State;

    if( ( CORDIC_FUNCTION_ACTIVE   == cordic_Runtime.InitState ) &&
        ( CORDIC_STATE_BUSY        == moduleState              ) &&
        ( CORDIC_XFER_MODE_POLL    == cordic_Runtime.XferMode  )    )
    {
        Cordic_Array_Poll();
    }
    else
    {
        /* No array calculation runs in the POLL mode */
    }
#endif /* CORDIC */

    return;
}


/**
 * \brief Starts a single calculation and returns its results (the zero overhead blocking mode).
 *
 * The module state, the function / scale combination and the arguments are checked before a register
 * is written. The arguments and the results are q1.31 numbers in \ref CORDIC_FORMAT_Q31 and q1.15
 * numbers sign extended to 32 bits in \ref CORDIC_FORMAT_Q15. A result that is not ready in time resets
 * the peripheral and returns an error.
 *
 * \param arguments [in]  : Arguments, see \ref cordic_Arguments_t. Must not be NULL.
 * \param results   [out] : Results, see \ref cordic_Results_t. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK when the results are valid, CORDIC_REQUEST_ERROR otherwise (not initialized,
 *         array calculation running, invalid scale, argument out of range, timeout, no CORDIC).
 */
cordic_RequestState_t Cordic_Set_Calculate( const cordic_Arguments_t * const arguments,
                                            cordic_Results_t * const results )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( ( CORDIC_NULL_PTR != arguments ) &&
        ( CORDIC_NULL_PTR != results   )    )
    {
        retState = Cordic_Run_Calculation( arguments, results );
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)arguments;
    (void)results;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Starts the calculation of an array (see \ref cordic_ArrayRequest_t).
 *
 * The request is checked (module state, function / scale combination, arrays, count, the Gpdma block size and the
 * word alignment in the DMA mode) before a register is written. The calculation runs in the transfer mode of the
 * configuration, the module is busy (the configuration is locked) until the end callback, the error callback or
 * \ref Cordic_Deinit. The arrays shall stay valid and unchanged until then.
 *
 * \param request [in] : Array request, see \ref cordic_ArrayRequest_t. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK when the calculation runs, CORDIC_REQUEST_ERROR for an invalid request, when the
 *         module is not initialized, while an array calculation is running, when a Gpdma channel cannot be
 *         configured and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Set_CalculateArray( const cordic_ArrayRequest_t * const request )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( CORDIC_NULL_PTR != request )
    {
        retState = Cordic_Run_Array( request );
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)request;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Gets the state of the array calculation.
 *
 * \param state [out] : State of the array calculation. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, when the module is
 *         not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_State( cordic_State_t * const state )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_NULL_PTR   != state    )    )
    {
        *state = cordic_Runtime.State;
    }
    else
    {
        /* Invalid parameter or the module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)state;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Sets the function of the calculation.
 *
 * The combination with the scale is checked when a calculation is started.
 *
 * \param function [in] : Function, value from \ref cordic_Function_t.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid function, when the module is
 *         not initialized, while an array calculation is running and on a line without the CORDIC.
 */
cordic_RequestState_t Cordic_Set_Function( cordic_Function_t function )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Available();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_FUNC_CNT   >  function )    )
    {
        cordic_Settings_t settings = cordic_Runtime.Settings;

        settings.Function = function;
        retState = Cordic_Set_Settings( &settings );
    }
    else
    {
        /* Invalid function or the configuration cannot be changed now */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)function;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Gets the function of the calculation.
 *
 * \param function [out] : Function. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, when the module is
 *         not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_Function( cordic_Function_t * const function )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_NULL_PTR   != function )    )
    {
        *function = cordic_Runtime.Settings.Function;
    }
    else
    {
        /* Invalid parameter or the module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)function;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Sets the precision of the calculation (the number of iterations divided by 4).
 *
 * \param precision [in] : Precision, value from \ref cordic_Precision_t.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid precision, when the module is
 *         not initialized, while an array calculation is running and on a line without the CORDIC.
 */
cordic_RequestState_t Cordic_Set_Precision( cordic_Precision_t precision )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Available();

    if( ( CORDIC_REQUEST_OK    == retState ) &&
        ( CORDIC_PRECISION_1   <= precision ) &&
        ( CORDIC_PRECISION_CNT >  precision )    )
    {
        cordic_Settings_t settings = cordic_Runtime.Settings;

        settings.Precision = precision;
        retState = Cordic_Set_Settings( &settings );
    }
    else
    {
        /* Invalid precision or the configuration cannot be changed now */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)precision;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Gets the precision of the calculation.
 *
 * \param precision [out] : Precision. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, when the module is
 *         not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_Precision( cordic_Precision_t * const precision )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_NULL_PTR   != precision )    )
    {
        *precision = cordic_Runtime.Settings.Precision;
    }
    else
    {
        /* Invalid parameter or the module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)precision;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Sets the scaling exponent of the calculation.
 *
 * The combination with the function is checked when a calculation is started.
 *
 * \param scale [in] : Scaling exponent, value from \ref cordic_Scale_t.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid scale, when the module is
 *         not initialized, while an array calculation is running and on a line without the CORDIC.
 */
cordic_RequestState_t Cordic_Set_Scale( cordic_Scale_t scale )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Available();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_SCALE_CNT  >  scale    )    )
    {
        cordic_Settings_t settings = cordic_Runtime.Settings;

        settings.Scale = scale;
        retState = Cordic_Set_Settings( &settings );
    }
    else
    {
        /* Invalid scale or the configuration cannot be changed now */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)scale;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Gets the scaling exponent of the calculation.
 *
 * \param scale [out] : Scaling exponent. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, when the module is
 *         not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_Scale( cordic_Scale_t * const scale )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_NULL_PTR   != scale    )    )
    {
        *scale = cordic_Runtime.Settings.Scale;
    }
    else
    {
        /* Invalid parameter or the module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)scale;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Sets the format of the arguments and the results.
 *
 * \param format [in] : Format, value from \ref cordic_Format_t.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid format, when the module is
 *         not initialized, while an array calculation is running and on a line without the CORDIC.
 */
cordic_RequestState_t Cordic_Set_Format( cordic_Format_t format )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Available();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_FORMAT_CNT >  format   )    )
    {
        cordic_Settings_t settings = cordic_Runtime.Settings;

        settings.Format = format;
        retState = Cordic_Set_Settings( &settings );
    }
    else
    {
        /* Invalid format or the configuration cannot be changed now */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)format;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Gets the format of the arguments and the results.
 *
 * \param format [out] : Format. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, when the module is
 *         not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_Format( cordic_Format_t * const format )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_NULL_PTR   != format   )    )
    {
        *format = cordic_Runtime.Settings.Format;
    }
    else
    {
        /* Invalid parameter or the module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)format;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Converts a float number to the q1.31 format with the range check.
 *
 * The number shall be in [-1, 1); a number outside of the range (also NaN) is reported as an error
 * instead of being wrapped. The result is rounded to the nearest value (half away from zero). The scale
 * is not applied: the caller multiplies the number by 2^-n before the conversion when a scale is used.
 *
 * \param value  [in]  : Number to convert.
 * \param qValue [out] : Number in the q1.31 format. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, a number outside
 *         of the range and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_FromFloat( float value, cordic_Q31_t * const qValue )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( ( CORDIC_NULL_PTR        != qValue ) &&
        ( CORDIC_FLOAT_MINUS_ONE <= value  ) &&
        ( CORDIC_FLOAT_ONE       >  value  )    )
    {
        const float scaled = ( value * CORDIC_Q31_SCALE_FLOAT );

        if( CORDIC_FLOAT_ZERO <= scaled )
        {
            *qValue = (cordic_Q31_t)( scaled + CORDIC_FLOAT_HALF );
        }
        else
        {
            *qValue = (cordic_Q31_t)( scaled - CORDIC_FLOAT_HALF );
        }

        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or the number is outside of [-1, 1) */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)value;
    (void)qValue;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Converts a q1.31 number to float.
 *
 * Every q1.31 value is valid. A float has 24 significant bits, so the values within 2^-24 of 1.0 give 1.0.
 *
 * \param qValue [in]  : Number in the q1.31 format.
 * \param value  [out] : Converted number in [-1, 1]. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter and on a device
 *         line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_ToFloat( cordic_Q31_t qValue, float * const value )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( CORDIC_NULL_PTR != value )
    {
        *value   = ( (float)qValue / CORDIC_Q31_SCALE_FLOAT );
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)qValue;
    (void)value;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Converts a float number to the q1.15 format with the range check.
 *
 * The number shall be in [-1, 1); a number outside of the range (also NaN) is reported as an error
 * instead of being wrapped. The result is rounded to the nearest value (half away from zero), the numbers
 * above 1.0 - 2^-16 give the largest q1.15 value. The scale is not applied by the function.
 *
 * \param value  [in]  : Number to convert.
 * \param qValue [out] : Number in the q1.15 format. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, a number outside
 *         of the range and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_FromFloatQ15( float value, cordic_Q15_t * const qValue )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( ( CORDIC_NULL_PTR        != qValue ) &&
        ( CORDIC_FLOAT_MINUS_ONE <= value  ) &&
        ( CORDIC_FLOAT_ONE       >  value  )    )
    {
        const float scaled  = ( value * CORDIC_Q15_SCALE_FLOAT );
        int32_t     rounded = 0;

        if( CORDIC_FLOAT_ZERO <= scaled )
        {
            rounded = (int32_t)( scaled + CORDIC_FLOAT_HALF );
        }
        else
        {
            rounded = (int32_t)( scaled - CORDIC_FLOAT_HALF );
        }

        if( CORDIC_Q15_ONE < rounded )
        {
            rounded = CORDIC_Q15_ONE;
        }
        else
        {
            /* The rounded number is inside of the q1.15 range */
        }

        *qValue  = (cordic_Q15_t)rounded;
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter or the number is outside of [-1, 1) */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)value;
    (void)qValue;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Converts a q1.15 number to float.
 *
 * \param qValue [in]  : Number in the q1.15 format.
 * \param value  [out] : Converted number in [-1, 1). Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter and on a device
 *         line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_ToFloatQ15( cordic_Q15_t qValue, float * const value )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    if( CORDIC_NULL_PTR != value )
    {
        *value   = ( (float)qValue / CORDIC_Q15_SCALE_FLOAT );
        retState = CORDIC_REQUEST_OK;
    }
    else
    {
        /* Invalid parameter, no action required */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)qValue;
    (void)value;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Sets the interrupt priority of the CORDIC interrupt.
 *
 * \param irqPrio [in] : Interrupt priority, forwarded to Nvic.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR when Nvic rejects the priority, when the module
 *         is not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Set_IrqPriority( cordic_IrqPrio_t irqPrio )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( CORDIC_REQUEST_OK == retState )
    {
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Prio( NVIC_PERIPH_IRQ_CORDIC, irqPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            cordic_Runtime.IrqPriority = irqPrio;
        }
        else
        {
            /* The priority was rejected, the stored one stays */
            retState = CORDIC_REQUEST_ERROR;
        }
    }
    else
    {
        /* The module is not initialized */
    }
#else
    (void)irqPrio;
#endif /* CORDIC */

    return ( retState );
}


/**
 * \brief Gets the interrupt priority of the CORDIC interrupt.
 *
 * \param irqPrio [out] : Interrupt priority. Must not be NULL.
 *
 * \return CORDIC_REQUEST_OK on success, CORDIC_REQUEST_ERROR for an invalid parameter, when the module is
 *         not initialized and on a device line without the CORDIC peripheral.
 */
cordic_RequestState_t Cordic_Get_IrqPriority( cordic_IrqPrio_t * const irqPrio )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

#if defined(CORDIC)
    retState = Cordic_Check_Initialized();

    if( ( CORDIC_REQUEST_OK == retState ) &&
        ( CORDIC_NULL_PTR   != irqPrio  )    )
    {
        *irqPrio = cordic_Runtime.IrqPriority;
    }
    else
    {
        /* Invalid parameter or the module is not initialized */
        retState = CORDIC_REQUEST_ERROR;
    }
#else
    (void)irqPrio;
#endif /* CORDIC */

    return ( retState );
}
