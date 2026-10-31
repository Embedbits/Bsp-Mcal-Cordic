/**
 * \author Mr.Nobody
 * \file Test_Cordic.c
 * \ingroup Cordic
 * \brief Unit tests of the CORDIC coprocessor (Cordic) module.
 *
 * Cordic.c and Cordic_Reg.c are compiled unchanged. The CORDIC registers are emulated by RegMem,
 * RCC, NVIC and Gpdma are mocked by CMock (strict order of the calls).
 *
 * \note Limits of the register emulation (RegMem registers are plain memory, the coprocessor does
 *       not calculate): the tests that run a calculation activate the HW model UtCordic_HwModel.
 *       It detects a change of CORDIC_WDATA (the tests write distinct, non-zero words), counts the
 *       words of a calculation (CORDIC_CSR.NARGS / ARGSIZE), then sets CORDIC_CSR.RRDY and stores a
 *       made-up result in CORDIC_RDATA (first argument word + offset in the 32-bit format, both
 *       half-words incremented in the 16-bit format). The mathematics of the functions is the
 *       hardware job and is not tested here. A read of CORDIC_RDATA is not trapped by RegMem and does
 *       not release the next result word: in the 32-bit format a second result is the same word as
 *       the first one, so the tests check that both results are taken from CORDIC_RDATA but not their
 *       order. The ranges, the word counts, the register encoding, the state machine, the callbacks
 *       and the Gpdma configuration are checked exactly.
 *
 * \note The test runner is generated from this file without running the preprocessor, so every test
 *       case is defined unconditionally. STM32H503, STM32H523 and STM32H533 have no CORDIC: the test
 *       cases ignore themselves there, only \ref Ut_Cordic_NoPeripheral_EveryFunctionReturnsError and
 *       the version test run on those lines.
 *
 * \note Errata sheets: the sheets available for the STM32H5 lines (ES0565, ES0621) contain no CORDIC
 *       entry; the sheet of the STM32H5E / H5F lines was not available and is unverified.
 */

/* ============================= INCLUDES =================================== */
#include <math.h>                           /* NAN, INFINITY                  */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Cordic_Port.h"                    /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpdma_Port.h"                 /* Gpdma module mock              */
#include "Stm32.h"                          /* CORDIC registers definition    */
/* ============================= TYPEDEFS =================================== */

#if defined(CORDIC)

/** Function case of the table driven tests: function, scale, valid arguments, expected word counts */
typedef struct
{
    cordic_Function_t Function; /**< Function under test                        */
    cordic_Scale_t    Scale;    /**< Scale valid for the function               */
    cordic_Q31_t      Value1;   /**< Valid primary argument                     */
    cordic_Q31_t      Value2;   /**< Valid secondary argument                   */
    uint32_t          ArgWords; /**< Expected number of the written data words  */
    uint32_t          NresBit;  /**< Expected CORDIC_CSR.NRES bit               */
}   utCordic_FuncCase_t;


/** Copy of one Gpdma_Init() call */
typedef struct
{
    gpdma_ConfigStruct_t   Config; /**< Channel configuration                    */
    gpdma_TransferConfig_t Xfer;   /**< Transfer configuration it points to      */
}   utCordic_DmaCapture_t;

#endif /* CORDIC */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Size of the log of the data words written by the code under test */
#define UT_CORDIC_LOG_SIZE                  ( 64u )

/** HW model: the 32-bit result is the first argument word plus this offset */
#define UT_CORDIC_MODEL_OFFSET              ( 1000 )

/** HW model: the 16-bit result is the written word plus 1 in the lower and 2 in the upper half-word */
#define UT_CORDIC_MODEL_Q15_ADD             ( 0x00020001u )

/** Length of the input and output arrays of the array tests */
#define UT_CORDIC_ARRAY_LEN                 ( 64u )

/** Length of the big arrays of the DMA block size limit test (8191 calculations of two words) */
#define UT_CORDIC_BIG_LEN                   ( 16384u )

/** Fill value of the output arrays */
#define UT_CORDIC_OUT_FILL                  ( 0x55555555 )

/** First input word of the array tests and the step between the words */
#define UT_CORDIC_IN_BASE                   ( 0x00100000 )
#define UT_CORDIC_IN_STEP                   ( 0x00001001 )

/** Interrupt priorities used by the tests */
#define UT_CORDIC_IRQ_PRIO                  ( 5u )
#define UT_CORDIC_IRQ_PRIO_OTHER            ( 9u )

/** Gpdma controller and channels of the DMA tests */
#define UT_CORDIC_DMA_PERIPH                ( GPDMA_PERIPH_1 )
#define UT_CORDIC_DMA_WRITE_CH              ( GPDMA_CHANNEL_2 )
#define UT_CORDIC_DMA_READ_CH               ( GPDMA_CHANNEL_3 )

/** Index of the captured Gpdma_Init() call of the write and of the read channel */
#define UT_CORDIC_CAPTURE_WRITE             ( 0u )
#define UT_CORDIC_CAPTURE_READ              ( 1u )

/** Number of the Gpdma_Init() calls that are captured */
#define UT_CORDIC_CAPTURE_CNT               ( 8u )

/** Number of the Gpdma_Init() call that fails (none) */
#define UT_CORDIC_NO_FAIL                   ( 100 )

/** Size of a data word in bytes */
#define UT_CORDIC_WORD_BYTES                ( 4u )

/** Largest block of one Gpdma channel in bytes */
#define UT_CORDIC_DMA_MAX_BYTES             ( 65535u )

/** CORDIC_CSR bits that select the function, the precision, the scale and the data formats */
#define UT_CORDIC_CSR_MASK                  ( CORDIC_CSR_FUNC    | CORDIC_CSR_PRECISION | CORDIC_CSR_SCALE | \
                                              CORDIC_CSR_ARGSIZE | CORDIC_CSR_RESSIZE   | CORDIC_CSR_NARGS | \
                                              CORDIC_CSR_NRES )

/** Expected CORDIC_CSR.Settings: cosine, precision 6, scale 0, 32-bit, two arguments and two results */
#define UT_CORDIC_CSR_COSINE_Q31            ( 0x00180060u )

/** CORDIC_CSR bits that enable the DMA requests of the coprocessor */
#define UT_CORDIC_CSR_DMA_BITS              ( CORDIC_CSR_DMAREN | CORDIC_CSR_DMAWEN )

/** Expected CORDIC_CSR.Settings: arctangent, precision 3, scale 2, 16-bit values */
#define UT_CORDIC_CSR_ARCTAN_Q15            ( 0x00600234u )

/** Expected CORDIC_CSR.Settings: square root, precision 6, scale 0, 32-bit, one argument, one result */
#define UT_CORDIC_CSR_SQRT_Q31              ( 0x00000069u )

/** Expected CORDIC_CSR.Settings: hyperbolic sine, precision 6, scale 0, 32-bit, one argument, two results */
#define UT_CORDIC_CSR_SINH_Q31              ( 0x00080066u )

/** Expected CORDIC_CSR.Settings: square root, precision 6, scale 0, 16-bit values */
#define UT_CORDIC_CSR_SQRT_Q15              ( 0x00600069u )

/** Expected CORDIC_CSR.Settings: precision 9 of the square root, scale 0, 32-bit */
#define UT_CORDIC_CSR_SQRT_PREC9            ( 0x00000099u )

/** Expected CORDIC_CSR.Settings: square root, precision 6, scale 3, 32-bit */
#define UT_CORDIC_CSR_SQRT_SCALE3           ( 0x00000369u )

/** Precision and scale values used by the tests */
#define UT_CORDIC_PRECISION_NINE            ( CORDIC_PRECISION_9 )

/** q1.31 values of the argument limits of the reference manual (parts per million of one, rounded down) */
#define UT_CORDIC_Q31_0P559                 ( 1200443359 )
#define UT_CORDIC_Q31_0P403                 ( 865435910 )
#define UT_CORDIC_Q31_0P0535                ( 114890375 )
#define UT_CORDIC_Q31_0P5                   ( 1073741824 )
#define UT_CORDIC_Q31_0P25                  ( 536870912 )
#define UT_CORDIC_Q31_0P75                  ( 1610612736 )
#define UT_CORDIC_Q31_0P375                 ( 805306368 )
#define UT_CORDIC_Q31_0P875                 ( 1879048192 )
#define UT_CORDIC_Q31_0P4375                ( 939524096 )
#define UT_CORDIC_Q31_0P584                 ( 1254130450 )
#define UT_CORDIC_Q31_0P027                 ( 57982058 )
#define UT_CORDIC_Q31_0P585                 ( 1256277934 )

/** Distinct valid arguments of the single calculations (q1.31) */
#define UT_CORDIC_ARG_A                     ( 0x12345678 )
#define UT_CORDIC_ARG_B                     ( 0x23456789 )
#define UT_CORDIC_ARG_C                     ( 0x0ABCDEF1 )
#define UT_CORDIC_ARG_D                     ( 0x0BCDEF12 )
#define UT_CORDIC_ARG_NEGATIVE              ( -0x0FFFFFFD )

/** Valid arguments of the function table test, every word differs from the others */
#define UT_CORDIC_FUNC_ARG_COSINE           ( 0x10000001 )
#define UT_CORDIC_FUNC_ARG_COSINE_2         ( 0x40000001 )
#define UT_CORDIC_FUNC_ARG_SINE             ( 0x10000002 )
#define UT_CORDIC_FUNC_ARG_SINE_2           ( 0x40000002 )
#define UT_CORDIC_FUNC_ARG_PHASE            ( 0x10000003 )
#define UT_CORDIC_FUNC_ARG_PHASE_2          ( -0x0FFFFFFD )
#define UT_CORDIC_FUNC_ARG_MODULUS          ( 0x10000004 )
#define UT_CORDIC_FUNC_ARG_MODULUS_2        ( 0x30000004 )
#define UT_CORDIC_FUNC_ARG_ARCTAN           ( 0x10000005 )
#define UT_CORDIC_FUNC_ARG_COSH             ( 0x10000006 )
#define UT_CORDIC_FUNC_ARG_SINH             ( 0x10000007 )
#define UT_CORDIC_FUNC_ARG_ATANH            ( 0x10000008 )
#define UT_CORDIC_FUNC_ARG_LN               ( 0x10000009 )
#define UT_CORDIC_FUNC_ARG_SQRT             ( 0x1000000A )

/** Arguments of the 16-bit format tests (q1.15 sign extended) */
#define UT_CORDIC_Q15_ARG_NEGATIVE          ( -0x1234 )
#define UT_CORDIC_Q15_ARG_MODULUS           ( 0x4000 )
#define UT_CORDIC_Q15_ARG_SQRT              ( 0x1000 )
#define UT_CORDIC_Q15_ARG_TOO_LARGE         ( 0x8000 )
#define UT_CORDIC_Q15_ARG_TOO_SMALL         ( -0x8001 )

/** Expected results of the 16-bit tests: the HW model adds 1 / 2 to the half-words */
#define UT_CORDIC_Q15_WORD_COSINE           ( 0x4000EDCCu )
#define UT_CORDIC_Q15_RES1_COSINE           ( -0x1233 )
#define UT_CORDIC_Q15_RES2_COSINE           ( 0x4002 )

/** Raw data word of the packed 16-bit result of the unpack test */
#define UT_CORDIC_Q15_WORD_RESULT           ( 0x8001FFFEu )
#define UT_CORDIC_Q15_RESULT_FIRST          ( -2 )
#define UT_CORDIC_Q15_RESULT_SECOND         ( -32767 )

/** Float values of the conversion tests */
#define UT_CORDIC_FLOAT_ZERO                ( 0.0f )
#define UT_CORDIC_FLOAT_HALF                ( 0.5f )
#define UT_CORDIC_FLOAT_QUARTER             ( 0.25f )
#define UT_CORDIC_FLOAT_MINUS_HALF          ( -0.5f )
#define UT_CORDIC_FLOAT_ONE                 ( 1.0f )
#define UT_CORDIC_FLOAT_MINUS_ONE           ( -1.0f )
#define UT_CORDIC_FLOAT_TWO                 ( 2.0f )
#define UT_CORDIC_FLOAT_BELOW_ONE           ( 0.99999994f )
#define UT_CORDIC_FLOAT_BELOW_MINUS_ONE     ( -1.0000001f )
#define UT_CORDIC_FLOAT_TINY                ( 2.3283064365386963e-10f )
#define UT_CORDIC_FLOAT_TINY_HALF           ( 1.1641532182693481e-10f )
#define UT_CORDIC_FLOAT_Q15_HALF_LSB        ( 1.52587890625e-05f )
#define UT_CORDIC_FLOAT_Q15_QUARTER_LSB     ( 7.62939453125e-06f )
#define UT_CORDIC_FLOAT_Q15_NEAR_ONE        ( 0.99998f )
#define UT_CORDIC_FLOAT_Q15_NINE            ( 0.9999f )
#define UT_CORDIC_FLOAT_ROUND_A             ( 0.3f )
#define UT_CORDIC_FLOAT_ROUND_B             ( -0.7f )
#define UT_CORDIC_FLOAT_TOLERANCE           ( 5.0e-10f )
#define UT_CORDIC_FLOAT_TOLERANCE_Q15       ( 4.0e-05f )

/** Expected q1.31 / q1.15 values of the conversion tests */
#define UT_CORDIC_QV_HALF                   ( 0x40000000 )
#define UT_CORDIC_QV_MINUS_HALF             ( -0x40000000 )
#define UT_CORDIC_QV_QUARTER                ( 0x20000000 )
#define UT_CORDIC_QV_BELOW_ONE              ( 0x7FFFFF80 )
#define UT_CORDIC_QV_Q15_HALF               ( 0x4000 )
#define UT_CORDIC_QV_Q15_QUARTER            ( 0x2000 )
#define UT_CORDIC_QV_Q15_NINE               ( 32765 )
#define UT_CORDIC_QV_Q15_LAST               ( 32767 )

/** Number of the Gpdma channels of a DMA request, offsets that make a word array unaligned, array indices */
#define UT_CORDIC_CHANNEL_CNT               ( 2u )
#define UT_CORDIC_MISALIGN_HALF             ( 2u )
#define UT_CORDIC_IDX_SECOND                ( 2u )
#define UT_CORDIC_IDX_AFTER                 ( 4u )

/** Number of data words of a calculation and number of the calculations of the array tests */
#define UT_CORDIC_WORDS_TWO                 ( 2u )
#define UT_CORDIC_COUNT_FOUR                ( 4u )
#define UT_CORDIC_COUNT_THREE               ( 3u )
#define UT_CORDIC_COUNT_TWO                 ( 2u )
#define UT_CORDIC_COUNT_FIVE                ( 5u )
#define UT_CORDIC_COUNT_TWENTY              ( 20u )
#define UT_CORDIC_COUNT_TASK                ( 8u )
#define UT_CORDIC_COUNT_TWICE_TASK          ( 16u )
#define UT_CORDIC_COUNT_ZERO                ( 0u )
#define UT_CORDIC_COUNT_TOO_LARGE           ( 0x80000000u )
#define UT_CORDIC_COUNT_BLOCK_LIMIT         ( 8191u )
#define UT_CORDIC_COUNT_BLOCK_OVER          ( 8192u )

/** Interrupt priority getters of an uninitialized module and the timeouts */
#define UT_CORDIC_DUMMY_PRIO                ( 123u )

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(CORDIC)

static cordic_Config_t       UtCordic_Get_Config          ( cordic_Function_t function,
                                                            cordic_Format_t format,
                                                            cordic_Scale_t scale,
                                                            cordic_XferMode_t xferMode );
static cordic_Config_t       UtCordic_Get_DmaConfig       ( cordic_Function_t function,
                                                            cordic_Format_t format,
                                                            cordic_Scale_t scale,
                                                            cordic_XferHalfCallback_t * halfCallback );
static void                  UtCordic_Expect_Start        ( void );
static void                  UtCordic_Expect_Reset        ( void );
static void                  UtCordic_Expect_Release      ( void );
static void                  UtCordic_Expect_Irq          ( void );
static void                  UtCordic_Expect_DmaChannel   ( gpdma_ChannelId_t channel );
static void                  UtCordic_Expect_DmaStart     ( void );
static void                  UtCordic_Expect_DmaRelease   ( void );
static void                  UtCordic_Init_Poll           ( cordic_Function_t function,
                                                            cordic_Format_t format,
                                                            cordic_Scale_t scale );
static void                  UtCordic_Init_Callbacks      ( cordic_Config_t * const config );
static void                  UtCordic_Init_Dma            ( cordic_Function_t function,
                                                            cordic_Format_t format,
                                                            cordic_Scale_t scale,
                                                            cordic_XferHalfCallback_t * halfCallback );
static void                  UtCordic_Select              ( cordic_Function_t function, cordic_Scale_t scale );
static void                  UtCordic_Fill_Input          ( cordic_Count_t count );
static void                  UtCordic_Fill_Output         ( void );
static cordic_ArrayRequest_t UtCordic_Get_Request         ( cordic_Count_t count );
static cordic_RequestState_t UtCordic_Calculate           ( cordic_Q31_t value1, cordic_Q31_t value2 );
static void                  UtCordic_Check_Accepted      ( cordic_Q31_t value1, cordic_Q31_t value2 );
static void                  UtCordic_Check_Rejected      ( cordic_Q31_t value1, cordic_Q31_t value2 );
static void                  UtCordic_Model_Start         ( void );
static void                  UtCordic_HwModel             ( void );
static void                  UtCordic_HwModel_ClearPrec   ( void );
static void                  UtCordic_CompleteCallback    ( void );
static void                  UtCordic_HalfCallback        ( void );
static void                  UtCordic_ErrorCallback       ( cordic_XferError_t xferError );
static nvic_RequestState_t   UtCordic_NvicHandlerStub     ( nvic_PeriphIrqList_t irqId,
                                                            const nvic_IsrCallback_t irqHandler,
                                                            int callCnt );
static gpdma_RequestState_t  UtCordic_GpdmaInitStub       ( gpdma_ConfigStruct_t * const configStruct, int callCnt );

#endif /* CORDIC */

/* ========================== LOCAL VARIABLES =============================== */

#if defined(CORDIC)

/** HW model: last seen CORDIC_WDATA, words of the running calculation, first word, log of the written words */
static uint32_t utCordic_ModelLast;
static uint32_t utCordic_ModelPending;
static uint32_t utCordic_ModelFirst;
static uint32_t utCordic_ModelLog[ UT_CORDIC_LOG_SIZE ];
static uint32_t utCordic_ModelLogCnt;

/** Callback records: counts, order of the half and of the complete callback, last error mask */
static uint32_t           utCordic_CompleteCnt;
static uint32_t           utCordic_HalfCnt;
static uint32_t           utCordic_ErrorCnt;
static uint32_t           utCordic_Order;
static uint32_t           utCordic_CompleteOrder;
static uint32_t           utCordic_HalfOrder;
static cordic_XferError_t utCordic_ErrorMask;

/** CORDIC interrupt handler registered in Nvic by the code under test */
static nvic_IsrCallback_t utCordic_Isr;

/** Captured Gpdma_Init() calls and the number of the call that fails */
static utCordic_DmaCapture_t utCordic_DmaCapture[ UT_CORDIC_CAPTURE_CNT ];
static uint32_t              utCordic_DmaCaptureCnt;
static int                   utCordic_GpdmaFailCall;

/** Arrays of the array tests */
static cordic_Q31_t utCordic_In[ UT_CORDIC_ARRAY_LEN ];
static cordic_Q31_t utCordic_Out[ UT_CORDIC_ARRAY_LEN ];
static cordic_Q31_t utCordic_BigIn[ UT_CORDIC_BIG_LEN ];
static cordic_Q31_t utCordic_BigOut[ UT_CORDIC_BIG_LEN ];

/** Table of the functions with valid arguments and the expected number of the written words */
static const utCordic_FuncCase_t utCordic_FuncCase[] =
{
    { CORDIC_FUNC_COSINE,
      CORDIC_SCALE_0, UT_CORDIC_FUNC_ARG_COSINE, UT_CORDIC_FUNC_ARG_COSINE_2, UT_CORDIC_WORDS_TWO, CORDIC_CSR_NRES },
    { CORDIC_FUNC_SINE,
      CORDIC_SCALE_0, UT_CORDIC_FUNC_ARG_SINE, UT_CORDIC_FUNC_ARG_SINE_2, UT_CORDIC_WORDS_TWO, CORDIC_CSR_NRES },
    { CORDIC_FUNC_PHASE,
      CORDIC_SCALE_0, UT_CORDIC_FUNC_ARG_PHASE, UT_CORDIC_FUNC_ARG_PHASE_2, UT_CORDIC_WORDS_TWO, CORDIC_CSR_NRES },
    { CORDIC_FUNC_MODULUS,
      CORDIC_SCALE_0, UT_CORDIC_FUNC_ARG_MODULUS, UT_CORDIC_FUNC_ARG_MODULUS_2, UT_CORDIC_WORDS_TWO, CORDIC_CSR_NRES },
    { CORDIC_FUNC_ARCTANGENT,
      CORDIC_SCALE_3, UT_CORDIC_FUNC_ARG_ARCTAN, 0, 1u, 0u },
    { CORDIC_FUNC_HYPERBOLIC_COSINE,
      CORDIC_SCALE_1, UT_CORDIC_FUNC_ARG_COSH, 0, 1u, CORDIC_CSR_NRES },
    { CORDIC_FUNC_HYPERBOLIC_SINE,
      CORDIC_SCALE_1, UT_CORDIC_FUNC_ARG_SINH, 0, 1u, CORDIC_CSR_NRES },
    { CORDIC_FUNC_ARCTANGENT_HYPERBOLIC,
      CORDIC_SCALE_1, UT_CORDIC_FUNC_ARG_ATANH, 0, 1u, 0u },
    { CORDIC_FUNC_NATURAL_LOGARITHM,
      CORDIC_SCALE_1, UT_CORDIC_FUNC_ARG_LN, 0, 1u, 0u },
    { CORDIC_FUNC_SQUARE_ROOT,
      CORDIC_SCALE_0, UT_CORDIC_FUNC_ARG_SQRT, 0, 1u, 0u },
};

#endif /* CORDIC */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );
#if defined(CORDIC)
    utCordic_ModelLast     = 0u;
    utCordic_ModelPending  = 0u;
    utCordic_ModelFirst    = 0u;
    utCordic_ModelLogCnt   = 0u;
    utCordic_CompleteCnt   = 0u;
    utCordic_HalfCnt       = 0u;
    utCordic_ErrorCnt      = 0u;
    utCordic_Order         = 0u;
    utCordic_CompleteOrder = 0u;
    utCordic_HalfOrder     = 0u;
    utCordic_ErrorMask     = CORDIC_XFER_ERROR_NONE;
    utCordic_Isr           = NULL;
    utCordic_DmaCaptureCnt = 0u;
    utCordic_GpdmaFailCall = UT_CORDIC_NO_FAIL;
#endif /* CORDIC */
}


void tearDown( void )
{
    ( void )RegMem_Set_ModelInactive();
}

/* ============================ HELPER FUNCTIONS ============================= */

#if defined(CORDIC)

/** \brief Default configuration with the function, the format, the scale and the transfer mode. */
static cordic_Config_t UtCordic_Get_Config( cordic_Function_t function,
                                            cordic_Format_t format,
                                            cordic_Scale_t scale,
                                            cordic_XferMode_t xferMode )
{
    cordic_Config_t config;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_DefaultConfig( &config ) );

    config.Function    = function;
    config.Format      = format;
    config.Scale       = scale;
    config.XferMode    = xferMode;
    config.IrqPriority = UT_CORDIC_IRQ_PRIO;

    return ( config );
}


/** \brief DMA configuration: Gpdma1, write channel 2, read channel 3, optional half callback. */
static cordic_Config_t UtCordic_Get_DmaConfig( cordic_Function_t function,
                                               cordic_Format_t format,
                                               cordic_Scale_t scale,
                                               cordic_XferHalfCallback_t * halfCallback )
{
    cordic_Config_t config = UtCordic_Get_Config( function, format, scale, CORDIC_XFER_MODE_DMA );

    config.DmaPeriphId          = UT_CORDIC_DMA_PERIPH;
    config.DmaWriteChannelId    = UT_CORDIC_DMA_WRITE_CH;
    config.DmaReadChannelId     = UT_CORDIC_DMA_READ_CH;
    config.DmaWritePriority     = GPDMA_PRIORITY_HIGH;
    config.DmaReadPriority      = GPDMA_PRIORITY_MEDIUM;
    config.XferCompleteCallback = UtCordic_CompleteCallback;
    config.XferHalfCallback     = halfCallback;
    config.XferErrorCallback    = UtCordic_ErrorCallback;

    return ( config );
}


/** \brief Expects the Rcc calls of the peripheral start: clock on, reset pulse. */
static void UtCordic_Expect_Start( void )
{
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_OK );
    UtCordic_Expect_Reset();
}


/** \brief Expects the Rcc reset pulse. */
static void UtCordic_Expect_Reset( void )
{
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_OK );
}


/** \brief Expects the calls of the release of the peripheral without a Gpdma channel. */
static void UtCordic_Expect_Release( void )
{
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, NVIC_REQUEST_OK );
    UtCordic_Expect_Reset();
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_OK );
}


/** \brief Expects the Nvic calls of the ISR transfer mode and records the registered handler. */
static void UtCordic_Expect_Irq( void )
{
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Handler_AddCallback( UtCordic_NvicHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, UT_CORDIC_IRQ_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, NVIC_REQUEST_OK );
}


/** \brief Expects the Gpdma calls that configure one channel: default configuration, init, interrupt. */
static void UtCordic_Expect_DmaChannel( gpdma_ChannelId_t channel )
{
    Gpdma_Get_DefaultConfig_ExpectAnyArgsAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_ExpectAnyArgsAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_AddCallback( UtCordic_GpdmaInitStub );
    Gpdma_Set_InterruptActive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, channel, GPDMA_REQUEST_OK );
}


/** \brief Expects the Gpdma calls of the start of a DMA request: both channels, read channel activated first. */
static void UtCordic_Expect_DmaStart( void )
{
    UtCordic_Expect_DmaChannel( UT_CORDIC_DMA_WRITE_CH );
    UtCordic_Expect_DmaChannel( UT_CORDIC_DMA_READ_CH );
    Gpdma_Set_ChannelActive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_READ_CH, GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelActive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_WRITE_CH, GPDMA_REQUEST_OK );
}


/** \brief Expects the Gpdma calls that release both channels of a previous request. */
static void UtCordic_Expect_DmaRelease( void )
{
    Gpdma_Deinit_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_WRITE_CH, GPDMA_REQUEST_OK );
    Gpdma_Deinit_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_READ_CH, GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptInactive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_WRITE_CH, GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptInactive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_READ_CH, GPDMA_REQUEST_OK );
}


/** \brief Initializes the module in the POLL mode with the function, the format and the scale. */
static void UtCordic_Init_Poll( cordic_Function_t function, cordic_Format_t format, cordic_Scale_t scale )
{
    const cordic_Config_t config = UtCordic_Get_Config( function, format, scale, CORDIC_XFER_MODE_POLL );

    UtCordic_Expect_Start();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
}


/** \brief Sets the three callbacks of a configuration. */
static void UtCordic_Init_Callbacks( cordic_Config_t * const config )
{
    config->XferCompleteCallback = UtCordic_CompleteCallback;
    config->XferHalfCallback     = UtCordic_HalfCallback;
    config->XferErrorCallback    = UtCordic_ErrorCallback;
}


/** \brief Initializes the module in the DMA mode (Gpdma channels are configured by the requests). */
static void UtCordic_Init_Dma( cordic_Function_t function,
                               cordic_Format_t format,
                               cordic_Scale_t scale,
                               cordic_XferHalfCallback_t * halfCallback )
{
    const cordic_Config_t config = UtCordic_Get_DmaConfig( function, format, scale, halfCallback );

    UtCordic_Expect_Start();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
}


/** \brief Selects the function and the scale of the initialized module. */
static void UtCordic_Select( cordic_Function_t function, cordic_Scale_t scale )
{
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Function( function ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( scale ) );
}


/** \brief Fills the input array with distinct non-zero words. */
static void UtCordic_Fill_Input( cordic_Count_t count )
{
    cordic_Count_t limit = count;

    if( UT_CORDIC_ARRAY_LEN < limit )
    {
        limit = UT_CORDIC_ARRAY_LEN;
    }
    else
    {
        /* The count fits the array */
    }

    for( cordic_Count_t idx = 0u; limit > idx; idx++ )
    {
        utCordic_In[ idx ] = ( UT_CORDIC_IN_BASE + ( (cordic_Q31_t)idx * UT_CORDIC_IN_STEP ) );
    }
}


/** \brief Fills the output array with the fill value. */
static void UtCordic_Fill_Output( void )
{
    for( uint32_t idx = 0u; UT_CORDIC_ARRAY_LEN > idx; idx++ )
    {
        utCordic_Out[ idx ] = UT_CORDIC_OUT_FILL;
    }
}


/** \brief Array request over the input and the output array. */
static cordic_ArrayRequest_t UtCordic_Get_Request( cordic_Count_t count )
{
    cordic_ArrayRequest_t request;

    request.Input  = utCordic_In;
    request.Output = utCordic_Out;
    request.Count  = count;

    return ( request );
}


/** \brief Runs a single calculation with the given arguments and returns the request state. */
static cordic_RequestState_t UtCordic_Calculate( cordic_Q31_t value1, cordic_Q31_t value2 )
{
    const cordic_Arguments_t arguments = { .Value1 = value1, .Value2 = value2 };
    cordic_Results_t         results   = { .Value1 = 0, .Value2 = 0 };

    return ( Cordic_Set_Calculate( &arguments, &results ) );
}


/** \brief Checks that a calculation with the arguments is accepted (HW model active). */
static void UtCordic_Check_Accepted( cordic_Q31_t value1, cordic_Q31_t value2 )
{
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, UtCordic_Calculate( value1, value2 ) );
}


/** \brief Checks that a calculation with the arguments is rejected and no word was written. */
static void UtCordic_Check_Rejected( cordic_Q31_t value1, cordic_Q31_t value2 )
{
    const uint32_t logCnt = utCordic_ModelLogCnt;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, UtCordic_Calculate( value1, value2 ) );
    TEST_ASSERT_EQUAL_UINT32( logCnt, utCordic_ModelLogCnt );
}


/** \brief Starts the HW model of the calculation (the written words are logged from now on). */
static void UtCordic_Model_Start( void )
{
    utCordic_ModelLast    = CORDIC->WDATA;
    utCordic_ModelPending = 0u;
    utCordic_ModelFirst   = 0u;
    utCordic_ModelLogCnt  = 0u;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtCordic_HwModel ) );
}


/**
 * \brief HW model: reacts to a new word in CORDIC_WDATA. When the expected number of the words is written it
 *        stores a made-up result in CORDIC_RDATA and sets CORDIC_CSR.RRDY.
 */
static void UtCordic_HwModel( void )
{
    const uint32_t wdata = CORDIC->WDATA;
    const uint32_t csr   = CORDIC->CSR;

    if( wdata != utCordic_ModelLast )
    {
        uint32_t expectedWords = 1u;
        uint32_t result        = 0u;

        utCordic_ModelLast = wdata;

        if( UT_CORDIC_LOG_SIZE > utCordic_ModelLogCnt )
        {
            utCordic_ModelLog[ utCordic_ModelLogCnt ] = wdata;
        }
        else
        {
            /* The log is full, the word is only counted */
        }

        utCordic_ModelLogCnt++;
        utCordic_ModelPending++;

        if( 1u == utCordic_ModelPending )
        {
            utCordic_ModelFirst = wdata;
        }
        else
        {
            /* The first word of the calculation is already known */
        }

        if( ( 0u != ( csr & CORDIC_CSR_NARGS  ) ) &&
            ( 0u == ( csr & CORDIC_CSR_ARGSIZE ) )    )
        {
            expectedWords = UT_CORDIC_WORDS_TWO;
        }
        else
        {
            expectedWords = 1u;
        }

        if( utCordic_ModelPending >= expectedWords )
        {
            utCordic_ModelPending = 0u;

            if( 0u != ( csr & CORDIC_CSR_ARGSIZE ) )
            {
                result = ( wdata + UT_CORDIC_MODEL_Q15_ADD );
            }
            else
            {
                result = ( utCordic_ModelFirst + (uint32_t)UT_CORDIC_MODEL_OFFSET );
            }

            __atomic_store_n( &CORDIC->RDATA, result, __ATOMIC_SEQ_CST );
            ( void )__atomic_or_fetch( &CORDIC->CSR, CORDIC_CSR_RRDY, __ATOMIC_SEQ_CST );
        }
        else
        {
            /* The calculation waits for the next argument word */
        }
    }
    else
    {
        /* No new argument word */
    }
}


/** \brief HW model of a faulty register: clears the CORDIC_CSR.PRECISION field after every store. */
static void UtCordic_HwModel_ClearPrec( void )
{
    ( void )__atomic_and_fetch( &CORDIC->CSR, ~CORDIC_CSR_PRECISION, __ATOMIC_SEQ_CST );
}


/** \brief Complete callback of the tests: counts the calls and records their order. */
static void UtCordic_CompleteCallback( void )
{
    utCordic_CompleteCnt++;
    utCordic_Order++;
    utCordic_CompleteOrder = utCordic_Order;
}


/** \brief Half callback of the tests: counts the calls and records their order. */
static void UtCordic_HalfCallback( void )
{
    utCordic_HalfCnt++;
    utCordic_Order++;
    utCordic_HalfOrder = utCordic_Order;
}


/** \brief Error callback of the tests: counts the calls and records the last error mask. */
static void UtCordic_ErrorCallback( cordic_XferError_t xferError )
{
    utCordic_ErrorCnt++;
    utCordic_ErrorMask = xferError;
}


/** \brief Nvic_Set_PeriphIrq_Handler() callback: records the interrupt handler. */
static nvic_RequestState_t UtCordic_NvicHandlerStub( nvic_PeriphIrqList_t irqId,
                                                     const nvic_IsrCallback_t irqHandler,
                                                     int callCnt )
{
    ( void )irqId;
    ( void )callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utCordic_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** \brief Gpdma_Init() callback: copies the configuration, the call UT_CORDIC fail number fails. */
static gpdma_RequestState_t UtCordic_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    gpdma_RequestState_t retState = GPDMA_REQUEST_OK;

    if( UT_CORDIC_CAPTURE_CNT > utCordic_DmaCaptureCnt )
    {
        utCordic_DmaCapture[ utCordic_DmaCaptureCnt ].Config = *configStruct;
        utCordic_DmaCapture[ utCordic_DmaCaptureCnt ].Xfer   = *configStruct->TransferConfig;
    }
    else
    {
        /* Not captured */
    }

    utCordic_DmaCaptureCnt++;

    if( utCordic_GpdmaFailCall == callCnt )
    {
        retState = GPDMA_REQUEST_ERROR;
    }
    else
    {
        retState = GPDMA_REQUEST_OK;
    }

    return ( retState );
}

#endif /* CORDIC */

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Cordic_Get_ModuleVersion() returns the version of the module.
 *
 * \par Expected results
 * - Version is 1.0.0 (on every device line).
 */
void Ut_Cordic_Get_ModuleVersion_ReturnsVersion( void )
{
    const cordic_ModuleVersion_t version = Cordic_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   Cordic_Get_DefaultConfig() fills the default configuration.
 *
 * \par Expected results
 * - CORDIC_REQUEST_OK, cosine, q1.31, precision 6, scale 0, POLL mode, priority 0, Gpdma1 channel 0 (write)
 *   and 1 (read) at low priority, no callback.
 * - NULL pointer: CORDIC_REQUEST_ERROR.
 */
void Ut_Cordic_Get_DefaultConfig_ReturnsDefaults( void )
{
#if defined(CORDIC)
    cordic_Config_t config;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_COSINE, config.Function );
    TEST_ASSERT_EQUAL( CORDIC_FORMAT_Q31, config.Format );
    TEST_ASSERT_EQUAL( CORDIC_PRECISION_6, config.Precision );
    TEST_ASSERT_EQUAL( CORDIC_SCALE_0, config.Scale );
    TEST_ASSERT_EQUAL( CORDIC_XFER_MODE_POLL, config.XferMode );
    TEST_ASSERT_EQUAL_UINT32( 0u, config.IrqPriority );
    TEST_ASSERT_EQUAL( GPDMA_PERIPH_1, config.DmaPeriphId );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_0, config.DmaWriteChannelId );
    TEST_ASSERT_EQUAL( GPDMA_CHANNEL_1, config.DmaReadChannelId );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_LOW, config.DmaWritePriority );
    TEST_ASSERT_EQUAL( GPDMA_PRIORITY_LOW, config.DmaReadPriority );
    TEST_ASSERT_NULL( config.XferCompleteCallback );
    TEST_ASSERT_NULL( config.XferHalfCallback );
    TEST_ASSERT_NULL( config.XferErrorCallback );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_DefaultConfig( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ========================== NO PERIPHERAL ================================= */

/**
 * \brief   Every function returns CORDIC_REQUEST_ERROR on a device line without the CORDIC peripheral.
 *
 * \details STM32H503, STM32H523 and STM32H533 have no CORDIC. The case is ignored on the other lines.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR from every function that returns a request state, no RCC / NVIC / Gpdma call
 *   (strict mocks), Cordic_Task() returns.
 */
void Ut_Cordic_NoPeripheral_EveryFunctionReturnsError( void )
{
#if !defined(CORDIC)
    cordic_Config_t       config   = { 0 };
    cordic_Arguments_t    argument = { 0 };
    cordic_Results_t      results  = { 0 };
    cordic_ArrayRequest_t request  = { 0 };
    cordic_State_t        state    = CORDIC_STATE_IDLE;
    cordic_Function_t     function = CORDIC_FUNC_COSINE;
    cordic_Precision_t    precision = CORDIC_PRECISION_1;
    cordic_Scale_t        scale    = CORDIC_SCALE_0;
    cordic_Format_t       format   = CORDIC_FORMAT_Q31;
    cordic_IrqPrio_t      irqPrio  = 0u;
    cordic_Q31_t          qValue   = 0;
    cordic_Q15_t          q15Value = 0;
    float                 value    = 0.0f;

    TEST_ASSERT_EQUAL_UINT8( 1u, Cordic_Get_ModuleVersion().Major );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_DefaultConfig( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Deinit() );
    Cordic_Task();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Calculate( &argument, &results ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Function( CORDIC_FUNC_SINE ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Precision( CORDIC_PRECISION_3 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Precision( &precision ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Scale( CORDIC_SCALE_1 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Scale( &scale ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Format( CORDIC_FORMAT_Q15 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Format( &format ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( 0.5f, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_ToFloat( 0, &value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloatQ15( 0.5f, &q15Value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_ToFloatQ15( 0, &value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_IrqPriority( irqPrio ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_IrqPriority( &irqPrio ) );
#else
    TEST_IGNORE_MESSAGE( "Device line with the CORDIC peripheral, see the other test cases" );
#endif /* !CORDIC */
}

/* ============================== INIT / DEINIT ============================= */

/**
 * \brief   Cordic_Init() with the default configuration programs CORDIC_CSR and initializes the module.
 *
 * \par Expected results
 * - RCC: clock on, reset active, reset inactive (strict order), no NVIC / Gpdma call in the POLL mode.
 * - CORDIC_CSR: cosine, precision 6, scale 0, 32-bit values, NARGS and NRES set (0x00180060), no enable bit.
 * - The getters return the configuration, the state is idle.
 */
void Ut_Cordic_Init_Q31Cosine_ProgramsCsrAndState( void )
{
#if defined(CORDIC)
    cordic_Function_t  function  = CORDIC_FUNC_SQUARE_ROOT;
    cordic_Precision_t precision = CORDIC_PRECISION_1;
    cordic_Scale_t     scale     = CORDIC_SCALE_7;
    cordic_Format_t    format    = CORDIC_FORMAT_Q15;
    cordic_State_t     state     = CORDIC_STATE_ERROR;
    cordic_IrqPrio_t   irqPrio   = UT_CORDIC_DUMMY_PRIO;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_COSINE_Q31, CORDIC->CSR );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_COSINE, function );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Precision( &precision ) );
    TEST_ASSERT_EQUAL( CORDIC_PRECISION_6, precision );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Scale( &scale ) );
    TEST_ASSERT_EQUAL( CORDIC_SCALE_0, scale );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Format( &format ) );
    TEST_ASSERT_EQUAL( CORDIC_FORMAT_Q31, format );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_IRQ_PRIO, irqPrio );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Init() with the 16-bit format sets ARGSIZE and RESSIZE and keeps NARGS / NRES cleared.
 *
 * \par Expected results
 * - Arctangent, precision 3, scale 2, q1.15: CORDIC_CSR settings 0x00600234.
 */
void Ut_Cordic_Init_Q15Arctangent_SetsSizeBits( void )
{
#if defined(CORDIC)
    cordic_Config_t config = UtCordic_Get_Config( CORDIC_FUNC_ARCTANGENT,
                                                  CORDIC_FORMAT_Q15,
                                                  CORDIC_SCALE_2,
                                                  CORDIC_XFER_MODE_POLL );

    config.Precision = CORDIC_PRECISION_3;

    UtCordic_Expect_Start();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_ARCTAN_Q15, CORDIC->CSR & UT_CORDIC_CSR_MASK );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Init() rejects an invalid configuration before any hardware access.
 *
 * \details NULL, function, format, precision (0 and the count), scale (count and not valid for the function),
 *          transfer mode, DMA mode with the same channels, an invalid controller, channel and priority.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR every time, no RCC / NVIC / Gpdma call (strict mocks), the module stays not initialized.
 */
void Ut_Cordic_Init_InvalidConfig_ReturnsErrorWithoutHardware( void )
{
#if defined(CORDIC)
    cordic_Config_t config = UtCordic_Get_Config( CORDIC_FUNC_COSINE,
                                                  CORDIC_FORMAT_Q31,
                                                  CORDIC_SCALE_0,
                                                  CORDIC_XFER_MODE_POLL );
    cordic_Config_t bad;
    cordic_State_t  state = CORDIC_STATE_IDLE;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( NULL ) );

    bad = config;
    bad.Function = CORDIC_FUNC_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Format = CORDIC_FORMAT_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Precision = (cordic_Precision_t)0;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Precision = CORDIC_PRECISION_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Scale = CORDIC_SCALE_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Scale = CORDIC_SCALE_1;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Function = CORDIC_FUNC_NATURAL_LOGARITHM;
    bad.Scale    = CORDIC_SCALE_0;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.Function = CORDIC_FUNC_SQUARE_ROOT;
    bad.Scale    = CORDIC_SCALE_3;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.XferMode = CORDIC_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    config = UtCordic_Get_DmaConfig( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );

    bad = config;
    bad.DmaReadChannelId = bad.DmaWriteChannelId;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.DmaPeriphId = GPDMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.DmaWriteChannelId = GPDMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.DmaReadChannelId = GPDMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.DmaWritePriority = GPDMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    bad = config;
    bad.DmaReadPriority = GPDMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &bad ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A second Cordic_Init() without Cordic_Deinit() is rejected.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR without any RCC / NVIC / Gpdma call, the module stays initialized with the first settings.
 */
void Ut_Cordic_Init_AlreadyInitialized_ReturnsError( void )
{
#if defined(CORDIC)
    const cordic_Config_t config   = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                          CORDIC_FORMAT_Q31,
                                                          CORDIC_SCALE_0,
                                                          CORDIC_XFER_MODE_POLL );
    cordic_Function_t     function = CORDIC_FUNC_COSINE;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_COSINE, function );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A failing peripheral clock enable ends Cordic_Init() with the release of the peripheral.
 *
 * \par Expected results
 * - Rcc_Set_PeriphActive() fails: CORDIC_REQUEST_ERROR, then the release (NVIC off, reset pulse, clock off),
 *   the module is not initialized.
 */
void Ut_Cordic_Init_ClockFails_ReleasesAndReturnsError( void )
{
#if defined(CORDIC)
    const cordic_Config_t config = UtCordic_Get_Config( CORDIC_FUNC_COSINE,
                                                        CORDIC_FORMAT_Q31,
                                                        CORDIC_SCALE_0,
                                                        CORDIC_XFER_MODE_POLL );
    cordic_Function_t     function = CORDIC_FUNC_COSINE;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_ERROR );
    UtCordic_Expect_Release();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A failing reset of the peripheral ends Cordic_Init() with the release of the peripheral.
 *
 * \par Expected results
 * - Rcc_Set_ResetActive() fails: CORDIC_REQUEST_ERROR, CORDIC_CSR is not written, the release follows.
 */
void Ut_Cordic_Init_ResetFails_ReleasesAndReturnsError( void )
{
#if defined(CORDIC)
    const cordic_Config_t config = UtCordic_Get_Config( CORDIC_FUNC_COSINE,
                                                        CORDIC_FORMAT_Q31,
                                                        CORDIC_SCALE_0,
                                                        CORDIC_XFER_MODE_POLL );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_ERROR );
    UtCordic_Expect_Release();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Init() verifies CORDIC_CSR by the read-back and deinitializes the module on a difference.
 *
 * \details A faulty register is emulated by a HW model that clears the PRECISION field after every store.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR, the release (NVIC off, reset pulse, clock off) follows, the module is not initialized.
 */
void Ut_Cordic_Init_ReadBackMismatch_DeinitializesModule( void )
{
#if defined(CORDIC)
    const cordic_Config_t config = UtCordic_Get_Config( CORDIC_FUNC_COSINE,
                                                        CORDIC_FORMAT_Q31,
                                                        CORDIC_SCALE_0,
                                                        CORDIC_XFER_MODE_POLL );
    cordic_Function_t     function = CORDIC_FUNC_COSINE;

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtCordic_HwModel_ClearPrec ) );

    UtCordic_Expect_Start();
    UtCordic_Expect_Release();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Init() in the ISR mode configures the CORDIC interrupt in Nvic.
 *
 * \par Expected results
 * - After the Rcc calls: handler registered, priority, interrupt enabled (strict order); the handler pointer
 *   is recorded, Cordic_Get_IrqPriority() returns the priority.
 */
void Ut_Cordic_Init_IsrMode_ConfiguresNvic( void )
{
#if defined(CORDIC)
    const cordic_Config_t config  = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                         CORDIC_FORMAT_Q31,
                                                         CORDIC_SCALE_0,
                                                         CORDIC_XFER_MODE_ISR );
    cordic_IrqPrio_t      irqPrio = 0u;

    UtCordic_Expect_Start();
    UtCordic_Expect_Irq();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    TEST_ASSERT_NOT_NULL( utCordic_Isr );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_IRQ_PRIO, irqPrio );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A rejected interrupt priority in the ISR mode ends Cordic_Init() with the release.
 *
 * \par Expected results
 * - Nvic_Set_PeriphIrq_Prio() fails: the interrupt is not enabled, CORDIC_REQUEST_ERROR, the release follows,
 *   the module is not initialized.
 */
void Ut_Cordic_Init_IsrMode_NvicFails_ReleasesAndReturnsError( void )
{
#if defined(CORDIC)
    const cordic_Config_t config   = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                          CORDIC_FORMAT_Q31,
                                                          CORDIC_SCALE_0,
                                                          CORDIC_XFER_MODE_ISR );
    cordic_Function_t     function = CORDIC_FUNC_COSINE;

    UtCordic_Expect_Start();
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, UT_CORDIC_IRQ_PRIO, NVIC_REQUEST_ERROR );
    UtCordic_Expect_Release();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Deinit() releases the peripheral and the module is not initialized afterwards.
 *
 * \par Expected results
 * - NVIC interrupt off, reset pulse, clock off (strict order), CORDIC_REQUEST_OK; every getter and a second
 *   Cordic_Deinit() return CORDIC_REQUEST_ERROR without a hardware access.
 */
void Ut_Cordic_Deinit_Initialized_ReleasesPeripheral( void )
{
#if defined(CORDIC)
    cordic_Function_t function = CORDIC_FUNC_COSINE;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Expect_Release();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Deinit() of a module that is not initialized returns an error.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR, no RCC / NVIC / Gpdma call (strict mocks).
 */
void Ut_Cordic_Deinit_NotInitialized_ReturnsError( void )
{
#if defined(CORDIC)
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A failing step of Cordic_Deinit() is reported, the module is deinitialized anyway.
 *
 * \par Expected results
 * - Rcc_Set_PeriphInactive() fails: CORDIC_REQUEST_ERROR, the module is not initialized afterwards.
 */
void Ut_Cordic_Deinit_ClockFails_ReturnsErrorButDeinitializes( void )
{
#if defined(CORDIC)
    cordic_Function_t function = CORDIC_FUNC_COSINE;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, NVIC_REQUEST_OK );
    UtCordic_Expect_Reset();
    Rcc_Set_PeriphInactive_ExpectAndReturn( RCC_PERIPH_CORDIC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Deinit() );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ================================ SETTINGS ================================ */

/**
 * \brief   Cordic_Set_Function() writes the function and the number of the data words to CORDIC_CSR.
 *
 * \par Expected results
 * - Square root: FUNC 9, NARGS and NRES cleared (0x69); hyperbolic sine: FUNC 6, NRES set (0x00080066);
 *   Cordic_Get_Function() returns the function.
 */
void Ut_Cordic_Set_Function_Nominal_UpdatesCsr( void )
{
#if defined(CORDIC)
    cordic_Function_t function = CORDIC_FUNC_COSINE;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Function( CORDIC_FUNC_SQUARE_ROOT ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q31, CORDIC->CSR & UT_CORDIC_CSR_MASK );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_SQUARE_ROOT, function );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Function( CORDIC_FUNC_HYPERBOLIC_SINE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SINH_Q31, CORDIC->CSR & UT_CORDIC_CSR_MASK );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Set_Function() rejects an invalid function and a module that is not initialized.
 *
 * \par Expected results
 * - Not initialized: CORDIC_REQUEST_ERROR. Initialized: an invalid function returns CORDIC_REQUEST_ERROR and
 *   CORDIC_CSR and the stored function are unchanged.
 */
void Ut_Cordic_Set_Function_Invalid_ReturnsError( void )
{
#if defined(CORDIC)
    cordic_Function_t function = CORDIC_FUNC_SQUARE_ROOT;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Function( CORDIC_FUNC_SINE ) );

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Function( CORDIC_FUNC_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_COSINE_Q31, CORDIC->CSR );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_COSINE, function );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A register that does not hold the new settings keeps the stored settings.
 *
 * \details A faulty register is emulated by a HW model that clears the PRECISION field after every store.
 *
 * \par Expected results
 * - Cordic_Set_Function() returns CORDIC_REQUEST_ERROR and Cordic_Get_Function() still returns the cosine.
 */
void Ut_Cordic_Set_Function_ReadBackMismatch_KeepsStoredSettings( void )
{
#if defined(CORDIC)
    cordic_Function_t function = CORDIC_FUNC_SQUARE_ROOT;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( UtCordic_HwModel_ClearPrec ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Function( CORDIC_FUNC_SQUARE_ROOT ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_COSINE, function );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Set_Precision() writes the precision, an invalid precision is rejected.
 *
 * \par Expected results
 * - Precision 9: CORDIC_CSR.PRECISION = 9 (0x99 with the cosine words), Cordic_Get_Precision() returns it.
 * - Precision 0 (reserved) and the count: CORDIC_REQUEST_ERROR, no change.
 */
void Ut_Cordic_Set_Precision_Nominal_UpdatesCsr( void )
{
#if defined(CORDIC)
    cordic_Precision_t precision = CORDIC_PRECISION_1;

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Precision( UT_CORDIC_PRECISION_NINE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_PREC9, CORDIC->CSR & UT_CORDIC_CSR_MASK );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Precision( &precision ) );
    TEST_ASSERT_EQUAL( UT_CORDIC_PRECISION_NINE, precision );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Precision( (cordic_Precision_t)0 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Precision( CORDIC_PRECISION_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_PREC9, CORDIC->CSR & UT_CORDIC_CSR_MASK );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Set_Scale() writes the scale without checking it against the function.
 *
 * \par Expected results
 * - Scale 3 (not valid for the cosine, valid for the arctangent): CORDIC_CSR.SCALE = 3, Cordic_Get_Scale()
 *   returns it. The count is rejected without a change.
 */
void Ut_Cordic_Set_Scale_Nominal_UpdatesCsr( void )
{
#if defined(CORDIC)
    cordic_Scale_t scale = CORDIC_SCALE_0;

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_3 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_SCALE3, CORDIC->CSR & UT_CORDIC_CSR_MASK );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Scale( &scale ) );
    TEST_ASSERT_EQUAL( CORDIC_SCALE_3, scale );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Scale( CORDIC_SCALE_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_SCALE3, CORDIC->CSR & UT_CORDIC_CSR_MASK );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Set_Format() switches the data size bits and the word counts.
 *
 * \par Expected results
 * - q1.15: ARGSIZE and RESSIZE set, NARGS / NRES cleared (0x00600069 for the square root); back to q1.31:
 *   size bits cleared (0x69). An invalid format is rejected without a change.
 */
void Ut_Cordic_Set_Format_Nominal_UpdatesCsr( void )
{
#if defined(CORDIC)
    cordic_Format_t format = CORDIC_FORMAT_Q31;

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Format( CORDIC_FORMAT_Q15 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q15, CORDIC->CSR & UT_CORDIC_CSR_MASK );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Format( &format ) );
    TEST_ASSERT_EQUAL( CORDIC_FORMAT_Q15, format );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Format( CORDIC_FORMAT_CNT ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q15, CORDIC->CSR & UT_CORDIC_CSR_MASK );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Format( CORDIC_FORMAT_Q31 ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q31, CORDIC->CSR & UT_CORDIC_CSR_MASK );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The getters reject a NULL pointer and a module that is not initialized.
 *
 * \par Expected results
 * - Before Cordic_Init(): CORDIC_REQUEST_ERROR from every getter, the output variables are not changed.
 * - After Cordic_Init(): a NULL pointer returns CORDIC_REQUEST_ERROR.
 */
void Ut_Cordic_Get_Settings_NullAndNotInitialized_ReturnError( void )
{
#if defined(CORDIC)
    cordic_Function_t  function  = CORDIC_FUNC_SQUARE_ROOT;
    cordic_Precision_t precision = CORDIC_PRECISION_1;
    cordic_Scale_t     scale     = CORDIC_SCALE_2;
    cordic_Format_t    format    = CORDIC_FORMAT_Q15;
    cordic_State_t     state     = CORDIC_STATE_ERROR;
    cordic_IrqPrio_t   irqPrio   = UT_CORDIC_DUMMY_PRIO;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Precision( &precision ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Scale( &scale ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Format( &format ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_SQUARE_ROOT, function );
    TEST_ASSERT_EQUAL( CORDIC_PRECISION_1, precision );
    TEST_ASSERT_EQUAL( CORDIC_SCALE_2, scale );
    TEST_ASSERT_EQUAL( CORDIC_FORMAT_Q15, format );
    TEST_ASSERT_EQUAL( CORDIC_STATE_ERROR, state );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_DUMMY_PRIO, irqPrio );

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Function( NULL ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Precision( NULL ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Scale( NULL ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_Format( NULL ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_State( NULL ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_IrqPriority( NULL ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The configuration cannot be changed while an array calculation is running.
 *
 * \par Expected results
 * - After Cordic_Set_CalculateArray() in the POLL mode (state busy): every setter, Cordic_Set_Calculate(), a
 *   second array request and Cordic_Init() return CORDIC_REQUEST_ERROR without a hardware access.
 */
void Ut_Cordic_Set_Settings_WhileBusy_ReturnsError( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request   = UtCordic_Get_Request( UT_CORDIC_COUNT_TWO );
    const cordic_Config_t       config    = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                                 CORDIC_FORMAT_Q31,
                                                                 CORDIC_SCALE_0,
                                                                 CORDIC_XFER_MODE_POLL );
    cordic_State_t              state     = CORDIC_STATE_IDLE;
    cordic_Function_t           function  = CORDIC_FUNC_COSINE;

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Fill_Input( UT_CORDIC_COUNT_TWO );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Function( CORDIC_FUNC_SINE ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Precision( CORDIC_PRECISION_3 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Scale( CORDIC_SCALE_1 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Format( CORDIC_FORMAT_Q15 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, UtCordic_Calculate( UT_CORDIC_ARG_A, 0 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Init( &config ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_Function( &function ) );
    TEST_ASSERT_EQUAL( CORDIC_FUNC_SQUARE_ROOT, function );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Set_IrqPriority() and Cordic_Get_IrqPriority() forward the priority to Nvic.
 *
 * \par Expected results
 * - Not initialized: CORDIC_REQUEST_ERROR without an Nvic call.
 * - Initialized: Nvic accepts the priority: CORDIC_REQUEST_OK and the getter returns it; Nvic rejects the
 *   priority: CORDIC_REQUEST_ERROR and the stored priority is unchanged.
 */
void Ut_Cordic_Set_IrqPriority_ForwardsToNvic( void )
{
#if defined(CORDIC)
    cordic_IrqPrio_t irqPrio = 0u;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_IrqPriority( UT_CORDIC_IRQ_PRIO_OTHER ) );

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, UT_CORDIC_IRQ_PRIO_OTHER, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_IrqPriority( UT_CORDIC_IRQ_PRIO_OTHER ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_IRQ_PRIO_OTHER, irqPrio );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_CORDIC, UT_CORDIC_DUMMY_PRIO, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_IrqPriority( UT_CORDIC_DUMMY_PRIO ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_IrqPriority( &irqPrio ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_IRQ_PRIO_OTHER, irqPrio );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ========================== SINGLE CALCULATION ============================ */

/**
 * \brief   A cosine calculation in the 32-bit format writes both arguments and reads the results.
 *
 * \par Expected results
 * - CORDIC_WDATA receives the primary and then the secondary argument (log of the HW model), the results are
 *   read from CORDIC_RDATA (first argument + model offset); both results are the same word with the plain
 *   register memory (see the file note).
 */
void Ut_Cordic_Set_Calculate_CosineQ31_WritesArgumentsAndReadsResults( void )
{
#if defined(CORDIC)
    const cordic_Arguments_t arguments = { .Value1 = UT_CORDIC_ARG_A, .Value2 = CORDIC_Q31_ONE };
    cordic_Results_t         results   = { .Value1 = 0, .Value2 = 0 };

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );

    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_TWO, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)UT_CORDIC_ARG_A, utCordic_ModelLog[ 0 ] );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)CORDIC_Q31_ONE, utCordic_ModelLog[ 1 ] );
    TEST_ASSERT_EQUAL_INT32( ( UT_CORDIC_ARG_A + UT_CORDIC_MODEL_OFFSET ), results.Value1 );
    TEST_ASSERT_EQUAL_INT32( ( UT_CORDIC_ARG_A + UT_CORDIC_MODEL_OFFSET ), results.Value2 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Every function writes the expected number of the data words and programs the number of the results.
 *
 * \details Table of the ten functions with valid arguments and scale in the 32-bit format.
 *
 * \par Expected results
 * - Cosine, sine, phase and modulus write two words, the other functions one word.
 * - CORDIC_CSR.NRES is set for the cosine, sine, phase, modulus, hyperbolic cosine and hyperbolic sine only;
 *   CORDIC_CSR.NARGS follows the two argument functions.
 * - The first result is the first argument word + model offset.
 */
void Ut_Cordic_Set_Calculate_EveryFunction_WritesExpectedWordCount( void )
{
#if defined(CORDIC)
    const uint32_t caseCnt = ( sizeof( utCordic_FuncCase ) / sizeof( utCordic_FuncCase[ 0 ] ) );
    cordic_Results_t results = { .Value1 = 0, .Value2 = 0 };

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    TEST_ASSERT_EQUAL_UINT32( CORDIC_FUNC_CNT, caseCnt );

    for( uint32_t idx = 0u; caseCnt > idx; idx++ )
    {
        const utCordic_FuncCase_t * const funcCase = &utCordic_FuncCase[ idx ];
        const cordic_Arguments_t arguments = { .Value1 = funcCase->Value1, .Value2 = funcCase->Value2 };
        const uint32_t           logBefore = utCordic_ModelLogCnt;
        uint32_t                 nargsBit  = 0u;

        UtCordic_Select( funcCase->Function, funcCase->Scale );

        if( UT_CORDIC_WORDS_TWO == funcCase->ArgWords )
        {
            nargsBit = CORDIC_CSR_NARGS;
        }
        else
        {
            nargsBit = 0u;
        }

        TEST_ASSERT_EQUAL_HEX32( nargsBit | funcCase->NresBit, CORDIC->CSR & ( CORDIC_CSR_NARGS | CORDIC_CSR_NRES ) );
        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );
        TEST_ASSERT_EQUAL_UINT32( funcCase->ArgWords, utCordic_ModelLogCnt - logBefore );
        TEST_ASSERT_EQUAL_INT32( ( funcCase->Value1 + UT_CORDIC_MODEL_OFFSET ), results.Value1 );
    }
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A cosine calculation in the 16-bit format packs the arguments and sign extends the results.
 *
 * \par Expected results
 * - CORDIC_WDATA receives one word with the primary argument in the lower and the secondary argument in the
 *   upper half-word (0x4000EDCC for -0x1234 and 0x4000).
 * - The packed result (model: both half-words incremented) is unpacked: -0x1233 and 0x4002, sign extended.
 */
void Ut_Cordic_Set_Calculate_CosineQ15_PacksArgumentsAndSignExtendsResults( void )
{
#if defined(CORDIC)
    const cordic_Arguments_t arguments = { .Value1 = UT_CORDIC_Q15_ARG_NEGATIVE, .Value2 = UT_CORDIC_Q15_ARG_MODULUS };
    cordic_Results_t         results   = { .Value1 = 0, .Value2 = 0 };

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q15, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_Q15_WORD_COSINE, utCordic_ModelLog[ 0 ] );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_Q15_RES1_COSINE, results.Value1 );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_Q15_RES2_COSINE, results.Value2 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A square root in the 16-bit format writes the primary argument only and returns one result.
 *
 * \par Expected results
 * - The secondary argument is ignored (upper half-word of the written word is 0), the secondary result is 0
 *   although the upper half-word of CORDIC_RDATA is not.
 */
void Ut_Cordic_Set_Calculate_SqrtQ15_OneArgumentOneResult( void )
{
#if defined(CORDIC)
    const cordic_Arguments_t arguments = { .Value1 = UT_CORDIC_Q15_ARG_SQRT, .Value2 = CORDIC_Q15_ONE };
    cordic_Results_t         results   = { .Value1 = 0, .Value2 = CORDIC_Q15_ONE };

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q15, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)UT_CORDIC_Q15_ARG_SQRT, utCordic_ModelLog[ 0 ] );
    TEST_ASSERT_EQUAL_INT32( ( UT_CORDIC_Q15_ARG_SQRT + 1 ), results.Value1 );
    TEST_ASSERT_EQUAL_INT32( 0, results.Value2 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A function with one result returns zero as the secondary result.
 *
 * \par Expected results
 * - Square root in the 32-bit format: the primary result is read, the secondary result is 0.
 */
void Ut_Cordic_Set_Calculate_SingleResult_SecondaryResultIsZero( void )
{
#if defined(CORDIC)
    const cordic_Arguments_t arguments = { .Value1 = UT_CORDIC_ARG_B, .Value2 = UT_CORDIC_ARG_C };
    cordic_Results_t         results   = { .Value1 = 0, .Value2 = UT_CORDIC_ARG_D };

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_INT32( ( UT_CORDIC_ARG_B + UT_CORDIC_MODEL_OFFSET ), results.Value1 );
    TEST_ASSERT_EQUAL_INT32( 0, results.Value2 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The modulus of the cosine and sine and the combination of the function and the scale are checked.
 *
 * \par Expected results
 * - A negative modulus is rejected (cosine and sine), 0 and the largest value are accepted.
 * - A scale that is not valid for the function (cosine with scale 1) is rejected.
 * - A rejected calculation writes no data word.
 */
void Ut_Cordic_Set_Calculate_ModulusAndScale_AreChecked( void )
{
#if defined(CORDIC)
    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    UtCordic_Check_Rejected( UT_CORDIC_ARG_A, -1 );
    UtCordic_Check_Rejected( UT_CORDIC_ARG_A, CORDIC_Q31_MINUS_ONE );
    UtCordic_Check_Accepted( UT_CORDIC_ARG_A, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_ARG_B, CORDIC_Q31_ONE );

    UtCordic_Select( CORDIC_FUNC_SINE, CORDIC_SCALE_0 );
    UtCordic_Check_Rejected( UT_CORDIC_ARG_C, -1 );
    UtCordic_Check_Accepted( UT_CORDIC_ARG_D, 1 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_1 ) );
    UtCordic_Check_Rejected( UT_CORDIC_ARG_A, UT_CORDIC_ARG_B );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The argument limits of the hyperbolic functions are checked on both sides.
 *
 * \par Expected results
 * - Hyperbolic cosine and sine (scale 1): +-0.559 accepted, one step beyond rejected.
 * - Arctangent hyperbolic (scale 1): +-0.403 accepted, one step beyond rejected.
 */
void Ut_Cordic_Set_Calculate_HyperbolicLimits_AreChecked( void )
{
#if defined(CORDIC)
    UtCordic_Init_Poll( CORDIC_FUNC_HYPERBOLIC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_1 );
    UtCordic_Model_Start();

    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P559, 0 );
    UtCordic_Check_Accepted( -UT_CORDIC_Q31_0P559, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P559 + 1 ), 0 );
    UtCordic_Check_Rejected( ( -UT_CORDIC_Q31_0P559 - 1 ), 0 );

    UtCordic_Select( CORDIC_FUNC_HYPERBOLIC_SINE, CORDIC_SCALE_1 );
    UtCordic_Check_Accepted( ( UT_CORDIC_Q31_0P559 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P559 + 1 ), 0 );

    UtCordic_Select( CORDIC_FUNC_ARCTANGENT_HYPERBOLIC, CORDIC_SCALE_1 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P403, 0 );
    UtCordic_Check_Accepted( -UT_CORDIC_Q31_0P403, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P403 + 1 ), 0 );
    UtCordic_Check_Rejected( ( -UT_CORDIC_Q31_0P403 - 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_0 ) );
    UtCordic_Check_Rejected( 0, 0 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The argument range of the natural logarithm depends on the scale (RM0481 Table 174).
 *
 * \par Expected results
 * - Scale 1: 0.0535 .. 0.5, scale 2: 0.25 .. 0.75, scale 3: 0.375 .. 0.875, scale 4: 0.4375 .. 0.584; both
 *   bounds accepted, one step outside rejected.
 * - Scale 0 and scale 5 are rejected for the logarithm.
 */
void Ut_Cordic_Set_Calculate_LogarithmRanges_FollowScale( void )
{
#if defined(CORDIC)
    UtCordic_Init_Poll( CORDIC_FUNC_NATURAL_LOGARITHM, CORDIC_FORMAT_Q31, CORDIC_SCALE_1 );
    UtCordic_Model_Start();

    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P0535, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P5, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P0535 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P5 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_2 ) );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P25, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P75, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P25 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P75 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_3 ) );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P375, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P875, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P375 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P875 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_4 ) );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P4375, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P584, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P4375 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P584 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_0 ) );
    UtCordic_Check_Rejected( UT_CORDIC_Q31_0P5, 0 );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_5 ) );
    UtCordic_Check_Rejected( UT_CORDIC_Q31_0P5, 0 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The argument range of the square root depends on the scale (RM0481 Table 176).
 *
 * \par Expected results
 * - Scale 0: 0.027 .. 0.75, scale 1: 0.375 .. 0.875, scale 2: 0.4375 .. 0.585; both bounds accepted, one step
 *   outside rejected. Scale 3 is rejected.
 */
void Ut_Cordic_Set_Calculate_SquareRootRanges_FollowScale( void )
{
#if defined(CORDIC)
    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P027, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P75, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P027 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P75 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_1 ) );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P375, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P875, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P375 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P875 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_2 ) );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P4375, 0 );
    UtCordic_Check_Accepted( UT_CORDIC_Q31_0P585, 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P4375 - 1 ), 0 );
    UtCordic_Check_Rejected( ( UT_CORDIC_Q31_0P585 + 1 ), 0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_3 ) );
    UtCordic_Check_Rejected( UT_CORDIC_Q31_0P5, 0 );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   In the 16-bit format an argument outside of the q1.15 range is rejected.
 *
 * \par Expected results
 * - 0x8000 and -0x8001 (do not fit 16 bits) are rejected for the primary and the secondary argument, the
 *   largest and the smallest q1.15 value are accepted; a rejected calculation writes no word.
 */
void Ut_Cordic_Set_Calculate_Q15Range_IsChecked( void )
{
#if defined(CORDIC)
    UtCordic_Init_Poll( CORDIC_FUNC_PHASE, CORDIC_FORMAT_Q15, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    UtCordic_Check_Rejected( UT_CORDIC_Q15_ARG_TOO_LARGE, 0 );
    UtCordic_Check_Rejected( UT_CORDIC_Q15_ARG_TOO_SMALL, 0 );
    UtCordic_Check_Rejected( 0, UT_CORDIC_Q15_ARG_TOO_LARGE );
    UtCordic_Check_Rejected( 0, UT_CORDIC_Q15_ARG_TOO_SMALL );
    UtCordic_Check_Accepted( CORDIC_Q15_ONE, CORDIC_Q15_MINUS_ONE );
    UtCordic_Check_Accepted( CORDIC_Q15_MINUS_ONE, CORDIC_Q15_ONE );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A result that is not ready in time resets the peripheral and returns an error.
 *
 * \details CORDIC_CSR.RRDY is never set (no HW model).
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR after the bounded wait, the Rcc reset pulse follows, the settings are written again
 *   (CORDIC_CSR still holds the cosine settings), the module stays initialized and idle.
 */
void Ut_Cordic_Set_Calculate_Timeout_ResetsPeripheral( void )
{
#if defined(CORDIC)
    cordic_State_t state = CORDIC_STATE_ERROR;

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Expect_Reset();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, UtCordic_Calculate( UT_CORDIC_ARG_A, UT_CORDIC_ARG_B ) );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_COSINE_Q31, CORDIC->CSR );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Set_Calculate() rejects a NULL pointer and a module that is not initialized.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR, no data word is written (CORDIC_WDATA stays 0).
 */
void Ut_Cordic_Set_Calculate_InvalidParameters_ReturnError( void )
{
#if defined(CORDIC)
    const cordic_Arguments_t arguments = { .Value1 = UT_CORDIC_ARG_A, .Value2 = 0 };
    cordic_Results_t         results   = { .Value1 = 0, .Value2 = 0 };

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Calculate( &arguments, &results ) );

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Calculate( NULL, &results ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_Calculate( &arguments, NULL ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->WDATA );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ============================== CONVERSION ================================ */

/**
 * \brief   Cordic_Get_FromFloat() converts to q1.31 with the rounding and the range check.
 *
 * \par Expected results
 * - 0, 0.5, -0.5, 0.25 and -1.0 give 0, 0x40000000, 0xC0000000, 0x20000000 and 0x80000000; the largest float
 *   below 1.0 gives 0x7FFFFF80.
 * - 2^-32 gives 1 and -2^-32 gives -1 (half away from zero), 2^-33 gives 0.
 * - 1.0, 2.0, the float below -1.0, NaN, infinity and a NULL pointer return CORDIC_REQUEST_ERROR and leave the
 *   output unchanged.
 */
void Ut_Cordic_Get_FromFloat_Q31_ConvertsAndChecksRange( void )
{
#if defined(CORDIC)
    cordic_Q31_t qValue = 0;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_ZERO, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( 0, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_HALF, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_QV_HALF, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_MINUS_HALF, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_QV_MINUS_HALF, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_QUARTER, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_QV_QUARTER, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_MINUS_ONE, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( CORDIC_Q31_MINUS_ONE, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_BELOW_ONE, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_QV_BELOW_ONE, qValue );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_TINY, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( 1, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( -UT_CORDIC_FLOAT_TINY, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( -1, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_TINY_HALF, &qValue ) );
    TEST_ASSERT_EQUAL_INT32( 0, qValue );

    qValue = UT_CORDIC_ARG_A;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_ONE, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_TWO, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_BELOW_MINUS_ONE, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( NAN, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( INFINITY, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_HALF, NULL ) );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_ARG_A, qValue );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Get_ToFloat() converts q1.31 to float.
 *
 * \par Expected results
 * - 0, 0x40000000, 0x20000000 and 0x80000000 give 0, 0.5, 0.25 and -1.0.
 * - 0x7FFFFFFF gives 1.0 (a float has 24 significant bits).
 * - A NULL pointer returns CORDIC_REQUEST_ERROR.
 */
void Ut_Cordic_Get_ToFloat_Q31_Converts( void )
{
#if defined(CORDIC)
    float value = UT_CORDIC_FLOAT_TWO;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( 0, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_ZERO, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( UT_CORDIC_QV_HALF, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_HALF, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( UT_CORDIC_QV_QUARTER, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_QUARTER, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( CORDIC_Q31_MINUS_ONE, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_MINUS_ONE, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( CORDIC_Q31_ONE, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_ONE, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_ToFloat( UT_CORDIC_QV_HALF, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Get_FromFloatQ15() converts to q1.15 with the rounding, the saturation and the range check.
 *
 * \par Expected results
 * - 0.5, 0.25, -1.0 give 0x4000, 0x2000, -32768; 0.9999 gives 32765.
 * - 0.99998 and the largest float below 1.0 give 32767 (the rounded value would not fit and is saturated).
 * - 2^-16 gives 1 and -2^-16 gives -1 (half away from zero), 2^-17 gives 0.
 * - 1.0, NaN and a NULL pointer return CORDIC_REQUEST_ERROR.
 */
void Ut_Cordic_Get_FromFloat_Q15_ConvertsSaturatesAndChecksRange( void )
{
#if defined(CORDIC)
    cordic_Q15_t qValue = 0;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_HALF, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_QV_Q15_HALF, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_QUARTER, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_QV_Q15_QUARTER, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_MINUS_ONE, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( CORDIC_Q15_MINUS_ONE, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_Q15_NINE, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_QV_Q15_NINE, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_Q15_NEAR_ONE, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_QV_Q15_LAST, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_BELOW_ONE, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_QV_Q15_LAST, qValue );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_Q15_HALF_LSB, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( 1, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( -UT_CORDIC_FLOAT_Q15_HALF_LSB, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( -1, qValue );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_Q15_QUARTER_LSB, &qValue ) );
    TEST_ASSERT_EQUAL_INT16( 0, qValue );

    qValue = UT_CORDIC_QV_Q15_NINE;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_ONE, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_BELOW_MINUS_ONE, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloatQ15( NAN, &qValue ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_HALF, NULL ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_QV_Q15_NINE, qValue );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Get_ToFloatQ15() converts q1.15 to float.
 *
 * \par Expected results
 * - 0x4000, 0x2000, -32768 and 32767 give 0.5, 0.25, -1.0 and 32767 / 32768.
 * - A NULL pointer returns CORDIC_REQUEST_ERROR.
 */
void Ut_Cordic_Get_ToFloat_Q15_Converts( void )
{
#if defined(CORDIC)
    float value = UT_CORDIC_FLOAT_TWO;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloatQ15( UT_CORDIC_QV_Q15_HALF, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_HALF, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloatQ15( UT_CORDIC_QV_Q15_QUARTER, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_QUARTER, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloatQ15( CORDIC_Q15_MINUS_ONE, &value ) );
    TEST_ASSERT_EQUAL_FLOAT( UT_CORDIC_FLOAT_MINUS_ONE, value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloatQ15( CORDIC_Q15_ONE, &value ) );
    TEST_ASSERT_FLOAT_WITHIN( UT_CORDIC_FLOAT_TOLERANCE_Q15, UT_CORDIC_FLOAT_ONE, value );
    TEST_ASSERT_TRUE( UT_CORDIC_FLOAT_ONE > value );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_ToFloatQ15( UT_CORDIC_QV_Q15_HALF, NULL ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A number converted to the fixed-point format and back keeps its value.
 *
 * \par Expected results
 * - 0.3 and -0.7 are exact in q1.31; in q1.15 they are within half a step (2^-16).
 */
void Ut_Cordic_Convert_RoundTrip_KeepsValue( void )
{
#if defined(CORDIC)
    cordic_Q31_t q31Value = 0;
    cordic_Q15_t q15Value = 0;
    float        value    = UT_CORDIC_FLOAT_ZERO;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_ROUND_A, &q31Value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( q31Value, &value ) );
    TEST_ASSERT_FLOAT_WITHIN( UT_CORDIC_FLOAT_TOLERANCE, UT_CORDIC_FLOAT_ROUND_A, value );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( UT_CORDIC_FLOAT_ROUND_B, &q31Value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloat( q31Value, &value ) );
    TEST_ASSERT_FLOAT_WITHIN( UT_CORDIC_FLOAT_TOLERANCE, UT_CORDIC_FLOAT_ROUND_B, value );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_ROUND_A, &q15Value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloatQ15( q15Value, &value ) );
    TEST_ASSERT_FLOAT_WITHIN( UT_CORDIC_FLOAT_TOLERANCE_Q15, UT_CORDIC_FLOAT_ROUND_A, value );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( UT_CORDIC_FLOAT_ROUND_B, &q15Value ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_ToFloatQ15( q15Value, &value ) );
    TEST_ASSERT_FLOAT_WITHIN( UT_CORDIC_FLOAT_TOLERANCE_Q15, UT_CORDIC_FLOAT_ROUND_B, value );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The packing macros of the 16-bit data word put the first value in the lower half-word.
 *
 * \par Expected results
 * - CORDIC_Q15_PACK( -0x1234, 0x4000 ) is 0x4000EDCC and the two extractors return the values (sign extended).
 * - 0x8001FFFE gives -2 and -32767.
 */
void Ut_Cordic_Q15Macros_PackAndExtract( void )
{
#if defined(CORDIC)
    const cordic_Q31_t word = CORDIC_Q15_PACK( UT_CORDIC_Q15_ARG_NEGATIVE, UT_CORDIC_Q15_ARG_MODULUS );

    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_Q15_WORD_COSINE, (uint32_t)word );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_Q15_ARG_NEGATIVE, CORDIC_Q15_FIRST( word ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_Q15_ARG_MODULUS, CORDIC_Q15_SECOND( word ) );

    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_Q15_RESULT_FIRST, CORDIC_Q15_FIRST( UT_CORDIC_Q15_WORD_RESULT ) );
    TEST_ASSERT_EQUAL_INT16( UT_CORDIC_Q15_RESULT_SECOND, CORDIC_Q15_SECOND( UT_CORDIC_Q15_WORD_RESULT ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ========================= ARRAY CALCULATION, POLL ======================== */

/**
 * \brief   An invalid array request is rejected before any hardware access.
 *
 * \details Not initialized, NULL request, NULL arrays, count 0 and above the limit, a scale that is not valid
 *          for the function.
 *
 * \par Expected results
 * - CORDIC_REQUEST_ERROR every time, the state stays idle, no data word is written, no mock call.
 */
void Ut_Cordic_Set_CalculateArray_InvalidRequest_ReturnsError( void )
{
#if defined(CORDIC)
    cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_TWO );
    cordic_ArrayRequest_t bad;
    cordic_State_t        state = CORDIC_STATE_BUSY;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( NULL ) );

    bad = request;
    bad.Input = NULL;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &bad ) );

    bad = request;
    bad.Output = NULL;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &bad ) );

    bad = request;
    bad.Count = UT_CORDIC_COUNT_ZERO;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &bad ) );

    bad = request;
    bad.Count = UT_CORDIC_COUNT_TOO_LARGE;
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &bad ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_3 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->WDATA );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The POLL mode calculates a short array by Cordic_Task() (square root, 32-bit, one word each way).
 *
 * \par Expected results
 * - After the request the module is busy and nothing is calculated; one Cordic_Task() call calculates the three
 *   calculations: output[i] = input[i] + model offset, the module is idle again.
 * - The half callback is called once before the complete callback, the error callback never.
 */
void Ut_Cordic_Set_CalculateArray_PollQ31_OneResultFunction_CalculatesInTask( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_Config_t             config  = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                               CORDIC_FORMAT_Q31,
                                                               CORDIC_SCALE_0,
                                                               CORDIC_XFER_MODE_POLL );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Callbacks( &config );
    UtCordic_Expect_Start();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    UtCordic_Model_Start();
    UtCordic_Fill_Input( UT_CORDIC_COUNT_THREE );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_OUT_FILL, utCordic_Out[ 0 ] );

    Cordic_Task();

    for( uint32_t idx = 0u; UT_CORDIC_COUNT_THREE > idx; idx++ )
    {
        TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ idx ] + UT_CORDIC_MODEL_OFFSET ), utCordic_Out[ idx ] );
    }

    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_OUT_FILL, utCordic_Out[ UT_CORDIC_COUNT_THREE ] );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_ErrorCnt );
    TEST_ASSERT_TRUE( utCordic_HalfOrder < utCordic_CompleteOrder );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR & ( CORDIC_CSR_IEN | CORDIC_CSR_DMAREN | CORDIC_CSR_DMAWEN ) );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The POLL mode of a function with two arguments and two results uses two words each way.
 *
 * \par Expected results
 * - Cosine, 32-bit, two calculations: four input words are written (log of the HW model), four output words
 *   are filled with the results of their calculation (both results of a calculation are the same word with
 *   the plain register memory, see the file note).
 */
void Ut_Cordic_Set_CalculateArray_PollQ31_TwoResultFunction_UsesTwoWordsPerCalculation( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_TWO );

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();
    UtCordic_Fill_Input( ( UT_CORDIC_COUNT_TWO * UT_CORDIC_WORDS_TWO ) );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    Cordic_Task();

    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_TWO * UT_CORDIC_WORDS_TWO ), utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ 0 ] + UT_CORDIC_MODEL_OFFSET ), utCordic_Out[ 0 ] );
    TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ 0 ] + UT_CORDIC_MODEL_OFFSET ), utCordic_Out[ 1 ] );
    TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ UT_CORDIC_IDX_SECOND ] + UT_CORDIC_MODEL_OFFSET ),
                             utCordic_Out[ UT_CORDIC_IDX_SECOND ] );
    TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ UT_CORDIC_IDX_SECOND ] + UT_CORDIC_MODEL_OFFSET ),
                             utCordic_Out[ UT_CORDIC_IDX_SECOND + 1 ] );
    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_OUT_FILL, utCordic_Out[ UT_CORDIC_IDX_AFTER ] );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The POLL mode of the 16-bit format uses one packed word per calculation.
 *
 * \par Expected results
 * - Cosine, 16-bit, three calculations: three words are written, output[i] = input[i] + 0x00020001 (both
 *   half-words incremented by the HW model).
 */
void Ut_Cordic_Set_CalculateArray_PollQ15_UsesPackedWords( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );

    UtCordic_Init_Poll( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q15, CORDIC_SCALE_0 );
    UtCordic_Model_Start();
    UtCordic_Fill_Input( UT_CORDIC_COUNT_THREE );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    Cordic_Task();

    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_THREE, utCordic_ModelLogCnt );

    for( uint32_t idx = 0u; UT_CORDIC_COUNT_THREE > idx; idx++ )
    {
        TEST_ASSERT_EQUAL_HEX32( ( (uint32_t)utCordic_In[ idx ] + UT_CORDIC_MODEL_Q15_ADD ),
                                 (uint32_t)utCordic_Out[ idx ] );
    }

    TEST_ASSERT_EQUAL_INT32( UT_CORDIC_OUT_FILL, utCordic_Out[ UT_CORDIC_COUNT_THREE ] );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Task() calculates at most eight calculations per call and reports the half once.
 *
 * \par Expected results
 * - 20 calculations: after the 1st call 8 are done (busy, no callback), after the 2nd call 16 are done (busy,
 *   the half callback was called once), after the 3rd call all are done (idle, the complete callback once).
 */
void Ut_Cordic_Task_PollArray_ServesEightCalculationsPerCall( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_TWENTY );
    cordic_Config_t             config  = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                               CORDIC_FORMAT_Q31,
                                                               CORDIC_SCALE_0,
                                                               CORDIC_XFER_MODE_POLL );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Callbacks( &config );
    UtCordic_Expect_Start();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    UtCordic_Model_Start();
    UtCordic_Fill_Input( UT_CORDIC_COUNT_TWENTY );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    Cordic_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_TASK, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );

    Cordic_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_TWICE_TASK, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );

    Cordic_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_TWENTY, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );

    for( uint32_t idx = 0u; UT_CORDIC_COUNT_TWENTY > idx; idx++ )
    {
        TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ idx ] + UT_CORDIC_MODEL_OFFSET ), utCordic_Out[ idx ] );
    }

    Cordic_Task();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_TWENTY, utCordic_ModelLogCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A result that is not ready in time ends the POLL array request with a timeout error.
 *
 * \details CORDIC_CSR.RRDY is never set (no HW model).
 *
 * \par Expected results
 * - Cordic_Task(): the error callback is called once with CORDIC_XFER_ERROR_TIMEOUT, the Rcc reset pulse follows,
 *   the state is CORDIC_STATE_ERROR, the complete callback is not called, CORDIC_CSR holds the settings.
 * - The configuration can be changed again after the error.
 */
void Ut_Cordic_Task_PollArray_Timeout_ReportsError( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_TWO );
    cordic_Config_t             config  = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                               CORDIC_FORMAT_Q31,
                                                               CORDIC_SCALE_0,
                                                               CORDIC_XFER_MODE_POLL );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Callbacks( &config );
    UtCordic_Expect_Start();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    UtCordic_Fill_Input( UT_CORDIC_COUNT_TWO );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    UtCordic_Expect_Reset();
    Cordic_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ErrorCnt );
    TEST_ASSERT_EQUAL( CORDIC_XFER_ERROR_TIMEOUT, utCordic_ErrorMask );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_ERROR, state );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q31, CORDIC->CSR );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Precision( CORDIC_PRECISION_3 ) );

    Cordic_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Task() without a running array request does nothing.
 *
 * \par Expected results
 * - Not initialized and idle: no register write, no mock call, no callback.
 */
void Ut_Cordic_Task_NoRequest_DoesNothing( void )
{
#if defined(CORDIC)
    Cordic_Task();

    UtCordic_Init_Poll( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0 );
    UtCordic_Model_Start();

    Cordic_Task();

    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q31, CORDIC->CSR );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ========================= ARRAY CALCULATION, ISR ========================= */

/**
 * \brief   The ISR mode calculates an array in the CORDIC interrupt.
 *
 * \par Expected results
 * - The request enables CORDIC_CSR.IEN and writes the first argument (busy).
 * - Every call of the registered handler reads a result and writes the next argument; after the third call the
 *   request is complete: output[i] = input[i] + model offset, IEN cleared, idle, half callback once before the
 *   complete callback.
 * - A further handler call changes nothing.
 */
void Ut_Cordic_Isr_ArrayRequest_CalculatesInInterrupt( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_Config_t             config  = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                               CORDIC_FORMAT_Q31,
                                                               CORDIC_SCALE_0,
                                                               CORDIC_XFER_MODE_ISR );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Callbacks( &config );
    UtCordic_Expect_Start();
    UtCordic_Expect_Irq();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    TEST_ASSERT_NOT_NULL( utCordic_Isr );

    UtCordic_Model_Start();
    UtCordic_Fill_Input( UT_CORDIC_COUNT_THREE );
    UtCordic_Fill_Output();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL_HEX32( CORDIC_CSR_IEN, CORDIC->CSR & CORDIC_CSR_IEN );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_TWO, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ 0 ] + UT_CORDIC_MODEL_OFFSET ), utCordic_Out[ 0 ] );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_THREE, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_THREE, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_TRUE( utCordic_HalfOrder < utCordic_CompleteOrder );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR & CORDIC_CSR_IEN );

    for( uint32_t idx = 0u; UT_CORDIC_COUNT_THREE > idx; idx++ )
    {
        TEST_ASSERT_EQUAL_INT32( ( utCordic_In[ idx ] + UT_CORDIC_MODEL_OFFSET ), utCordic_Out[ idx ] );
    }

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_THREE, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The half callback is called when exactly the half of the calculations is done.
 *
 * \par Expected results
 * - ISR mode, four calculations: no half callback after the first handler call, one after the second (two done,
 *   two remaining), still one after the third and the fourth call; the complete callback only after the fourth.
 */
void Ut_Cordic_Isr_ArrayRequest_ReportsHalfAtTheMiddle( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_FOUR );
    cordic_Config_t             config  = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                               CORDIC_FORMAT_Q31,
                                                               CORDIC_SCALE_0,
                                                               CORDIC_XFER_MODE_ISR );

    UtCordic_Init_Callbacks( &config );
    UtCordic_Expect_Start();
    UtCordic_Expect_Irq();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );

    UtCordic_Model_Start();
    UtCordic_Fill_Input( UT_CORDIC_COUNT_FOUR );
    UtCordic_Fill_Output();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_HalfCnt );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );

    utCordic_Isr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The interrupt handler ignores an interrupt while no array request runs.
 *
 * \par Expected results
 * - No data word is written, CORDIC_CSR is unchanged, no callback is called.
 */
void Ut_Cordic_Isr_NoRequest_IsIgnored( void )
{
#if defined(CORDIC)
    cordic_Config_t config = UtCordic_Get_Config( CORDIC_FUNC_SQUARE_ROOT,
                                                  CORDIC_FORMAT_Q31,
                                                  CORDIC_SCALE_0,
                                                  CORDIC_XFER_MODE_ISR );

    UtCordic_Init_Callbacks( &config );
    UtCordic_Expect_Start();
    UtCordic_Expect_Irq();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
    TEST_ASSERT_NOT_NULL( utCordic_Isr );

    UtCordic_Model_Start();
    CORDIC->CSR |= CORDIC_CSR_RRDY;

    utCordic_Isr();

    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_ModelLogCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}

/* ========================= ARRAY CALCULATION, DMA ========================= */

/**
 * \brief   The DMA mode configures the two Gpdma channels and enables the DMA requests of the coprocessor.
 *
 * \details Cosine, 32-bit, 4 calculations: two words each way, 32 bytes per block.
 *
 * \par Expected results
 * - Write channel: memory to peripheral, request CORDIC_WRITE, source = input array incremented, destination =
 *   CORDIC_WDATA static, 32-bit data, burst 1, block 32 bytes, request mode single, channel 2, priority high.
 * - Read channel: peripheral to memory, request CORDIC_READ, source = CORDIC_RDATA static, destination = output
 *   array incremented, block 32 bytes, channel 3, priority medium, the transfer complete callback set, no half
 *   transfer callback (not configured); both channels have the error callback, one single linked list item.
 * - The channels are activated (read first), CORDIC_CSR.DMAREN and DMAWEN are set, the state is busy, no
 *   callback was called.
 */
void Ut_Cordic_Set_CalculateArray_Dma_ConfiguresChannelsAndEnablesRequests( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_FOUR );
    const uint32_t              wdata   = (uint32_t)(uintptr_t)&CORDIC->WDATA;
    const uint32_t              rdata   = (uint32_t)(uintptr_t)&CORDIC->RDATA;
    const uint32_t              blockBytes = ( UT_CORDIC_COUNT_FOUR * UT_CORDIC_WORDS_TWO * UT_CORDIC_WORD_BYTES );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Dma( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );
    UtCordic_Expect_DmaStart();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_CHANNEL_CNT, utCordic_DmaCaptureCnt );

    {
        const utCordic_DmaCapture_t * const capture = &utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ];

        TEST_ASSERT_EQUAL( UT_CORDIC_DMA_PERIPH, capture->Config.PeriphId );
        TEST_ASSERT_EQUAL( UT_CORDIC_DMA_WRITE_CH, capture->Config.ChannelId );
        TEST_ASSERT_EQUAL( GPDMA_PRIORITY_HIGH, capture->Config.ChannelPrio );
        TEST_ASSERT_EQUAL( GPDMA_XFER_EXEC_BLOCKING, capture->Config.TransferExecMode );
        TEST_ASSERT_EQUAL_UINT32( 1u, capture->Config.TransfersCount );
        TEST_ASSERT_EQUAL( GPDMA_TRANSFER_LIST_ACCESS_SINGLE, capture->Config.XferListAccessMode );
        TEST_ASSERT_NOT_NULL( capture->Config.XferList );
        TEST_ASSERT_EQUAL( GPDMA_TRANSFER_LIST_LOCKED, capture->Config.TransferLockState );
        TEST_ASSERT_NULL( capture->Config.TransferCompleteIsr );
        TEST_ASSERT_NULL( capture->Config.HalfTransferIsr );
        TEST_ASSERT_NOT_NULL( capture->Config.ErrorIsr );
        TEST_ASSERT_EQUAL( GPDMA_ERROR_ALL, capture->Config.ErrorMask );
        TEST_ASSERT_EQUAL( GPDMA_DIR_MEMORY_TO_PERIPH, capture->Xfer.Direction );
        TEST_ASSERT_EQUAL( GPDMA_REQ_CORDIC_WRITE, capture->Xfer.RequestSource );
        TEST_ASSERT_EQUAL( GPDMA_PERIPH_REQ_SINGLE, capture->Xfer.RequestMode );
        TEST_ASSERT_EQUAL( GPDMA_TRG_NOT_USED, capture->Xfer.TriggerType );
        TEST_ASSERT_EQUAL_UINT32( blockBytes, capture->Xfer.BlockSize );
        TEST_ASSERT_EQUAL_UINT32( 0u, capture->Xfer.BlockRepetitionCount );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utCordic_In, capture->Xfer.SourceAddr );
        TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_32BITS, capture->Xfer.SourceDataSize );
        TEST_ASSERT_EQUAL_UINT32( 1u, capture->Xfer.SourceBurstLength );
        TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, capture->Xfer.SourceAddrMode );
        TEST_ASSERT_EQUAL_HEX32( wdata, capture->Xfer.DestinationAddr );
        TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_32BITS, capture->Xfer.DestinationDataSize );
        TEST_ASSERT_EQUAL_UINT32( 1u, capture->Xfer.DestinationBurstLength );
        TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, capture->Xfer.DestinationAddrMode );
    }

    {
        const utCordic_DmaCapture_t * const capture = &utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ];

        TEST_ASSERT_EQUAL( UT_CORDIC_DMA_PERIPH, capture->Config.PeriphId );
        TEST_ASSERT_EQUAL( UT_CORDIC_DMA_READ_CH, capture->Config.ChannelId );
        TEST_ASSERT_EQUAL( GPDMA_PRIORITY_MEDIUM, capture->Config.ChannelPrio );
        TEST_ASSERT_NOT_NULL( capture->Config.XferList );
        TEST_ASSERT_TRUE( capture->Config.XferList != utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ].Config.XferList );
        TEST_ASSERT_NOT_NULL( capture->Config.TransferCompleteIsr );
        TEST_ASSERT_NULL( capture->Config.HalfTransferIsr );
        TEST_ASSERT_NOT_NULL( capture->Config.ErrorIsr );
        TEST_ASSERT_EQUAL( GPDMA_DIR_PERIPH_TO_MEMORY, capture->Xfer.Direction );
        TEST_ASSERT_EQUAL( GPDMA_REQ_CORDIC_READ, capture->Xfer.RequestSource );
        TEST_ASSERT_EQUAL( GPDMA_PERIPH_REQ_SINGLE, capture->Xfer.RequestMode );
        TEST_ASSERT_EQUAL_UINT32( blockBytes, capture->Xfer.BlockSize );
        TEST_ASSERT_EQUAL_HEX32( rdata, capture->Xfer.SourceAddr );
        TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_32BITS, capture->Xfer.SourceDataSize );
        TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC, capture->Xfer.SourceAddrMode );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utCordic_Out, capture->Xfer.DestinationAddr );
        TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_32BITS, capture->Xfer.DestinationDataSize );
        TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT, capture->Xfer.DestinationAddrMode );
    }

    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_DMA_BITS, CORDIC->CSR & UT_CORDIC_CSR_DMA_BITS );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR & CORDIC_CSR_IEN );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt + utCordic_HalfCnt + utCordic_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The transfer complete callback of the result channel ends the DMA request.
 *
 * \details The half callback is configured: the half transfer callback of the result channel is set.
 *
 * \par Expected results
 * - The half transfer callback calls the half callback once, the transfer complete callback the complete
 *   callback once (the half callback is not repeated); the state is idle, CORDIC_CSR.DMAREN and DMAWEN are
 *   cleared, the error callback is not called.
 */
void Ut_Cordic_Dma_TransferComplete_EndsRequest( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_FIVE );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Dma( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, UtCordic_HalfCallback );
    UtCordic_Expect_DmaStart();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    TEST_ASSERT_NOT_NULL( utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.HalfTransferIsr );
    TEST_ASSERT_NOT_NULL( utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.TransferCompleteIsr );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_FIVE * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Xfer.BlockSize );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_FIVE * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ].Xfer.BlockSize );

    utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.HalfTransferIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );

    utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.TransferCompleteIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_HalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_ErrorCnt );
    TEST_ASSERT_TRUE( utCordic_HalfOrder < utCordic_CompleteOrder );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR & UT_CORDIC_CSR_DMA_BITS );

    utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.TransferCompleteIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The block sizes follow the function and the format.
 *
 * \details Hyperbolic cosine, 32-bit, 5 calculations: one argument word and two result words per calculation.
 *
 * \par Expected results
 * - Write block 5 * 1 * 4 = 20 bytes, read block 5 * 2 * 4 = 40 bytes.
 */
void Ut_Cordic_Set_CalculateArray_Dma_BlockSizesFollowFunction( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_FIVE );

    UtCordic_Init_Dma( CORDIC_FUNC_HYPERBOLIC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_1, NULL );
    UtCordic_Expect_DmaStart();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_FIVE * 1u * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ].Xfer.BlockSize );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_FIVE * UT_CORDIC_WORDS_TWO * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Xfer.BlockSize );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   In the 16-bit format the DMA block is one packed word per calculation each way.
 *
 * \details Cosine, 16-bit, 5 calculations.
 *
 * \par Expected results
 * - Write and read block 5 * 4 = 20 bytes.
 */
void Ut_Cordic_Set_CalculateArray_Dma_Q15BlockSizesArePacked( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_FIVE );

    UtCordic_Init_Dma( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q15, CORDIC_SCALE_0, NULL );
    UtCordic_Expect_DmaStart();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_FIVE * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ].Xfer.BlockSize );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_FIVE * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Xfer.BlockSize );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A Gpdma error ends the DMA request with an error and resets the peripheral.
 *
 * \par Expected results
 * - The error callback of a channel: both channels are stopped (write first), the Rcc reset pulse follows, the
 *   state is CORDIC_STATE_ERROR, the error callback is called once with CORDIC_XFER_ERROR_DMA, the DMA requests
 *   of the coprocessor are disabled and CORDIC_CSR holds the settings.
 * - A second error callback of the other channel changes nothing (no further call).
 */
void Ut_Cordic_Dma_ChannelError_StopsChannelsAndResetsPeripheral( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Dma( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );
    UtCordic_Expect_DmaStart();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    Gpdma_Set_ChannelInactive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_WRITE_CH, GPDMA_REQUEST_OK );
    Gpdma_Set_ChannelInactive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_READ_CH, GPDMA_REQUEST_OK );
    UtCordic_Expect_Reset();

    utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ].Config.ErrorIsr( GPDMA_ERROR_TRANSFER );

    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ErrorCnt );
    TEST_ASSERT_EQUAL( CORDIC_XFER_ERROR_DMA, utCordic_ErrorMask );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_ERROR, state );
    TEST_ASSERT_EQUAL_HEX32( UT_CORDIC_CSR_SQRT_Q31, CORDIC->CSR );

    utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.ErrorIsr( GPDMA_ERROR_TRANSFER );
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A DMA request that does not fit the Gpdma block or is not word aligned is rejected.
 *
 * \details Cosine, 32-bit: 8192 calculations make 65536 bytes (one more than a Gpdma block), 8191 make 65528.
 *
 * \par Expected results
 * - 8192 calculations, an input array and an output array that is not 4 byte aligned: CORDIC_REQUEST_ERROR
 *   without a Gpdma call (strict mocks), the module stays idle.
 * - 8191 calculations are accepted: both blocks 65528 bytes (at most 65535).
 */
void Ut_Cordic_Set_CalculateArray_Dma_BlockLimitAndAlignment_AreChecked( void )
{
#if defined(CORDIC)
    cordic_ArrayRequest_t request = { .Input  = utCordic_BigIn,
                                      .Output = utCordic_BigOut,
                                      .Count  = UT_CORDIC_COUNT_BLOCK_OVER };
    cordic_State_t        state   = CORDIC_STATE_BUSY;

    UtCordic_Init_Dma( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );

    request.Count = UT_CORDIC_COUNT_TWO;
    request.Input = (const cordic_Q31_t *)( (uintptr_t)utCordic_BigIn + 1u );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );

    request.Input  = utCordic_BigIn;
    request.Output = (cordic_Q31_t *)( (uintptr_t)utCordic_BigOut + UT_CORDIC_MISALIGN_HALF );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );

    request.Output = utCordic_BigOut;
    request.Count  = UT_CORDIC_COUNT_BLOCK_LIMIT;
    UtCordic_Expect_DmaStart();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_BLOCK_LIMIT * UT_CORDIC_WORDS_TWO * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_WRITE ].Xfer.BlockSize );
    TEST_ASSERT_EQUAL_UINT32( ( UT_CORDIC_COUNT_BLOCK_LIMIT * UT_CORDIC_WORDS_TWO * UT_CORDIC_WORD_BYTES ),
                              utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Xfer.BlockSize );
    TEST_ASSERT_TRUE( UT_CORDIC_DMA_MAX_BYTES >= utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Xfer.BlockSize );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A failing Gpdma channel configuration ends the DMA request start with the release of the channels.
 *
 * \par Expected results
 * - Gpdma_Init() of the read channel fails: CORDIC_REQUEST_ERROR, both channels are released (Gpdma_Deinit and
 *   interrupt off), the state is idle, the DMA requests of the coprocessor stay disabled.
 */
void Ut_Cordic_Set_CalculateArray_Dma_GpdmaInitFails_ReleasesChannels( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_State_t              state   = CORDIC_STATE_BUSY;

    UtCordic_Init_Dma( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );

    UtCordic_Expect_DmaChannel( UT_CORDIC_DMA_WRITE_CH );
    Gpdma_Get_DefaultConfig_ExpectAnyArgsAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_ExpectAnyArgsAndReturn( GPDMA_REQUEST_ERROR );
    Gpdma_Init_AddCallback( UtCordic_GpdmaInitStub );
    utCordic_GpdmaFailCall = 1;
    UtCordic_Expect_DmaRelease();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR & UT_CORDIC_CSR_DMA_BITS );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   A Gpdma channel that cannot be activated ends the DMA request start with the release of the channels.
 *
 * \par Expected results
 * - Gpdma_Set_ChannelActive() of the read channel fails: CORDIC_REQUEST_ERROR, both channels are released, the
 *   state is idle, CORDIC_CSR.DMAREN and DMAWEN stay cleared.
 */
void Ut_Cordic_Set_CalculateArray_Dma_ActivationFails_ReleasesChannels( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_State_t              state   = CORDIC_STATE_BUSY;

    UtCordic_Init_Dma( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );

    UtCordic_Expect_DmaChannel( UT_CORDIC_DMA_WRITE_CH );
    UtCordic_Expect_DmaChannel( UT_CORDIC_DMA_READ_CH );
    Gpdma_Set_ChannelActive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_READ_CH, GPDMA_REQUEST_ERROR );
    Gpdma_Set_ChannelActive_ExpectAndReturn( UT_CORDIC_DMA_PERIPH, UT_CORDIC_DMA_WRITE_CH, GPDMA_REQUEST_OK );
    UtCordic_Expect_DmaRelease();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_HEX32( 0u, CORDIC->CSR & UT_CORDIC_CSR_DMA_BITS );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   The next DMA request releases the Gpdma channels of the previous one first.
 *
 * \par Expected results
 * - After the end of the first request a second request calls Gpdma_Deinit() of the write and the read channel and
 *   switches both interrupts off, then configures the channels again; CORDIC_REQUEST_OK, busy.
 */
void Ut_Cordic_Set_CalculateArray_Dma_SecondRequest_ReleasesPreviousChannels( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_State_t              state   = CORDIC_STATE_IDLE;

    UtCordic_Init_Dma( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );
    UtCordic_Expect_DmaStart();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    utCordic_DmaCapture[ UT_CORDIC_CAPTURE_READ ].Config.TransferCompleteIsr();
    TEST_ASSERT_EQUAL_UINT32( 1u, utCordic_CompleteCnt );

    UtCordic_Expect_DmaRelease();
    UtCordic_Expect_DmaStart();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    TEST_ASSERT_EQUAL_UINT32( UT_CORDIC_COUNT_FOUR, utCordic_DmaCaptureCnt );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_BUSY, state );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}


/**
 * \brief   Cordic_Deinit() during a DMA request releases the Gpdma channels without a callback.
 *
 * \par Expected results
 * - Gpdma_Deinit() of both channels and both interrupts off, then NVIC off, reset pulse and clock off; the
 *   function returns CORDIC_REQUEST_OK, no callback is called, the module is not initialized.
 */
void Ut_Cordic_Deinit_WhileDmaBusy_ReleasesChannels( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request  = UtCordic_Get_Request( UT_CORDIC_COUNT_THREE );
    cordic_State_t              state    = CORDIC_STATE_IDLE;

    UtCordic_Init_Dma( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, NULL );
    UtCordic_Expect_DmaStart();
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    UtCordic_Expect_DmaRelease();
    UtCordic_Expect_Release();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_ERROR, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utCordic_CompleteCnt + utCordic_HalfCnt + utCordic_ErrorCnt );
#else
    TEST_IGNORE_MESSAGE( "Device line without the CORDIC peripheral" );
#endif /* CORDIC */
}
