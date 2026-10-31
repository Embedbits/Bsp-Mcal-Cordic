/**
 * \author Mr.Nobody
 * \file ItTest_Cordic.c
 * \ingroup Cordic
 * \brief Integration tests of the CORDIC coprocessor (Cordic) module on target.
 *
 * The Cordic module runs on the MCU together with the real CORDIC hardware. The tests verify what the
 * unit tests (emulated registers) cannot: the results of the functions compared with the
 * double-precision values over valid ranges (the reference values were computed offline, the toolchain libm
 * does not link in the firmware), in both data formats and in all three array transfer modes (POLL by Cordic_Task,
 * ISR, DMA through Gpdma channels 0 and 1 of GPDMA1).
 *
 * Used resources: CORDIC, GPDMA1 (DMA test only), the CORDIC interrupt (ISR test only). No external wiring,
 * so no fixture feature is needed and the tests run on every board with a CORDIC.
 *
 * \note These tests compile and link for the target but are never run on hardware by the agent that wrote
 *       them (no board access); the behavior on a real board is unverified. The cycle count against the
 *       software library with the DWT counter, mentioned in the README testing chapter, is not part of
 *       these tests (a timing assertion without a measured limit on a board would be a guess).
 * \note STM32H503, STM32H523 and STM32H533 have no CORDIC: on those lines every test is ignored
 *       (TEST_IGNORE_MESSAGE), the generated test runner is not preprocessor aware, so every test case
 *       function below is defined unconditionally.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Cordic_Port.h"                    /* Module under test              */
#include "Stm32.h"                          /* CORDIC instance macro          */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

#if defined(CORDIC)

static void ItCordic_Init                ( cordic_Function_t function,
                                           cordic_Format_t format,
                                           cordic_Scale_t scale,
                                           cordic_XferMode_t xferMode );
static void ItCordic_Complete_Callback   ( void );
static void ItCordic_Fill_Table          ( void );
static void ItCordic_Check_SineTable     ( void );
static double ItCordic_Get_Angle         ( uint32_t index, double step );
static double ItCordic_To_Double         ( cordic_Q31_t qValue );
static void ItCordic_Wait_ArrayEnd       ( void );

#endif /* CORDIC */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** The number pi */
#define IT_CORDIC_PI                        ( 3.14159265358979323846 )

/** Scale factor of the q1.31 format (2^31) and of the q1.15 format (2^15) */
#define IT_CORDIC_Q31_SCALE                 ( 2147483648.0 )
#define IT_CORDIC_Q15_SCALE                 ( 32768.0 )

/** Largest difference to the double-precision result: q1.31 trigonometric functions at the precision 6
 *  (2^-19 residual of the reference manual, with a margin), q1.15 values, logarithm and square root */
#define IT_CORDIC_TOLERANCE_Q31             ( 7.62939453125e-06 )
#define IT_CORDIC_TOLERANCE_Q15             ( 2.44140625e-04 )
#define IT_CORDIC_TOLERANCE_LN_SQRT         ( 6.103515625e-05 )

/** Number of the angles of the single calculation tests, first angle and step (as a fraction of pi) */
#define IT_CORDIC_ANGLE_CNT                 ( 16u )
#define IT_CORDIC_ANGLE_FIRST               ( -1.0 )
#define IT_CORDIC_ANGLE_STEP                ( 0.125 )

/** Step of the angles of the table of the sines (as a fraction of pi) */
#define IT_CORDIC_TABLE_ANGLE_STEP          ( 0.0625 )

/** Number of calculations of the table of the sines and number of the data words per calculation */
#define IT_CORDIC_TABLE_CNT                 ( 32u )
#define IT_CORDIC_WORDS_PER_CALC            ( 2u )

/** Largest number of the Cordic_Task() calls and of the waits for the end of an array request */
#define IT_CORDIC_TASK_CALL_MAX             ( 64u )
#define IT_CORDIC_WAIT_MAX                  ( 20000000u )

/** Arguments of the phase and modulus, the square root and the logarithm tests */
#define IT_CORDIC_X_COORDINATE              ( 0.5f )
#define IT_CORDIC_Y_COORDINATE              ( 0.3f )
#define IT_CORDIC_SQRT_ARGUMENT             ( 0.25f )
#define IT_CORDIC_LN_SCALED_ARGUMENT        ( 0.25f )

/** Double-precision results computed offline: atan2( 0.3, 0.5 ) / pi and the modulus of the vector ( 0.5, 0.3 )
 *  (the float values of the coordinates), the square root of 0.25 and ln( 0.5 ) * 2^-2 */
#define IT_CORDIC_REF_PHASE                 ( 0.172020875203 )
#define IT_CORDIC_REF_MODULUS               ( 0.583095195618 )
#define IT_CORDIC_REF_SQRT                  ( 0.5 )
#define IT_CORDIC_REF_LN                    ( -0.173286795140 )

/* ========================== LOCAL VARIABLES =============================== */

#if defined(CORDIC)

/** Input and output arrays of the array tests (4 byte aligned, two words per calculation) */
static cordic_Q31_t itCordic_In[ IT_CORDIC_TABLE_CNT * IT_CORDIC_WORDS_PER_CALC ];
static cordic_Q31_t itCordic_Out[ IT_CORDIC_TABLE_CNT * IT_CORDIC_WORDS_PER_CALC ];

/** Number of the complete callbacks, set by the callback of the array request */
static volatile uint32_t itCordic_CompleteCnt;

/** Reference sine and cosine of the angles of the single calculation tests (double precision, computed offline) */
static const double itCordic_RefSin[ IT_CORDIC_ANGLE_CNT ] =
{
    0.000000000000, -0.382683432365, -0.707106781187, -0.923879532511,
    -1.000000000000, -0.923879532511, -0.707106781187, -0.382683432365,
    0.000000000000, 0.382683432365, 0.707106781187, 0.923879532511,
    1.000000000000, 0.923879532511, 0.707106781187, 0.382683432365,
};

static const double itCordic_RefCos[ IT_CORDIC_ANGLE_CNT ] =
{
    -1.000000000000, -0.923879532511, -0.707106781187, -0.382683432365,
    0.000000000000, 0.382683432365, 0.707106781187, 0.923879532511,
    1.000000000000, 0.923879532511, 0.707106781187, 0.382683432365,
    0.000000000000, -0.382683432365, -0.707106781187, -0.923879532511,
};

/** Reference sine and cosine of the angles of the table of the sines (double precision, computed offline) */
static const double itCordic_RefSinTable[ IT_CORDIC_TABLE_CNT ] =
{
    0.000000000000, -0.195090322016, -0.382683432365, -0.555570233020,
    -0.707106781187, -0.831469612303, -0.923879532511, -0.980785280403,
    -1.000000000000, -0.980785280403, -0.923879532511, -0.831469612303,
    -0.707106781187, -0.555570233020, -0.382683432365, -0.195090322016,
    0.000000000000, 0.195090322016, 0.382683432365, 0.555570233020,
    0.707106781187, 0.831469612303, 0.923879532511, 0.980785280403,
    1.000000000000, 0.980785280403, 0.923879532511, 0.831469612303,
    0.707106781187, 0.555570233020, 0.382683432365, 0.195090322016,
};

static const double itCordic_RefCosTable[ IT_CORDIC_TABLE_CNT ] =
{
    -1.000000000000, -0.980785280403, -0.923879532511, -0.831469612303,
    -0.707106781187, -0.555570233020, -0.382683432365, -0.195090322016,
    0.000000000000, 0.195090322016, 0.382683432365, 0.555570233020,
    0.707106781187, 0.831469612303, 0.923879532511, 0.980785280403,
    1.000000000000, 0.980785280403, 0.923879532511, 0.831469612303,
    0.707106781187, 0.555570233020, 0.382683432365, 0.195090322016,
    0.000000000000, -0.195090322016, -0.382683432365, -0.555570233020,
    -0.707106781187, -0.831469612303, -0.923879532511, -0.980785280403,
};

#endif /* CORDIC */

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    /* Nothing to reset before a test case (fresh boot, see ItTarget_SystemReset) */
}


void tearDown( void )
{
    /* Peripheral state is cleared by system reset after the test case */
}

/* ============================ HELPER FUNCTIONS ============================= */

#if defined(CORDIC)

/** \brief Initializes the module: the function, the format, the scale, the transfer mode and the callback. */
static void ItCordic_Init( cordic_Function_t function,
                           cordic_Format_t format,
                           cordic_Scale_t scale,
                           cordic_XferMode_t xferMode )
{
    cordic_Config_t config;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_DefaultConfig( &config ) );

    config.Function             = function;
    config.Format               = format;
    config.Scale                = scale;
    config.XferMode             = xferMode;
    config.XferCompleteCallback = ItCordic_Complete_Callback;

    itCordic_CompleteCnt = 0u;

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Init( &config ) );
}


/** \brief Complete callback of the array requests. */
static void ItCordic_Complete_Callback( void )
{
    itCordic_CompleteCnt++;
}


/** \brief Angle of an entry as a fraction of pi: -1.0 and the index times the step. */
static double ItCordic_Get_Angle( uint32_t index, double step )
{
    return ( IT_CORDIC_ANGLE_FIRST + ( (double)index * step ) );
}


/** \brief A q1.31 value as a double precision number. */
static double ItCordic_To_Double( cordic_Q31_t qValue )
{
    return ( (double)qValue / IT_CORDIC_Q31_SCALE );
}


/** \brief Fills the input array of the table of the sines: the angle (q1.31) and the modulus 1.0 per calculation. */
static void ItCordic_Fill_Table( void )
{
    for( uint32_t index = 0u; IT_CORDIC_TABLE_CNT > index; index++ )
    {
        const float  angle = (float)ItCordic_Get_Angle( index, IT_CORDIC_TABLE_ANGLE_STEP );
        cordic_Q31_t qAngle = 0;

        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( angle, &qAngle ) );

        itCordic_In[ ( index * IT_CORDIC_WORDS_PER_CALC ) ]     = qAngle;
        itCordic_In[ ( index * IT_CORDIC_WORDS_PER_CALC ) + 1u ] = CORDIC_Q31_ONE;
        itCordic_Out[ ( index * IT_CORDIC_WORDS_PER_CALC ) ]     = 0;
        itCordic_Out[ ( index * IT_CORDIC_WORDS_PER_CALC ) + 1u ] = 0;
    }
}


/** \brief Compares the output array with the reference sine and cosine. */
static void ItCordic_Check_SineTable( void )
{
    for( uint32_t index = 0u; IT_CORDIC_TABLE_CNT > index; index++ )
    {
        const double sine   = ItCordic_To_Double( itCordic_Out[ ( index * IT_CORDIC_WORDS_PER_CALC ) ] );
        const double cosine = ItCordic_To_Double( itCordic_Out[ ( index * IT_CORDIC_WORDS_PER_CALC ) + 1u ] );

        TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, itCordic_RefSinTable[ index ], sine );
        TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, itCordic_RefCosTable[ index ], cosine );
    }
}


/** \brief Waits (bounded) for the end of the array request: the complete callback was called. */
static void ItCordic_Wait_ArrayEnd( void )
{
    for( uint32_t waitIdx = 0u; IT_CORDIC_WAIT_MAX > waitIdx; waitIdx++ )
    {
        const uint32_t completeCnt = itCordic_CompleteCnt;

        if( 0u != completeCnt )
        {
            break;
        }
        else
        {
            /* The request is still running */
        }
    }
}

#endif /* CORDIC */

/* =============================== TESTS ==================================== */

/**
 * \par Expected results
 * Cordic_Init() with the default configuration succeeds, the state is idle and Cordic_Deinit() succeeds.
 */
void It_Cordic_Init_Nominal_InitializesAndDeinitializes( void )
{
#if defined(CORDIC)
    cordic_State_t state = CORDIC_STATE_BUSY;

    ItCordic_Init( CORDIC_FUNC_COSINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * The sine and the cosine (q1.31, modulus 1.0) of 16 angles from -pi to 7 pi / 8 are within 2^-17 of the
 * double-precision values.
 */
void It_Cordic_Set_Calculate_SineQ31_MatchesLibm( void )
{
#if defined(CORDIC)
    ItCordic_Init( CORDIC_FUNC_SINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_POLL );

    for( uint32_t index = 0u; IT_CORDIC_ANGLE_CNT > index; index++ )
    {
        const double       angle     = ItCordic_Get_Angle( index, IT_CORDIC_ANGLE_STEP );
        cordic_Arguments_t arguments = { .Value1 = 0, .Value2 = CORDIC_Q31_ONE };
        cordic_Results_t   results   = { .Value1 = 0, .Value2 = 0 };

        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( (float)angle, &arguments.Value1 ) );
        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );

        const double sine   = ItCordic_To_Double( results.Value1 );
        const double cosine = ItCordic_To_Double( results.Value2 );

        TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, itCordic_RefSin[ index ], sine );
        TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, itCordic_RefCos[ index ], cosine );
    }

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * The same angles in the q1.15 format give sine and cosine within 2^-12 of the double-precision values.
 */
void It_Cordic_Set_Calculate_SineQ15_MatchesLibm( void )
{
#if defined(CORDIC)
    ItCordic_Init( CORDIC_FUNC_SINE, CORDIC_FORMAT_Q15, CORDIC_SCALE_0, CORDIC_XFER_MODE_POLL );

    for( uint32_t index = 0u; IT_CORDIC_ANGLE_CNT > index; index++ )
    {
        const double       angle     = ItCordic_Get_Angle( index, IT_CORDIC_ANGLE_STEP );
        cordic_Q15_t       qAngle    = 0;
        cordic_Arguments_t arguments = { .Value1 = 0, .Value2 = CORDIC_Q15_ONE };
        cordic_Results_t   results   = { .Value1 = 0, .Value2 = 0 };

        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloatQ15( (float)angle, &qAngle ) );
        arguments.Value1 = qAngle;
        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );

        const double sine   = ( (double)results.Value1 / IT_CORDIC_Q15_SCALE );
        const double cosine = ( (double)results.Value2 / IT_CORDIC_Q15_SCALE );

        TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q15, itCordic_RefSin[ index ], sine );
        TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q15, itCordic_RefCos[ index ], cosine );
    }

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * The phase atan2( y, x ) / pi and the modulus sqrt( x^2 + y^2 ) of the vector ( 0.5, 0.3 ) are within 2^-17 of
 * the double-precision results; the modulus function returns them in the opposite order.
 */
void It_Cordic_Set_Calculate_PhaseAndModulus_MatchesLibm( void )
{
#if defined(CORDIC)
    cordic_Arguments_t arguments = { .Value1 = 0, .Value2 = 0 };
    cordic_Results_t   results   = { .Value1 = 0, .Value2 = 0 };
    ItCordic_Init( CORDIC_FUNC_PHASE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( IT_CORDIC_X_COORDINATE, &arguments.Value1 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( IT_CORDIC_Y_COORDINATE, &arguments.Value2 ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );
    TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, IT_CORDIC_REF_PHASE, ItCordic_To_Double( results.Value1 ) );
    TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, IT_CORDIC_REF_MODULUS, ItCordic_To_Double( results.Value2 ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Function( CORDIC_FUNC_MODULUS ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );
    TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, IT_CORDIC_REF_MODULUS, ItCordic_To_Double( results.Value1 ) );
    TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_Q31, IT_CORDIC_REF_PHASE, ItCordic_To_Double( results.Value2 ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * The square root of 0.25 (scale 0) is 0.5 and the natural logarithm of 0.5 (scale 1, the argument is 0.5 * 2^-1,
 * the result is ln( 0.5 ) * 2^-2) matches the double-precision value within 2^-14.
 */
void It_Cordic_Set_Calculate_SquareRootAndLogarithm_MatchesLibm( void )
{
#if defined(CORDIC)
    cordic_Arguments_t arguments = { .Value1 = 0, .Value2 = 0 };
    cordic_Results_t   results   = { .Value1 = 0, .Value2 = 0 };

    ItCordic_Init( CORDIC_FUNC_SQUARE_ROOT, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_POLL );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( IT_CORDIC_SQRT_ARGUMENT, &arguments.Value1 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );
    TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_LN_SQRT, IT_CORDIC_REF_SQRT, ItCordic_To_Double( results.Value1 ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Function( CORDIC_FUNC_NATURAL_LOGARITHM ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Scale( CORDIC_SCALE_1 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_FromFloat( IT_CORDIC_LN_SCALED_ARGUMENT, &arguments.Value1 ) );
    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_Calculate( &arguments, &results ) );
    TEST_ASSERT_DOUBLE_WITHIN( IT_CORDIC_TOLERANCE_LN_SQRT, IT_CORDIC_REF_LN, ItCordic_To_Double( results.Value1 ) );

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * A table of 32 sines and cosines in the POLL mode: Cordic_Task() finishes the request within a few calls, the
 * complete callback is called once, the table matches the double-precision values within 2^-17.
 */
void It_Cordic_Set_CalculateArray_Poll_TableOfSines( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = { .Input  = itCordic_In,
                                            .Output = itCordic_Out,
                                            .Count  = IT_CORDIC_TABLE_CNT };
    cordic_State_t              state   = CORDIC_STATE_BUSY;

    ItCordic_Init( CORDIC_FUNC_SINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_POLL );
    ItCordic_Fill_Table();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );

    for( uint32_t callIdx = 0u; IT_CORDIC_TASK_CALL_MAX > callIdx; callIdx++ )
    {
        Cordic_Task();

        TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );

        if( CORDIC_STATE_BUSY != state )
        {
            break;
        }
        else
        {
            /* The request is still running */
        }
    }

    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_UINT32( 1u, itCordic_CompleteCnt );
    ItCordic_Check_SineTable();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * The same table in the ISR mode: the CORDIC interrupt finishes the request (bounded wait for the complete
 * callback), the table matches the double-precision values within 2^-17.
 */
void It_Cordic_Set_CalculateArray_Isr_TableOfSines( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = { .Input  = itCordic_In,
                                            .Output = itCordic_Out,
                                            .Count  = IT_CORDIC_TABLE_CNT };
    cordic_State_t              state   = CORDIC_STATE_BUSY;

    ItCordic_Init( CORDIC_FUNC_SINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_ISR );
    ItCordic_Fill_Table();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    ItCordic_Wait_ArrayEnd();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_UINT32( 1u, itCordic_CompleteCnt );
    ItCordic_Check_SineTable();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}


/**
 * \par Expected results
 * The same table in the DMA mode (GPDMA1 channel 0 feeds the arguments, channel 1 takes the results): the
 * request ends without CPU load (bounded wait for the complete callback), the table matches the
 * double-precision values within 2^-17.
 */
void It_Cordic_Set_CalculateArray_Dma_TableOfSines( void )
{
#if defined(CORDIC)
    const cordic_ArrayRequest_t request = { .Input  = itCordic_In,
                                            .Output = itCordic_Out,
                                            .Count  = IT_CORDIC_TABLE_CNT };
    cordic_State_t              state   = CORDIC_STATE_BUSY;

    ItCordic_Init( CORDIC_FUNC_SINE, CORDIC_FORMAT_Q31, CORDIC_SCALE_0, CORDIC_XFER_MODE_DMA );
    ItCordic_Fill_Table();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Set_CalculateArray( &request ) );
    ItCordic_Wait_ArrayEnd();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Get_State( &state ) );
    TEST_ASSERT_EQUAL( CORDIC_STATE_IDLE, state );
    TEST_ASSERT_EQUAL_UINT32( 1u, itCordic_CompleteCnt );
    ItCordic_Check_SineTable();

    TEST_ASSERT_EQUAL( CORDIC_REQUEST_OK, Cordic_Deinit() );
#else
    TEST_IGNORE_MESSAGE( "No CORDIC on this device line" );
#endif /* CORDIC */
}
