/**
 * \author Mr.Nobody
 * \file Cordic_Reg.c
 * \ingroup Cordic
 * \brief Cordic module Reg component source file
 *
 * This component contains the functionality for register operations of the
 * CORDIC peripheral: generic accessors with a bounds checked register
 * identification, the access to the data registers (a read of CORDIC_RDATA
 * has a side effect, it releases the result, so it has a function of its own),
 * the addresses of the data registers for the DMA and the wait for the result.
 *
 * STM32H503, STM32H523 and STM32H533 have no CORDIC peripheral (no CORDIC
 * instance in their CMSIS header): the register access is compiled only where
 * the instance exists and the public functions of the module do not use it on
 * the other lines.
 *
 */

/* ============================= INCLUDES =================================== */
#include "Cordic_Reg.h"                     /* Module Reg component header    */
#include "Stm32.h"                          /* Device registers definition    */
/* ========================== PRIVATE DECLARATIONS =========================== */

#if defined(CORDIC)

/** \brief Count of read-back iterations of the wait for the result */
#define CORDIC_REG_WAIT_TIMEOUT_RAW      ( 10000u )

/** \brief Register value without any bit set */
#define CORDIC_REG_BITS_CLEAR            ( 0u )

/** \brief Physical address of a CORDIC register, referenced through its enumeration id */
typedef struct
{
    cordic_RegId_t      RegId;   /**< Peripheral register ID      */
    volatile uint32_t * RegAddr; /**< Peripheral register address */
}   cordic_RegList_t;

/* =========================== LOCAL VARIABLES ============================== */

/** \brief CORDIC register addresses, referenced through their enumeration id */
static const cordic_RegList_t cordic_RegList[] =
{
    { .RegId = CORDIC_REG_CSR  , .RegAddr = &( CORDIC->CSR   ) },
    { .RegId = CORDIC_REG_WDATA, .RegAddr = &( CORDIC->WDATA ) },
    { .RegId = CORDIC_REG_RDATA, .RegAddr = &( CORDIC->RDATA ) },
};

_Static_assert( ( sizeof( cordic_RegList ) / sizeof( cordic_RegList_t ) ) == CORDIC_REG_CNT,
                "Cordic_Reg: cordic_RegList has incorrect size." );

/* ========================= EXPORTED FUNCTIONS =============================== */

/**
 * \brief Sets the register bit(s) with the given mask.
 *
 * \param regId   [in] : CORDIC register ID
 * \param bitMask [in] : Mask to apply to the register
 */
void Cordic_Reg_Set_RegBit( cordic_RegId_t regId, uint32_t bitMask )
{
    if( CORDIC_REG_CNT > regId )
    {
        *cordic_RegList[ regId ].RegAddr |= bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Resets the register bit(s) with the given mask.
 *
 * \param regId   [in] : CORDIC register ID
 * \param bitMask [in] : Mask to apply to the register
 */
void Cordic_Reg_Reset_RegBit( cordic_RegId_t regId, uint32_t bitMask )
{
    if( CORDIC_REG_CNT > regId )
    {
        *cordic_RegList[ regId ].RegAddr &= ~bitMask;
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register bit(s) value with the given mask.
 *
 * \param regId   [in] : CORDIC register ID
 * \param bitMask [in] : Mask to apply to the register
 *
 * \return Register bit value with applied mask, CORDIC_REG_BITS_CLEAR for an invalid register ID.
 */
uint32_t Cordic_Reg_Get_RegBit( cordic_RegId_t regId, uint32_t bitMask )
{
    uint32_t retValue = CORDIC_REG_BITS_CLEAR;

    if( CORDIC_REG_CNT > regId )
    {
        retValue = ( *cordic_RegList[ regId ].RegAddr & bitMask );
    }
    else
    {
        /* Invalid register ID, no bit is set */
        retValue = CORDIC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Updates the register value with the given mask and value, the other bits are preserved.
 *
 * \param regId    [in] : CORDIC register ID
 * \param regMask  [in] : Mask of the field written
 * \param regValue [in] : Value to write into the field (already shifted into position)
 */
void Cordic_Reg_Set_RegVal( cordic_RegId_t regId, uint32_t regMask, uint32_t regValue )
{
    if( CORDIC_REG_CNT > regId )
    {
        const uint32_t regCurrent = *cordic_RegList[ regId ].RegAddr;

        *cordic_RegList[ regId ].RegAddr = ( ( regCurrent & ~regMask ) | ( regValue & regMask ) );
    }
    else
    {
        /* Invalid register ID, no action required */
    }
}


/**
 * \brief Gets the register value with the given mask.
 *
 * \param regId   [in] : CORDIC register ID
 * \param regMask [in] : Mask of the field read
 *
 * \return Register field value with applied mask, CORDIC_REG_BITS_CLEAR for an invalid register ID.
 */
uint32_t Cordic_Reg_Get_RegVal( cordic_RegId_t regId, uint32_t regMask )
{
    uint32_t retValue = CORDIC_REG_BITS_CLEAR;

    if( CORDIC_REG_CNT > regId )
    {
        retValue = ( *cordic_RegList[ regId ].RegAddr & regMask );
    }
    else
    {
        /* Invalid register ID, no value is returned */
        retValue = CORDIC_REG_BITS_CLEAR;
    }

    return ( retValue );
}


/**
 * \brief Writes one data word to the argument register (CORDIC_WDATA).
 *
 * When the expected number of arguments has been written the calculation starts.
 *
 * \param argData [in] : Argument data word (one q1.31 value or two packed q1.15 values)
 */
void Cordic_Reg_Set_Data( uint32_t argData )
{
    *cordic_RegList[ CORDIC_REG_WDATA ].RegAddr = argData;
}


/**
 * \brief Reads one data word of the result register (CORDIC_RDATA).
 *
 * The read releases the result: when the expected number of reads was made the result
 * ready flag is cleared and a pending calculation starts. The register is read exactly once.
 *
 * \return Result data word (one q1.31 value or two packed q1.15 values).
 */
uint32_t Cordic_Reg_Get_Data( void )
{
    return ( *cordic_RegList[ CORDIC_REG_RDATA ].RegAddr );
}


/**
 * \brief Gets the address of the argument register for the DMA (CORDIC_WDATA).
 *
 * \return Bus address of CORDIC_WDATA.
 */
uint32_t Cordic_Reg_Get_WdataAddr( void )
{
    return ( (uint32_t)(uintptr_t)cordic_RegList[ CORDIC_REG_WDATA ].RegAddr );
}


/**
 * \brief Gets the address of the result register for the DMA (CORDIC_RDATA).
 *
 * \return Bus address of CORDIC_RDATA.
 */
uint32_t Cordic_Reg_Get_RdataAddr( void )
{
    return ( (uint32_t)(uintptr_t)cordic_RegList[ CORDIC_REG_RDATA ].RegAddr );
}


/**
 * \brief Gets the result ready flag (CORDIC_CSR.RRDY).
 *
 * \return CORDIC_FLAG_ACTIVE when CORDIC_RDATA holds a new result, CORDIC_FLAG_INACTIVE otherwise.
 */
cordic_FlagState_t Cordic_Reg_Get_ResultReady( void )
{
    cordic_FlagState_t retState = CORDIC_FLAG_INACTIVE;
    const uint32_t     readyBit = Cordic_Reg_Get_RegBit( CORDIC_REG_CSR, CORDIC_CSR_RRDY );

    if( CORDIC_REG_BITS_CLEAR != readyBit )
    {
        retState = CORDIC_FLAG_ACTIVE;
    }
    else
    {
        retState = CORDIC_FLAG_INACTIVE;
    }

    return ( retState );
}


/**
 * \brief Waits for the result ready flag (CORDIC_CSR.RRDY), a bounded loop on the register state.
 *
 * \return CORDIC_REQUEST_OK when the flag was set in time, CORDIC_REQUEST_ERROR on the timeout.
 */
cordic_RequestState_t Cordic_Reg_Wait_ResultReady( void )
{
    cordic_RequestState_t retState = CORDIC_REQUEST_ERROR;

    for( uint32_t iterationCnt = 0u; CORDIC_REG_WAIT_TIMEOUT_RAW > iterationCnt; iterationCnt++ )
    {
        const cordic_FlagState_t readyState = Cordic_Reg_Get_ResultReady();

        if( CORDIC_FLAG_ACTIVE == readyState )
        {
            retState = CORDIC_REQUEST_OK;
            break;
        }
        else
        {
            retState = CORDIC_REQUEST_ERROR;
        }
    }

    return ( retState );
}

#endif /* CORDIC */
