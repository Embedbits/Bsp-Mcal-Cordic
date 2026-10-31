#======================== Unit tests configuration ============================#
# Unit tests of Cordic module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(CORDIC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR "${CORDIC_DIR}/.." ABSOLUTE)

# CORDIC_CSR.RRDY and the result are produced by the hardware after the arguments are written,
# which plain RegMem does not emulate: the tests that run a calculation activate the HW model
# UtCordic_HwModel (see the test source). RCC, NVIC and Gpdma are mocked by CMock.
UnitTesting_Add_Test(
    NAME            Cordic
    TEST_SOURCE     Test_Cordic.c
    UUT_SOURCES     ${CORDIC_DIR}/Cordic.c
                    ${CORDIC_DIR}/Cordic_Reg.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpdma/Gpdma_Port.h
    INCLUDE_DIRS    ${CORDIC_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpdma
    LINK_LIBS       ut_RegMem                           # Registers in host memory
                    ut_CmsisHost                        # CMSIS core intrinsics on host
    FORCE_INCLUDES  ${UNIT_TESTING_CMSIS_HOST_HEADER}   # Replaces cmsis_gcc.h
    HOST_LIBS       Ral_Lib                             # ST LL compiled for host
    COVERAGE_EXCLUDE ".*/Ral/.*"
    RUN_SERIAL
)
