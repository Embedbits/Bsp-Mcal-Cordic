#===================== Integration tests configuration ========================#
# Integration tests of Cordic module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Cordic.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Cordic.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")

    # STM32H7R / STM32H7S (NUCLEO-H7S3L8, 64 KB flash)
    # The firmware needs 102 % of the 64 KB flash at -O0 and does not link, optimization is required.
    if(CMAKE_C_FLAGS_INTEGRATIONTEST MATCHES "-O0")
        message(WARNING "Integration test IT_Cordic of STM32H7R / STM32H7S does not fit into 64 KB flash at -O0 - "
                        "configure with -DINTEGRATION_TEST_OPTIMIZATION=-Os")
    endif()

endif()

IntegrationTesting_Add_Test(
    NAME            Cordic
    TEST_SOURCE     ItTest_Cordic.c
    SOURCES         ItTarget_Cordic.c   # Target interface of the test set
    LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                    Nvic_Lib            # Fault handlers, system reset (ItTarget)
                    Rcc_Lib             # Peripheral clocks
                    Dma_Lib             # DMA transfer mode (classic STM32H7 lines)
                    Gpdma_Lib           # DMA transfer mode (STM32H7R / STM32H7S)
                    Cordic_Lib
    TIMEOUT         60
)
