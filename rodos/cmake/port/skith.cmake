set(board skith)
set(OSC_CLK 16000000)
set(RODOS_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(linker_script ${RODOS_DIR}/src/bare-metal/stm32f4/scripts/stm32_flash.ld)
set(mcu_flag STM32F40_41xxx)

include(${CMAKE_CURRENT_LIST_DIR}/stm32f4.cmake)
