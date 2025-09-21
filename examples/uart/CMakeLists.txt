cmake_minimum_required(VERSION 3.15)

# Set toolchain before project()
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Set toolchain paths
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)
set(CMAKE_DEBUGGER arm-none-eabi-gdb)

# Prevent CMake from trying to test the compiler
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Project definition
project(freertos_uart_pca10056 C ASM)

# Define paths
set(SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(PROJ_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Define target
set(TARGET nrf52840_xxaa)

# Linker script
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/freertos_uart_gcc_nrf52.ld)

# Source files (FreeRTOS + UART combined)
set(SRC_FILES
    # Startup and system files (nRF52840)
    ${SDK_ROOT}/modules/nrfx/mdk/gcc_startup_nrf52840.S
    ${SDK_ROOT}/modules/nrfx/mdk/system_nrf52840.c
    
    # Board support and utilities
    ${SDK_ROOT}/components/boards/boards.c
    ${SDK_ROOT}/components/libraries/util/app_error.c
    ${SDK_ROOT}/components/libraries/util/app_error_handler_gcc.c
    ${SDK_ROOT}/components/libraries/util/app_error_weak.c
    ${SDK_ROOT}/components/libraries/util/app_util_platform.c
    ${SDK_ROOT}/components/libraries/util/nrf_assert.c
    ${SDK_ROOT}/components/libraries/atomic/nrf_atomic.c
    ${SDK_ROOT}/components/libraries/balloc/nrf_balloc.c
    ${SDK_ROOT}/components/libraries/memobj/nrf_memobj.c
    ${SDK_ROOT}/components/libraries/ringbuf/nrf_ringbuf.c
    ${SDK_ROOT}/components/libraries/strerror/nrf_strerror.c
    ${SDK_ROOT}/components/libraries/bsp/bsp.c
    
    # FreeRTOS source files
    ${SDK_ROOT}/external/freertos/source/croutine.c
    ${SDK_ROOT}/external/freertos/source/event_groups.c
    ${SDK_ROOT}/external/freertos/source/portable/MemMang/heap_1.c
    ${SDK_ROOT}/external/freertos/source/list.c
    ${SDK_ROOT}/external/freertos/portable/GCC/nrf52/port.c
    ${SDK_ROOT}/external/freertos/portable/CMSIS/nrf52/port_cmsis.c
    ${SDK_ROOT}/external/freertos/portable/CMSIS/nrf52/port_cmsis_systick.c
    ${SDK_ROOT}/external/freertos/source/queue.c
    ${SDK_ROOT}/external/freertos/source/stream_buffer.c
    ${SDK_ROOT}/external/freertos/source/tasks.c
    ${SDK_ROOT}/external/freertos/source/timers.c
    ${SDK_ROOT}/components/libraries/timer/app_timer_freertos.c
    
    # UART source files
    ${SDK_ROOT}/components/libraries/fifo/app_fifo.c
    ${SDK_ROOT}/components/libraries/uart/app_uart_fifo.c
    ${SDK_ROOT}/components/libraries/uart/retarget.c
    ${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_uart.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uart.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_uarte.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/prs/nrfx_prs.c
    
    # Clock and power management
    ${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_clock.c
    ${SDK_ROOT}/modules/nrfx/soc/nrfx_atomic.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_clock.c
    ${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c
    ${SDK_ROOT}/components/drivers_nrf/nrf_soc_nosd/nrf_nvic.c
    ${SDK_ROOT}/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c
    
    # Button support
    ${SDK_ROOT}/components/libraries/button/app_button.c
    
    # Logging and printf support
    ${SDK_ROOT}/components/libraries/log/src/nrf_log_frontend.c
    ${SDK_ROOT}/components/libraries/log/src/nrf_log_str_formatter.c
    ${SDK_ROOT}/external/fprintf/nrf_fprintf.c
    ${SDK_ROOT}/external/fprintf/nrf_fprintf_format.c
    
    # Main application
    ${PROJ_DIR}/main.c
)

# Include directories (combined from both projects)
set(INC_FOLDERS
    ${PROJ_DIR}/config
    ${SDK_ROOT}/components
    ${SDK_ROOT}/modules/nrfx/mdk
    ${PROJ_DIR}
    
    # FreeRTOS includes
    ${SDK_ROOT}/external/freertos/source/include
    ${SDK_ROOT}/external/freertos/config
    ${SDK_ROOT}/external/freertos/portable/CMSIS/nrf52
    ${SDK_ROOT}/external/freertos/portable/GCC/nrf52
    
    # UART includes
    ${SDK_ROOT}/components/libraries/uart
    ${SDK_ROOT}/components/libraries/fifo
    ${SDK_ROOT}/integration/nrfx/legacy
    
    # Common library includes
    ${SDK_ROOT}/components/libraries/timer
    ${SDK_ROOT}/components/libraries/strerror
    ${SDK_ROOT}/components/toolchain/cmsis/include
    ${SDK_ROOT}/components/libraries/util
    ${SDK_ROOT}/components/libraries/delay
    ${SDK_ROOT}/components/libraries/balloc
    ${SDK_ROOT}/components/libraries/ringbuf
    ${SDK_ROOT}/modules/nrfx/hal
    ${SDK_ROOT}/components/libraries/bsp
    ${SDK_ROOT}/components/libraries/log
    ${SDK_ROOT}/components/libraries/button
    ${SDK_ROOT}/modules/nrfx
    ${SDK_ROOT}/components/libraries/experimental_section_vars
    ${SDK_ROOT}/integration/nrfx
    ${SDK_ROOT}/components/drivers_nrf/nrf_soc_nosd
    ${SDK_ROOT}/components/libraries/atomic
    ${SDK_ROOT}/components/boards
    ${SDK_ROOT}/components/libraries/memobj
    ${SDK_ROOT}/modules/nrfx/drivers/include
    ${SDK_ROOT}/external/fprintf
    ${SDK_ROOT}/components/libraries/log/src
)

# Create executable
add_executable(${TARGET} ${SRC_FILES})

# Set target properties
target_include_directories(${TARGET} PRIVATE ${INC_FOLDERS})

# Compiler definitions (combined from both projects)
target_compile_definitions(${TARGET} PRIVATE
    -DBOARD_PCA10056
    -DCONFIG_GPIO_AS_PINRESET
    -DFLOAT_ABI_HARD
    -DFREERTOS
    -DNRF52840_XXAA
    -D__HEAP_SIZE=8192
    -D__STACK_SIZE=8192
    
    # POWER module configuration
    -DNRFX_POWER_ENABLED=1
    -DNRFX_POWER_CONFIG_IRQ_PRIORITY=6
    
    # CLOCK module configuration
    -DNRFX_CLOCK_ENABLED=1
    -DNRFX_CLOCK_CONFIG_IRQ_PRIORITY=6
    -DNRFX_CLOCK_CONFIG_LF_SRC=1          # 1=XTAL (external crystal) or 0=RC (internal)
    
    # Legacy clock support
    -DNRF_CLOCK_ENABLED=1
    -DCLOCK_CONFIG_LF_SRC=1               # Must match with NRFX_CLOCK_CONFIG_LF_SRC
    -DCLOCK_CONFIG_IRQ_PRIORITY=6
    
    # UART specific defines
    -DAPP_UART_ENABLED=1
    -DAPP_FIFO_ENABLED=1
    -DAPP_UART_DRIVER_INSTANCE=0
    -DUART_DEFAULT_CONFIG_PARITY=0
)

# C Compiler flags
target_compile_options(${TARGET} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-std=c99>
    -mcpu=cortex-m4
    -mthumb
    -mabi=aapcs
    -Wall
    -Werror
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -ffunction-sections
    -fdata-sections
    -fno-strict-aliasing
    -fno-builtin
    -fshort-enums
    -O3
    -g3
)

# ASM Compiler flags
target_compile_options(${TARGET} PRIVATE
    $<$<COMPILE_LANGUAGE:ASM>:
        -mcpu=cortex-m4
        -mthumb
        -mabi=aapcs
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -g3
    >
)

# Linker flags
target_link_options(${TARGET} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mabi=aapcs
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -Wl,--gc-sections
    --specs=nano.specs
    -L${SDK_ROOT}/modules/nrfx/mdk
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.map
    -O3
    -g3
)

# Link libraries
target_link_libraries(${TARGET} PRIVATE
    -lc
    -lnosys
    -lm
)

# Post-build commands to create hex and bin files
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
    COMMENT "Building ${TARGET}.hex and ${TARGET}.bin"
)

# Print executable size
add_custom_command(TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}
    COMMENT "Executable size:"
)

# Custom targets for flashing and erasing
add_custom_target(flash
    DEPENDS ${TARGET}
    COMMAND nrfjprog -f nrf52 --program ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}.hex --sectorerase
    COMMAND nrfjprog -f nrf52 --reset
    COMMENT "Flashing ${TARGET}.hex to nRF52840"
)

add_custom_target(erase
    COMMAND nrfjprog -f nrf52 --eraseall
    COMMENT "Erasing nRF52840 flash"
)

# Custom target for SDK config
add_custom_target(sdk_config
    COMMAND java -jar ${SDK_ROOT}/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar ${CMAKE_CURRENT_SOURCE_DIR}/config/sdk_config.h
    COMMENT "Opening SDK configuration wizard"
)

# Set output directory
set_target_properties(${TARGET} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_NAME ${TARGET}
)

# Ensure linker script exists
if(NOT EXISTS ${LINKER_SCRIPT})
    message(FATAL_ERROR "Linker script not found: ${LINKER_SCRIPT}")
endif()
