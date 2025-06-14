#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.20)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(BSICS-SVR C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Uncomment for hardware floating point.
#   Define according to Cortex core:
#   ARM_MATH_CM7, ARM_MATH_CM4, ARM_MATH_CM3, ARM_MATH_CM0PLUS or ARM_MATH_CM0
add_compile_definitions(ARM_MATH_CM7;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
#   Define according to Cortex core
#   -mfpu specifies floating-point hardware (or hardware emulation) availability.
#   Permissible names are: ‘auto’, ‘vfpv2’, ‘vfpv3’, ‘vfpv3-fp16’, ‘vfpv3-d16’,
#   ‘vfpv3-d16-fp16’, ‘vfpv3xd’, ‘vfpv3xd-fp16’, ‘neon-vfpv3’, ‘neon-fp16’,
#   ‘vfpv4’, ‘vfpv4-d16’, ‘fpv4-sp-d16’, ‘neon-vfpv4’,
#   ‘fpv5-d16’, ‘fpv5-sp-d16’, ‘fp-armv8’, ‘neon-fp-armv8’ and ‘crypto-neon-fp-armv8’.
#   Note that ‘neon’ is an alias for ‘neon-vfpv3’ and ‘vfp’ is an alias for ‘vfpv2’.
#   Examples: CM4: fpu=fpv4-sp-d16, CM7: fpu=fpv5-sp-d16
#   see for more: https://gcc.gnu.org/onlinedocs/gcc/ARM-Options.html
add_compile_options(-mfloat-abi=hard -mfpu=fpv5-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv5-sp-d16)

# Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to explicitly deactivate Run-Time Type Information
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "NonOptimized")
    message(STATUS "No optimizations")
    add_compile_options(-O0 -g3)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Inc Middlewares/Third_Party/LwIP/src/include Middlewares/Third_Party/LwIP/system Drivers/STM32F7xx_HAL_Driver/Inc Drivers/STM32F7xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 Middlewares/Third_Party/LwIP/src/include/netif/ppp Drivers/CMSIS/Device/ST/STM32F7xx/Include Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS Middlewares/Third_Party/LwIP/src/include/lwip Middlewares/Third_Party/LwIP/src/include/lwip/apps Middlewares/Third_Party/LwIP/src/include/lwip/priv Middlewares/Third_Party/LwIP/src/include/lwip/prot Middlewares/Third_Party/LwIP/src/include/netif Middlewares/Third_Party/LwIP/src/include/compat/posix Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa Middlewares/Third_Party/LwIP/system/arch Drivers/CMSIS/Include Middlewares/Third_Party/LwIP/src/include/compat/posix/net Middlewares/Third_Party/LwIP/src/include/compat/posix/sys Middlewares/Third_Party/LwIP/src/include/compat/stdc)
include_directories(Applications/SCPI_Server/Inc Middlewares/Third_Party/libscpi/inc) # added
include_directories(Drivers/HW/ADG715/Lib Drivers/HW/PCA953x/Lib Drivers/HW/MCP342x/Lib Drivers/HW/MCP9808/Lib Drivers/HW/MP8862/Lib Drivers/HW/SSD1306/Lib) # added

add_definitions(-DUSE_HAL_DRIVER -DSTM32F767xx)

set(EXCLUDE_DIRS "example/" "Middlewares/Third_Party/libscpi/src/" "Middlewares/Third_Party/libscpi/test/") # added

file(GLOB_RECURSE SOURCES "startup/*.*" "Middlewares/*.*" "Drivers/*.*" "Src/*.*" "Inc/*.*")
file(GLOB_RECURSE SOURCES ${SOURCES} "Applications/SCPI_Server/Src/*.*") # added

# see https://stackoverflow.com/questions/24491129/excluding-directory-somewhere-in-file-structure-from-cmake-sourcefile-list
foreach (TMP_PATH ${SOURCES})  # added
    foreach (EXCLUDE_DIR ${EXCLUDE_DIRS}) # added
        string (FIND ${TMP_PATH} ${EXCLUDE_DIR} EXCLUDE_DIR_FOUND)  # added
        if (NOT ${EXCLUDE_DIR_FOUND} EQUAL -1)  # added
            list (REMOVE_ITEM SOURCES ${TMP_PATH})  # added
        endif ()  # added
    endforeach(EXCLUDE_DIR)
endforeach(TMP_PATH)  # added


set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F767ZITx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
# add_link_options(-specs=nano.specs -u _printf_float)
add_link_options(-T ${LINKER_SCRIPT})



add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})
target_link_libraries(${PROJECT_NAME}.elf ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/libscpi/dist/libscpi.a) # added


set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
