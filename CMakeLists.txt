set(TOOLCHAIN_PREFIX "/usr/local")
set(CMAKE_TOOLCHAIN_FILE "/Users/fuehner/git/stm32-cmake/cmake/gcc_stm32.cmake")
set(STM32Cube_DIR "/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0")
set(STM32_CHIP STM32F103CBT6)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "cmake" "../cmake" "../../cmake")
#add_definitions(-DHSE_VALUE='((uint32_t)12000000)')

PROJECT(move32)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
ENABLE_LANGUAGE(ASM)

FIND_PACKAGE(CMSIS REQUIRED)
FIND_PACKAGE(STM32HAL COMPONENTS adc gpio i2c tim uart REQUIRED)

#message("CMSIS Sources: ${CMSIS_SOURCES}")
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s

#message("STHAL Sources: ${STM32HAL_SOURCES}")
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c;
#/Users/fuehner/STM32Cube/Repository/STM32Cube_FW_F1_V1.4.0/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_INCLUDE_DIRS}
    ${STM32HAL_INCLUDE_DIR}
    "include"
)

file(GLOB PROJECT_SOURCES "src/*.h" "src/*.hpp" "src/*.c" "src/*.cpp")

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES} ${CMSIS_SOURCES} ${STM32HAL_SOURCES})

FUNCTION(STM32_ADD_TARGET_LINK_FLAGS TARGET ADD_FLAGS)
    GET_TARGET_PROPERTY(TARGET_LD_FLAGS ${TARGET} LINK_FLAGS)
    IF(TARGET_LD_FLAGS)
        SET(TARGET_LD_FLAGS "${TARGET_LD_FLAGS} ${ADD_FLAGS}")
    ELSE()
        SET(TARGET_LD_FLAGS ${ADD_FLAGS})
    ENDIF()
    SET_TARGET_PROPERTIES(${TARGET} PROPERTIES LINK_FLAGS ${TARGET_LD_FLAGS})
ENDFUNCTION(STM32_ADD_TARGET_LINK_FLAGS)

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_TARGET_LINK_FLAGS(${CMAKE_PROJECT_NAME} "-specs=nano.specs -lc -lm") #-lnosys
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})
