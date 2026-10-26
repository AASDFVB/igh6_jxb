cmake_minimum_required(VERSION 3.20)

# Configure with:
#   cmake --preset Debug          (see CMakePresets.json)
# or:
#   cmake -B build -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

project(igh6_jxb LANGUAGES C ASM)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS} -fdata-sections -ffunction-sections -Wall -Wextra")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${MCU_FLAGS} \
  -T${CMAKE_SOURCE_DIR}/STM32F407IGHX_FLASH.ld \
  --specs=nano.specs -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map")

file(GLOB CORE_SOURCES   ${CMAKE_SOURCE_DIR}/Src/*.c)
file(GLOB APP_SOURCES    ${CMAKE_SOURCE_DIR}/app/*.c)
file(GLOB HAL_SOURCES    ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/*.c)
file(GLOB USB_SOURCES
  ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/*.c
  ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/*.c)
file(GLOB STARTUP_SOURCES ${CMAKE_SOURCE_DIR}/Startup/*.s)

add_executable(${PROJECT_NAME}
  ${CORE_SOURCES} ${APP_SOURCES} ${HAL_SOURCES} ${USB_SOURCES} ${STARTUP_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_SOURCE_DIR}/Inc
  ${CMAKE_SOURCE_DIR}/app
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
  ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_HAL_DRIVER
  STM32F407xx)

# 让产物叫 igh6_jxb.elf（cortex-debug / openocd 都按这个后缀找）
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
  COMMENT "objcopy -> ${PROJECT_NAME}.hex/.bin")
