# arm-none-eabi GCC toolchain, installed under ~/.local (no sudo, no system packages).
# Point TOOLCHAIN_ROOT elsewhere to use another GCC.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT DEFINED TOOLCHAIN_ROOT)
  set(TOOLCHAIN_ROOT "$ENV{HOME}/.local/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1")
endif()

set(CMAKE_C_COMPILER "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-objcopy")
set(CMAKE_SIZE "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-size")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
