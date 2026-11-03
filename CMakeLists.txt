cmake_minimum_required(VERSION 3.16)
project(freertos_canbus_modbus_gateway VERSION 1.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Werror -Wpedantic -Wshadow -ffunction-sections -fdata-sections)
endif()

add_library(gateway_core STATIC
    src/modbus_rtu.c
    src/can_protocol.c
    src/watchdog_supervisor.c
)

target_include_directories(gateway_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

enable_testing()

add_executable(test_gateway tests/test_gateway.c)
target_link_libraries(test_gateway PRIVATE gateway_core)
add_test(NAME GatewayUnitTestSuite COMMAND test_gateway)
