cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# SDK paths come from build.ps1 or the environment; project may live outside hpm_apps.
# 编码器移植阶段默认只编译应用和 VOFA；保留的电机/core 接口补齐后再开启。
option(MOTOR_CONTROL_BUILD_MOTOR "Build retained motor initialization and core extensions" OFF)
set(CONFIG_A_LIBMOTORCTRL ${MOTOR_CONTROL_BUILD_MOTOR})
if(MOTOR_CONTROL_BUILD_MOTOR)
    set(MOTOR_EXTENSION_ENABLED 1)
else()
    set(MOTOR_EXTENSION_ENABLED 0)
endif()
set(CONFIG_A_VOFA_DEBUG 1)
set(CONFIG_VOFA_DEBUG_HPM_UART 1)
set(CONFIG_HPM_ADC 1)
set(HPM_SDK_NO_NANO_SPECS 1)
set(RV_ABI "ilp32f")
set(RV_ARCH "rv32imafc")
find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
get_filename_component(
    HPM_APPS_BASE
    "${HPM_SDK_BASE}/../hpm_apps"
    ABSOLUTE
)
if(NOT EXISTS "${HPM_APPS_BASE}/CMakeLists.txt")
    message(FATAL_ERROR "HPM_APPS_BASE must point to the complete v1.9 hpm_apps")
endif()
sdk_compile_definitions(-DCONFIG_USE_HPM6E00=1 -DCONFIG_USE_FUNCTION=${MOTOR_EXTENSION_ENABLED})
sdk_compile_definitions(-DBOARD_SHOW_CLOCK=0 -DBOARD_SHOW_BANNER=0)
# Use the SDK's normal flash_xip linker/startup, not the OTA application partition.
add_subdirectory("${HPM_APPS_BASE}" "${CMAKE_BINARY_DIR}/hpm_apps")

project(motor_control)
sdk_inc(../motor/inc ../core/inc inc)

sdk_app_src(src/main.c src/motor_vofa.c src/encoder.c
    src/adc.c
    src/pwm.c src/pwm_test.c)

# 仅延后编译，不删除这些初始化和扩展源码。
if(MOTOR_CONTROL_BUILD_MOTOR)
    sdk_app_src(../motor/src/pmsm_init.c)
    sdk_app_src(../core/src/pmsm_currentctrl_3P3Z.c
        ../core/src/motor_params_detection.c ../core/src/pmsm_detection.c
        ../core/src/pmsm_interia.c)
    sdk_link_libraries("${HPM_APPS_BASE}/middleware/hpm_motor/lib/libhpm_motor.a"
        "${HPM_APPS_BASE}/middleware/hpm_motor/lib/libhpm_motor_inertia_identification.a")
endif()
sdk_ld_options("-lm")
sdk_ses_opt_debug_connection("J-Link")
sdk_ses_opt_debug_jlink_speed(1000)
generate_ses_project()
# Run for every CMake entry point, including CMake Tools and Ninja regeneration.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/../organize_ses.py")
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/../organize_ses.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
        "${CMAKE_CURRENT_BINARY_DIR}/segger_embedded_studio/${CMAKE_PROJECT_NAME}.emProject"
    RESULT_VARIABLE ses_organize_result
    OUTPUT_VARIABLE ses_organize_output
    ERROR_VARIABLE ses_organize_error
)
if(NOT "${ses_organize_result}" STREQUAL "0")
    message(FATAL_ERROR "SES project grouping failed: ${ses_organize_output}${ses_organize_error}")
endif()
string(STRIP "${ses_organize_output}" ses_organize_output)
message(STATUS "${ses_organize_output}")

include("${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/hpm_flash.cmake")
