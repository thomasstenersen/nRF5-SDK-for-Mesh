set(target "thingy_${PLATFORM}_${SOFTDEVICE}")
set(THINGY_ROOT ${CMAKE_SOURCE_DIR}/external/thingy)
if (NOT ${PLATFORM} MATCHES nrf52832)
    message(FATAL_ERROR "Only nRF52832 supported for thingy")
endif()

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_ui.c"
    "${CMAKE_SOURCE_DIR}/mesh/stack/src/mesh_stack.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_provisionee.c"
    # "${CMAKE_SOURCE_DIR}/examples/common/src/rtt_input.c"
    # "${CMAKE_SOURCE_DIR}/examples/common/src/simple_hal.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_app_utils.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_adv.c"
    "${CMAKE_SOURCE_DIR}/mesh/prov/src/nrf_mesh_prov_bearer_gatt.c"

    "${SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c"
    "${SDK_ROOT}/components/ble/common/ble_advdata.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c"

    ${WEAK_SOURCE_FILES}
    ${MESH_CORE_SOURCE_FILES}
    ${MESH_BEARER_SOURCE_FILES}
    ${MESH_GATT_SOURCE_FILES}
    ${SIMPLE_ON_OFF_SERVER_SOURCE_FILES}
    ${SIMPLE_ON_OFF_CLIENT_SOURCE_FILES}
    ${CONFIG_SERVER_SOURCE_FILES}
    ${HEALTH_SERVER_SOURCE_FILES}
    ${ACCESS_SOURCE_FILES}
    ${PROV_PROVISIONEE_SOURCE_FILES}
    ${PROV_COMMON_SOURCE_FILES}
    ${${PLATFORM}_SOURCE_FILES}
    )

# Thingy drivers
target_sources(${target} PUBLIC
    "${THINGY_ROOT}/source/util/twi_manager.c"
    "${THINGY_ROOT}/source/util/support_func.c"
    "${THINGY_ROOT}/source/drivers/drv_ext_light.c"
    "${THINGY_ROOT}/source/drivers/drv_ext_gpio.c"
    "${THINGY_ROOT}/source/util/sx150x_led_drv_calc.c"
    "${THINGY_ROOT}/source/drivers/drv_sx1509.c"
    )
# SDK drivers
target_sources(${target} PUBLIC
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_twi.c"
    "${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_twi.c"
    "${SDK_ROOT}/components/libraries/button/app_button.c"
    "${SDK_ROOT}/components/libraries/util/app_error.c"
    "${SDK_ROOT}/components/libraries/scheduler/app_scheduler.c"
    "${SDK_ROOT}/components/libraries/timer/app_timer.c"
    "${SDK_ROOT}/components/libraries/util/nrf_assert.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c"
    "${SDK_ROOT}/components/libraries/util/sdk_mapped_flags.c"
    "${SDK_ROOT}/components/libraries/queue/nrf_queue.c"
    "${SDK_ROOT}/components/libraries/experimental_log/src/nrf_log_frontend.c"
    "${SDK_ROOT}/components/libraries/experimental_log/src/nrf_log_default_backends.c"
    "${SDK_ROOT}/components/libraries/experimental_log/src/nrf_log_backend_serial.c"
    "${SDK_ROOT}/components/libraries/experimental_log/src/nrf_log_backend_rtt.c"
    "${SDK_ROOT}/components/libraries/experimental_log/src/nrf_log_str_formatter.c"
    "${SDK_ROOT}/components/libraries/experimental_memobj/nrf_memobj.c"
    "${SDK_ROOT}/components/libraries/balloc/nrf_balloc.c"
    "${SDK_ROOT}/components/libraries/atomic/nrf_atomic.c"
    "${SDK_ROOT}/components/libraries/strerror/nrf_strerror.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/prs/nrfx_prs.c"
    "${SDK_ROOT}/external/fprintf/nrf_fprintf.c"
    "${SDK_ROOT}/external/fprintf/nrf_fprintf_format.c"
    "${SDK_ROOT}/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c"
    "${SDK_ROOT}/external/segger_rtt/SEGGER_RTT.c"
    "${SDK_ROOT}/external/segger_rtt/SEGGER_RTT_printf.c"
    )
target_include_directories(${target} PUBLIC
    "${SDK_ROOT}/modules/nrfx/drivers/include"
    "${SDK_ROOT}/integration/nrfx/legacy"
    "${SDK_ROOT}/modules/nrfx/"
    "${SDK_ROOT}/components/ble/common"
    "${SDK_ROOT}/components/softdevice/common"
    "${SDK_ROOT}/components/libraries/button"
    "${SDK_ROOT}/components/libraries/timer/"
    "${SDK_ROOT}/components/libraries/balloc/"
    "${SDK_ROOT}/components/libraries/experimental_section_vars"
    "${SDK_ROOT}/components/libraries/experimental_log"
    "${SDK_ROOT}/components/libraries/experimental_log/src"
    "${SDK_ROOT}/components/libraries/experimental_memobj/"
    "${SDK_ROOT}/components/libraries/strerror"
    "${SDK_ROOT}/components/libraries/twi"
    "${SDK_ROOT}/components/libraries/scheduler"
    "${SDK_ROOT}/components/libraries/atomic/"
    "${SDK_ROOT}/modules/nrfx/drivers/src/prs/"
    "${SDK_ROOT}/external/fprintf/"
    "${SDK_ROOT}/external/segger_rtt/"
    )


target_include_directories(${target} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
    "${CMAKE_SOURCE_DIR}/examples/common/include"
    # "${CMAKE_SOURCE_DIR}/external/rtt/include"
    ${SIMPLE_ON_OFF_SERVER_INCLUDE_DIRS}
    ${SIMPLE_ON_OFF_CLIENT_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${HEALTH_SERVER_INCLUDE_DIRS}
    ${MESH_GATT_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    )
target_include_directories(${target} PUBLIC
    "${THINGY_ROOT}/include/board"
    "${THINGY_ROOT}/include/drivers"
    "${THINGY_ROOT}/include/macros"
    "${THINGY_ROOT}/include/ble_services"
    "${THINGY_ROOT}/include/util"
    "${THINGY_ROOT}/libs/bv32fp-1.2"
    "${THINGY_ROOT}/libs/dvi_adpcm"
    "${THINGY_ROOT}/libs/eMD6/core/driver/eMPL"
    "${THINGY_ROOT}/libs/eMD6/core/driver/include"
    "${THINGY_ROOT}/libs/eMD6/core/driver/nRF52"
    "${THINGY_ROOT}/libs/eMD6/core/eMPL-hal"
    "${THINGY_ROOT}/libs/eMD6/core/mllite"
    "${THINGY_ROOT}/libs/eMD6/core/mpl"
    "${THINGY_ROOT}/libs/sr3_audio"
    "${THINGY_ROOT}/tools/sound"
    )


set_target_link_options(${target}
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/${PLATFORM}_${SOFTDEVICE})

target_compile_options(${target} PUBLIC
    ${${ARCH}_DEFINES})

target_compile_definitions(${target} PUBLIC
    -DNRF52
    -DNRF52_SERIES
    -DNRF52832
    -DNRF52832_XXAA
    -DBLE_STACK_SUPPORT_REQD
    -D__HEAP_SIZE=1024
    -DS132
    -DNRF_SD_BLE_API_VERSION=6
    -DNRF52_PAN_12
    -DNRF52_PAN_15
    -DNRF52_PAN_20
    -DNRF52_PAN_30
    -DNRF52_PAN_31
    -DNRF52_PAN_36
    -DNRF52_PAN_51
    -DNRF52_PAN_53
    -DNRF52_PAN_54
    -DNRF52_PAN_55
    -DNRF52_PAN_58
    -DNRF52_PAN_62
    -DNRF52_PAN_63
    -DNRF52_PAN_64
    # -DTHINGY_HW_v0_8_0
    -DSOFTDEVICE_PRESENT
    -DSWI_DISABLE0
    -DBSP_DEFINES_ONLY
    -DMPU9250
    -DEMPL
    -DUSE_DMP
    -DEMPL_TARGET_NRF52
    -DARM_MATH_CM4
    -DBLE_DFU_APP_SUPPORT
    -DNRF_DFU_SETTINGS_VERSION=1
    -DAUDIO_EQ_DEF=3
    -DHAL_NFC_ENGINEERING_BC_FTPAN_WORKAROUND
    -DGATT_PROXY
    -DMESH_FEATURE_GATT
    -DNRF_MESH_LOG_ENABLE=0
    -DBOARD_PCA20020)

target_link_libraries(${target}
    # rtt_${PLATFORM}
    uECC_${PLATFORM}
    ${THINGY_ROOT}/libs/liblibmplmpu_m4_hardfp/liblibmplmpu.a
    ${THINGY_ROOT}/sdk_components/toolchain/cmsis/dsp/GCC/libarm_cortexM4lf_math.a
    ${THINGY_ROOT}/sdk_components/nfc/t2t_lib/nfc_t2t_lib_gcc.a

    # This has to come last.
    m
    )

create_hex(${target})
add_flash_target(${target})

get_property(target_include_dirs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
add_pc_lint(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${target_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES}")
add_ses_project(${target})
