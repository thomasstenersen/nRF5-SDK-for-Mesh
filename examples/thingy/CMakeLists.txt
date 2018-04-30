set(target "thingy")
set(THINGY_ROOT ${CMAKE_SOURCE_DIR}/external/thingy)
if (NOT ${PLATFORM} MATCHES nrf52832)
    message(FATAL_ERROR "Only nRF52832 supported for thingy")
endif()

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    # "${CMAKE_SOURCE_DIR}/mesh/stack/src/mesh_stack.c"
    # "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_provisionee.c"
    # "${MBTLE_SOURCE_DIR}/examples/common/src/rtt_input.c"
    # "${CMAKE_SOURCE_DIR}/examples/common/src/simple_hal.c"
    # "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_app_utils.c"
    # "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_adv.c"
    # "${CMAKE_SOURCE_DIR}/mesh/prov/src/nrf_mesh_prov_bearer_gatt.c"
    # ${MESH_CORE_SOURCE_FILES}
    # ${MESH_BEARER_SOURCE_FILES}
    # ${MESH_GATT_SOURCE_FILES}
    # ${SIMPLE_ON_OFF_SERVER_SOURCE_FILES}
    # ${CONFIG_SERVER_SOURCE_FILES}
    # ${HEALTH_SERVER_SOURCE_FILES}
    # ${ACCESS_SOURCE_FILES}
    # ${PROV_PROVISIONEE_SOURCE_FILES}
    # ${PROV_COMMON_SOURCE_FILES}
    ${${PLATFORM}_SOURCE_FILES}
    )

target_sources(${target} PUBLIC
    "${THINGY_ROOT}/source/modules/m_ble.c"
    "${THINGY_ROOT}/source/modules/m_ble_flash.c"
    "${THINGY_ROOT}/source/util/advertiser_beacon_timeslot.c"
    "${THINGY_ROOT}/source/util/twi_manager.c"
    "${THINGY_ROOT}/source/util/support_func.c"
    "${THINGY_ROOT}/source/modules/m_environment.c"
    "${THINGY_ROOT}/source/modules/m_environment_flash.c"
    "${THINGY_ROOT}/source/modules/m_sound.c"
    "${THINGY_ROOT}/source/modules/m_motion.c"
    "${THINGY_ROOT}/source/modules/m_motion_flash.c"
    "${THINGY_ROOT}/source/modules/m_ui.c"
    "${THINGY_ROOT}/source/modules/m_ui_flash.c"
    "${THINGY_ROOT}/source/modules/m_batt_meas.c"
    "${THINGY_ROOT}/source/drivers/drv_gas_sensor.c"
    "${THINGY_ROOT}/source/drivers/drv_ccs811.c"
    "${THINGY_ROOT}/source/drivers/drv_pressure.c"
    "${THINGY_ROOT}/source/drivers/drv_lps22hb.c"
    "${THINGY_ROOT}/source/drivers/drv_humidity.c"
    "${THINGY_ROOT}/source/drivers/drv_hts221.c"
    "${THINGY_ROOT}/source/drivers/drv_ext_light.c"
    "${THINGY_ROOT}/source/drivers/drv_ext_gpio.c"
    "${THINGY_ROOT}/source/drivers/drv_nfc.c"
    "${THINGY_ROOT}/source/util/sx150x_led_drv_calc.c"
    "${THINGY_ROOT}/source/drivers/drv_sx1509.c"
    "${THINGY_ROOT}/source/drivers/drv_bh1745.c"
    "${THINGY_ROOT}/source/drivers/drv_color.c"
    "${THINGY_ROOT}/source/drivers/drv_speaker.c"
    "${THINGY_ROOT}/source/drivers/drv_mpu9250.c"
    "${THINGY_ROOT}/source/drivers/drv_motion.c"
    "${THINGY_ROOT}/source/drivers/drv_acc_lis3dh.c"
    "${THINGY_ROOT}/source/drivers/drv_mic.c"
    "${THINGY_ROOT}/libs/eMD6/core/driver/eMPL/inv_mpu.c"
    "${THINGY_ROOT}/libs/eMD6/core/driver/eMPL/inv_mpu_dmp_motion_driver.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/data_builder.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/hal_outputs.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/message_layer.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/ml_math_func.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/mlmath.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/mpl.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/results_holder.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/start_manager.c"
    "${THINGY_ROOT}/libs/eMD6/core/mllite/storage_manager.c"
    "${THINGY_ROOT}/libs/eMD6/core/eMPL-hal/eMPL_outputs.c"
    "${THINGY_ROOT}/libs/eMD6/core/driver/nRF52/log_nrf52.c"
    "${THINGY_ROOT}/libs/dvi_adpcm/dvi_adpcm.c"
    "${THINGY_ROOT}/libs/sr3_audio/drv_audio_coder_adpcm.c"
    "${THINGY_ROOT}/libs/sr3_audio/drv_audio_coder_bv32fp.c"
    "${THINGY_ROOT}/libs/sr3_audio/drv_audio_dsp.c"
    "${THINGY_ROOT}/libs/sr3_audio/drv_audio_pdm.c"
    "${THINGY_ROOT}/sdk_components/ble/common/ble_advdata.c"
    "${THINGY_ROOT}/sdk_components/ble/ble_advertising/ble_advertising.c"
    "${THINGY_ROOT}/sdk_components/ble/common/ble_conn_params.c"
    "${THINGY_ROOT}/sdk_components/ble/common/ble_srv_common.c"
    "${THINGY_ROOT}/sdk_components/ble/common/ble_conn_state.c"
    "${THINGY_ROOT}/source/ble_services/ble_tes.c"
    "${THINGY_ROOT}/source/ble_services/ble_tcs.c"
    "${THINGY_ROOT}/source/ble_services/ble_uis.c"
    "${THINGY_ROOT}/source/ble_services/ble_tms.c"
    "${THINGY_ROOT}/source/ble_services/ble_tss.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/common/nrf_drv_common.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/gpiote/nrf_drv_gpiote.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/twi_master/nrf_drv_twi.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/pwm/nrf_drv_pwm.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/pdm/nrf_drv_pdm.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/clock/nrf_drv_clock.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/saadc/nrf_drv_saadc.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/hal/nrf_saadc.c"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/rng/nrf_drv_rng.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/launchapp/nfc_launchapp_msg.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/launchapp/nfc_launchapp_rec.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/uri/nfc_uri_msg.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/uri/nfc_uri_rec.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/generic/message/nfc_ndef_msg.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/generic/record/nfc_ndef_record.c"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/text/nfc_text_rec.c"
    "${THINGY_ROOT}/sdk_components/nfc/t2t_lib/hal_t2t/hal_nfc_t2t.c"
    "${THINGY_ROOT}/sdk_components/libraries/button/app_button.c"
    "${THINGY_ROOT}/sdk_components/libraries/util/app_error.c"
    "${THINGY_ROOT}/sdk_components/libraries/scheduler/app_scheduler.c"
    "${THINGY_ROOT}/sdk_components/libraries/timer/app_timer.c"
    "${THINGY_ROOT}/sdk_components/libraries/util/nrf_assert.c"
    "${THINGY_ROOT}/sdk_components/libraries/uart/retarget.c"
    "${THINGY_ROOT}/sdk_components/libraries/led_softblink/led_softblink.c"
    "${THINGY_ROOT}/sdk_components/libraries/low_power_pwm/low_power_pwm.c"
    "${THINGY_ROOT}/sdk_components/libraries/fstorage/fstorage.c"
    "${THINGY_ROOT}/sdk_components/libraries/fds/fds.c"
    "${THINGY_ROOT}/sdk_components/libraries/util/app_util_platform.c"
    "${THINGY_ROOT}/sdk_components/libraries/util/sdk_mapped_flags.c"
    "${THINGY_ROOT}/sdk_components/libraries/queue/nrf_queue.c"
    "${THINGY_ROOT}/external/segger_rtt/RTT_Syscalls_GCC.c"
    "${THINGY_ROOT}/external/segger_rtt/SEGGER_RTT.c"
    "${THINGY_ROOT}/external/segger_rtt/SEGGER_RTT_printf.c"
    "${THINGY_ROOT}/external/sdk13_override/softdevice_handler_appsh.c"
    "${THINGY_ROOT}/sdk_components/softdevice/common/softdevice_handler/softdevice_handler.c"
    "${THINGY_ROOT}/sdk_components/libraries/log/src/nrf_log_frontend.c"
    "${THINGY_ROOT}/sdk_components/libraries/log/src/nrf_log_backend_serial.c"
    "${THINGY_ROOT}/source/util/nrf_dfu_flash_buttonless.c"
    "${THINGY_ROOT}/sdk_components/ble/ble_services/ble_dfu/ble_dfu.c"
    "${THINGY_ROOT}/sdk_components/ble/ble_services/ble_bas/ble_bas.c"
    "${THINGY_ROOT}/sdk_components/libraries/bootloader/dfu/nrf_dfu_settings.c"
    "${THINGY_ROOT}/sdk_components/libraries/crc32/crc32.c"
    "${THINGY_ROOT}/sdk_components/libraries/strerror/nrf_strerror.c")

target_include_directories(${target} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    # "${CMAKE_CURRENT_SOURCE_DIR}/../include"
    # "${CMAKE_SOURCE_DIR}/examples/common/include"
    "${CMAKE_SOURCE_DIR}/external/rtt/include"
    # ${SIMPLE_ON_OFF_SERVER_INCLUDE_DIRS}
    # ${CONFIG_SERVER_INCLUDE_DIRS}
    # ${HEALTH_SERVER_INCLUDE_DIRS}
    # ${MESH_GATT_INCLUDE_DIRS}
    # ${MESH_INCLUDE_DIRS}
    # ${${SOFTDEVICE}_INCLUDE_DIRS}
    # ${${PLATFORM}_INCLUDE_DIRS}
    )

target_include_directories(${target} PUBLIC
    "${THINGY_ROOT}/include/ble_services"
    "${THINGY_ROOT}/include/board"
    "${THINGY_ROOT}/include/drivers"
    "${THINGY_ROOT}/include/macros"
    "${THINGY_ROOT}/include/modules"
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
    "${THINGY_ROOT}/sdk_components/boards"
    "${THINGY_ROOT}/sdk_components/ble/ble_advertising"
    "${THINGY_ROOT}/sdk_components/ble/ble_services/ble_bas"
    "${THINGY_ROOT}/sdk_components/ble/ble_services/ble_dfu"
    "${THINGY_ROOT}/sdk_components/ble/common"
    "${THINGY_ROOT}/sdk_components/device"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/clock"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/common"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/config"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/delay"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/gpiote"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/hal"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/pdm"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/pstorage"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/pwm"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/rng"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/saadc"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/twi_master"
    "${THINGY_ROOT}/sdk_components/drivers_nrf/uart"
    "${THINGY_ROOT}/sdk_components/libraries/bootloader/dfu"
    "${THINGY_ROOT}/sdk_components/libraries/button"
    "${THINGY_ROOT}/sdk_components/libraries/crc32"
    "${THINGY_ROOT}/sdk_components/libraries/experimental_section_vars"
    "${THINGY_ROOT}/sdk_components/libraries/experimental_section_vars"
    "${THINGY_ROOT}/sdk_components/libraries/fds"
    "${THINGY_ROOT}/sdk_components/libraries/fifo"
    "${THINGY_ROOT}/sdk_components/libraries/fstorage"
    "${THINGY_ROOT}/sdk_components/libraries/fstorage"
    "${THINGY_ROOT}/sdk_components/libraries/led_softblink"
    "${THINGY_ROOT}/sdk_components/libraries/log"
    "${THINGY_ROOT}/sdk_components/libraries/log/src"
    "${THINGY_ROOT}/sdk_components/libraries/low_power_pwm"
    "${THINGY_ROOT}/sdk_components/libraries/scheduler"
    "${THINGY_ROOT}/sdk_components/libraries/strerror"
    "${THINGY_ROOT}/sdk_components/libraries/timer"
    "${THINGY_ROOT}/sdk_components/libraries/uart"
    "${THINGY_ROOT}/sdk_components/libraries/util"
    "${THINGY_ROOT}/sdk_components/libraries/queue"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/generic/record"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/generic/message"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/launchapp"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/uri"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/text"
    "${THINGY_ROOT}/sdk_components/nfc/t2t_lib"
    "${THINGY_ROOT}/sdk_components/nfc/ndef/text"
    "${THINGY_ROOT}/sdk_components/softdevice/common/softdevice_handler"
    "${THINGY_ROOT}/sdk_components/softdevice/s132/headers"
    "${THINGY_ROOT}/sdk_components/softdevice/s132/headers/nrf52"
    "${THINGY_ROOT}/sdk_components/toolchain"
    "${THINGY_ROOT}/sdk_components/toolchain/cmsis/include"
    "${THINGY_ROOT}/sdk_components/toolchain/gcc"
    "${THINGY_ROOT}/tools/sound")


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
    -DNRF_SD_BLE_API_VERSION=5
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
    -DSOFTDEVICE_PRESENT
    -DSWI_DISABLE0
    -DBSP_DEFINES_ONLY
    -DMPU9250
    -DEMPL
    -DUSE_DMP
    -DEMPL_TARGET_NRF52
    -DARM_MATH_CM4
    -DNRF_LOG_USES_RTT
    -DNRF_LOG_USES_RTT=1
    -DBLE_DFU_APP_SUPPORT
    -DNRF_DFU_SETTINGS_VERSION=1
    -DAUDIO_EQ_DEF=3
    -DHAL_NFC_ENGINEERING_BC_FTPAN_WORKAROUND)

target_link_libraries(${target}
    rtt_${PLATFORM}
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
