cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	MBEDTLS_CONFIG_FILE=<mbedtls_config.h> 
	USE_PWR_LDO_SUPPLY 
	USE_HAL_DRIVER 
	STM32H743xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../FATFS/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../FATFS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MBEDTLS/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FatFs/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/include/mbedtls
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MBEDTLS/App/mbedtls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../MBEDTLS/Target/hardware_rng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../FATFS/App/fatfs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../FATFS/Target/bsp_driver_sd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../FATFS/Target/sd_diskio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h7xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32h7xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32h743xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32h7xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dcmi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rng_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rtc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_sdmmc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_delayblock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_sd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mmc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mmc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
)

# Drivers Midllewares


set(USB_Device_Library_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)
set(mbedTLS_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/aes.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/aesni.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/arc4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/asn1parse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/asn1write.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/base64.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/bignum.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/blowfish.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/camellia.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ccm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/certs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/chacha20.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/chachapoly.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/cipher.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/cipher_wrap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ctr_drbg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/des.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/dhm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ecdh.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ecdsa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ecp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ecp_curves.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/entropy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/entropy_poll.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/error.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/gcm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/hkdf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/hmac_drbg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/md.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/md5.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/md_wrap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/memory_buffer_alloc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/oid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/padlock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pk.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pkcs12.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pkcs5.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pkparse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pkwrite.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/pk_wrap.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/platform.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/platform_util.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/poly1305.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ripemd160.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/rsa.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/rsa_internal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/sha1.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/sha256.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/sha512.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ssl_cache.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ssl_ciphersuites.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ssl_cli.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ssl_cookie.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ssl_ticket.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/ssl_tls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/version.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/version_features.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509write_crt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509write_csr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509_create.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509_crl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509_crt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/x509_csr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/mbedTLS/library/xtea.c
)
set(FatFs_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FatFs/src/diskio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FatFs/src/ff.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FatFs/src/option/syscall.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FatFs/src/option/ccsbcs.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    USB_Device_Library
	mbedTLS
	FatFs
	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Create mbedTLS static library
add_library(mbedTLS OBJECT)
target_sources(mbedTLS PRIVATE ${mbedTLS_Src})
target_link_libraries(mbedTLS PUBLIC stm32cubemx)

# Create FatFs static library
add_library(FatFs OBJECT)
target_sources(FatFs PRIVATE ${FatFs_Src})
target_link_libraries(FatFs PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
