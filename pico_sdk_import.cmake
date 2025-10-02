set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH environment variable is not set")
endif()

include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

