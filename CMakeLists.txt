set(CMSISDSP ${CMAKE_CURRENT_LIST_DIR}/3rd-party/cmsis-dsp)

include_directories(
    include
    /usr/arm-linux-gnueabi/include
    ${CMSISDSP}/Include
    ${CMSISDSP}/PrivateInclude
    ${CMSISDSP}/Include/dsp
)

file(GLOB_RECURSE PHOENIX_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(phoenix ${PHOENIX_SOURCES})

target_link_libraries(phoenix
    CMSISDSP
    stm32cubemx
)