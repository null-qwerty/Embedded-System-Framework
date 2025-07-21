# 编译 DSP 库
set(CMSISDSP ${CMAKE_CURRENT_LIST_DIR}/3rd-party/cmsis-dsp CACHE INTERNAL "CMSIS DSP library path")
FILE(GLOB_RECURSE CMSISDSP_SOURCES ${CMSISDSP}/Source/*.*)
add_library(dsp STATIC ${CMSISDSP_SOURCES})
target_include_directories(dsp PRIVATE
    ${CMSISDSP}/Include
    ${CMSISDSP}/PrivateInclude
    ${CMSISDSP}/Include/dsp
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

set(FRAMEWORK_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
    /usr/arm-linux-gnueabi/include
    /usr/arm-linux-gnueabi/include/c++/14.2.1
    ${CMSISDSP}/Include
    ${CMSISDSP}/PrivateInclude
    ${CMSISDSP}/Include/dsp
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    CACHE INTERNAL "FRAMEWORK include directories"
)

include_directories(
    ${FRAMEWORK_INCLUDE_DIRS}
)

file(GLOB_RECURSE FRAMEWORK_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE 3RD-PARTY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/*.cpp")
list(REMOVE_ITEM FRAMEWORK_SOURCES ${3RD-PARTY_SOURCES})

add_library(embedded_system_framework ${FRAMEWORK_SOURCES})

target_link_libraries(embedded_system_framework
    dsp
    stm32cubemx
)