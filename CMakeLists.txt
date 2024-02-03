cmake_minimum_required(VERSION 3.20.0)

set(lib sx127x)
PROJECT(${lib} C)

set(${lib}_libtype PRIVATE PARENT_SCOPE)

file(GLOB SRCSC "./*.c")
file(GLOB SRCSH "./*.h")
add_library(${lib} ${${lib}_libtype} ${SRCSH} ${SRCSC})
target_link_libraries(${lib} PUBLIC zephyr_interface)
target_link_libraries(${lib} PUBLIC xspi)
target_link_libraries(${lib} PUBLIC xlog)
target_link_libraries(${lib} PUBLIC xglob)
target_link_libraries(${lib} PUBLIC xgpio)
target_link_libraries(${lib} PUBLIC bits)

set(lora_sdk $ENV{ZEPHYR_BASE}../modules/lib/loramac-node/src/radio/sx1276)
target_include_directories(${lib} PUBLIC ./ ${lora_sdk})
