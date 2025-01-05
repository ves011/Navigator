# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(navigator)

target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/client.key" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "certs/ca.crt" TEXT)