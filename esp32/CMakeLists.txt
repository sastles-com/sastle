# ESP32 Isolation Sphere Project
# CMake build configuration

cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF CMake functions
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project configuration
set(PROJECT_NAME "isolation_sphere")
set(PROJECT_VER "1.0.0")

# Component directories
set(EXTRA_COMPONENT_DIRS 
    "${CMAKE_CURRENT_SOURCE_DIR}/components"
)

# Enable Unity test framework
set(TEST_COMPONENTS "main" CACHE STRING "Test components")

# Compile options
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Project declaration
project(${PROJECT_NAME})