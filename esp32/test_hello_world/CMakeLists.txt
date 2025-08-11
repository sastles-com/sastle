# ESP32S3 Hello World Test Project
cmake_minimum_required(VERSION 3.16)

# Set additional component directories
# set(EXTRA_COMPONENT_DIRS "../components")  # Temporarily disabled for simple test

# Include ESP-IDF components
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project name and version
project(test_hello_world)