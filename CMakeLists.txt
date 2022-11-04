cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
pico_sdk_init()

# Project name and a few useful settings. Other commands can pick up the results
project(
  RP2040Base
  VERSION 0.1
  DESCRIPTION "RP2040 24way Test Bed"
)

add_subdirectory(src)