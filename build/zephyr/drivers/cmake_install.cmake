# Install script for directory: /home/alex/zephyrproject/zephyr/drivers

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/alex/.espressif/tools/xtensa-esp32-elf/esp-2020r3-8.4.0/xtensa-esp32-elf/bin/xtensa-esp32-elf-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/console/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/interrupt_controller/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/pcie/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/clock_control/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/gpio/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/i2c/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/pinmux/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/serial/cmake_install.cmake")
  include("/home/alex/zephyrproject/zephyr/samples/i2c_LM75A/build/zephyr/drivers/timer/cmake_install.cmake")

endif()

