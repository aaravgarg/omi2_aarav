# Install script for directory: /opt/nordic/ncs/v2.9.1/zephyr/drivers/sensor

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  set(CMAKE_OBJDUMP "/opt/nordic/ncs/toolchains/b8efef2ad5/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/adi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/ams/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/aosong/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/asahi_kasei/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/bosch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/espressif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/honeywell/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/infineon/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/ite/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/jedec/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/maxim/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/meas/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/microchip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/nuvoton/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/nxp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/rohm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/seeed/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/sensirion/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/silabs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/tdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/ti/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/vishay/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/Users/aaravgarg/Coding/omi2_aarav/test/build/build_OMI2_test_1/test/zephyr/drivers/sensor/wsen/cmake_install.cmake")
endif()

