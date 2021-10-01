# Install script for directory: /home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/src/interbotix_ros_arms/interbotix_descriptions

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/install")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_descriptions/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_descriptions/catkin_generated/installspace/interbotix_descriptions.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_descriptions/cmake" TYPE FILE FILES
    "/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_descriptions/catkin_generated/installspace/interbotix_descriptionsConfig.cmake"
    "/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_descriptions/catkin_generated/installspace/interbotix_descriptionsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_descriptions" TYPE FILE FILES "/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/src/interbotix_ros_arms/interbotix_descriptions/package.xml")
endif()

