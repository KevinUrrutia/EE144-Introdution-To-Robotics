# Install script for directory: /home/kevin/EE144-Introdution-To-Robotics/ee144_ws/src/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_moveit_interface/srv" TYPE FILE FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/src/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/srv/MoveItPlan.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_moveit_interface/cmake" TYPE FILE FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/interbotix_moveit_interface-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/devel/include/interbotix_moveit_interface")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/devel/share/roseus/ros/interbotix_moveit_interface")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/devel/share/common-lisp/ros/interbotix_moveit_interface")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/devel/share/gennodejs/ros/interbotix_moveit_interface")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/devel/lib/python2.7/dist-packages/interbotix_moveit_interface")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/devel/lib/python2.7/dist-packages/interbotix_moveit_interface")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/interbotix_moveit_interface.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_moveit_interface/cmake" TYPE FILE FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/interbotix_moveit_interface-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_moveit_interface/cmake" TYPE FILE FILES
    "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/interbotix_moveit_interfaceConfig.cmake"
    "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/interbotix_moveit_interfaceConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/interbotix_moveit_interface" TYPE FILE FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/src/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_moveit_interface" TYPE PROGRAM FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/moveit_interface_gui")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/interbotix_moveit_interface" TYPE PROGRAM FILES "/home/kevin/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_examples/interbotix_moveit_interface/catkin_generated/installspace/moveit_python_interface")
endif()

