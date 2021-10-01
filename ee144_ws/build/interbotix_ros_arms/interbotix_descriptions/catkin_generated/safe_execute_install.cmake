execute_process(COMMAND "/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_descriptions/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/kevinurrutia/EE144-Introdution-To-Robotics/ee144_ws/build/interbotix_ros_arms/interbotix_descriptions/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
