execute_process(COMMAND "/home/vojta/code-nao-simulation/gazebo9/catkin_ws/build/nao_apps/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/vojta/code-nao-simulation/gazebo9/catkin_ws/build/nao_apps/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
