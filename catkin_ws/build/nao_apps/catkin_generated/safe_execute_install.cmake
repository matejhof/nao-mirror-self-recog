execute_process(COMMAND "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_apps/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_apps/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
