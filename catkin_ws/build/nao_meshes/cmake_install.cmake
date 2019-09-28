# Install script for directory: /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/nao_meshes

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" TYPE PROGRAM FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" TYPE PROGRAM FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/setup.bash;/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" TYPE FILE FILES
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/setup.bash"
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/setup.sh;/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" TYPE FILE FILES
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/setup.sh"
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/setup.zsh;/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" TYPE FILE FILES
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/setup.zsh"
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" TYPE FILE FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/nao_meshes.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nao_meshes/cmake" TYPE FILE FILES
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/nao_meshesConfig.cmake"
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/catkin_generated/installspace/nao_meshesConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nao_meshes" TYPE FILE FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/nao_meshes/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
# uncompress the archive
message(STATUS "using /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp/installer.run to decompress to /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp")
execute_process(COMMAND /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp/installer.run --mode unattended --prefix /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp
                OUTPUT_VARIABLE OUT
                ERROR_VARIABLE ERROR
                RESULT_VARIABLE RESULT
)
message(STATUS "intall out: ${OUT}")
message(STATUS "install error: ${ERROR}")
message(STATUS "install result: ${RESULT}")

execute_process(COMMAND ls
                WORKING_DIRECTORY /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp
                OUTPUT_VARIABLE OUT_LS
                ERROR_VARIABLE ERROR_LS
                RESULT_VARIABLE RESULT_LS
)
message(STATUS "ls out: ${OUT_LS}")
message(STATUS "ls error: ${ERROR_LS}")
message(STATUS "ls result: ${RESULT_LS}")

execute_process(COMMAND file /home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp/installer.run
                OUTPUT_VARIABLE OUT_FILE
                ERROR_VARIABLE ERROR_FILE
                RESULT_VARIABLE RESULT_FILE
)
message(STATUS "file out: ${OUT_FILE}")
message(STATUS "file error: ${ERROR_FILE}")
message(STATUS "file result: ${RESULT_FILE}")

endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nao_meshes//" TYPE DIRECTORY FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp/meshes")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nao_meshes//" TYPE DIRECTORY FILES "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/devel/.private/nao_meshes/tmp/texture")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_meshes/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
