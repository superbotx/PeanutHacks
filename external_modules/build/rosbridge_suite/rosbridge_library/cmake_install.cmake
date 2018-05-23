# Install script for directory: /home/ashis/botx_ws/PeanutHacks/external_modules/src/rosbridge_suite/rosbridge_library

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ashis/botx_ws/PeanutHacks/external_modules/install")
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
  include("/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosbridge_library/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosbridge_library/cmake" TYPE FILE FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosbridge_library/catkin_generated/installspace/rosbridge_library-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/include/rosbridge_library")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/share/roseus/ros/rosbridge_library")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/share/common-lisp/ros/rosbridge_library")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/share/gennodejs/ros/rosbridge_library")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/home/ashis/botx_ws/venv3/bin/python" -m compileall "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/lib/python3/dist-packages/rosbridge_library")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/lib/python3/dist-packages/rosbridge_library" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/lib/python3/dist-packages/rosbridge_library" FILES_MATCHING REGEX "/home/ashis/botx_ws/PeanutHacks/external_modules/devel/lib/python3/dist-packages/rosbridge_library/.+/__init__.pyc?$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosbridge_library/catkin_generated/installspace/rosbridge_library.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosbridge_library/cmake" TYPE FILE FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosbridge_library/catkin_generated/installspace/rosbridge_library-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosbridge_library/cmake" TYPE FILE FILES
    "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosbridge_library/catkin_generated/installspace/rosbridge_libraryConfig.cmake"
    "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosbridge_library/catkin_generated/installspace/rosbridge_libraryConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rosbridge_library" TYPE FILE FILES "/home/ashis/botx_ws/PeanutHacks/external_modules/src/rosbridge_suite/rosbridge_library/package.xml")
endif()

