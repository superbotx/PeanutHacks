execute_process(COMMAND "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosapi/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosapi/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
