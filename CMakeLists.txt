cmake_minimum_required(VERSION 3.0.2)
project(crsf_drv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  catkin_virtualenv
  rostest
)

set(executables
  scripts/crsf_drv_node.py
)

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3.8
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crsf_drv
#  CATKIN_DEPENDS rospy,sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Make sure your python executables are installed using `catkin_install_python`:
catkin_install_python(
  PROGRAMS
  ${executables}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # nosetests
  catkin_add_nosetests(tests
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )

  catkin_python_setup()

  # rostests
  # catkin_install_python(
  #   PROGRAMS
  #     tests/test_script
  #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  add_rostest(tests/run_tests.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()