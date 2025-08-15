cmake_minimum_required(VERSION 3.5)
#######################
project(force_control_demo)

include_directories(
  include/
  # yaml-cpp
  /home/kukabot/Projects/force_ctl/yaml-cpp/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(force_control_swing src/main.cc)
target_link_libraries(force_control_swing
  HI_COMMON
  UR_RTDE
  ATI_NETFT
  rt
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)

add_executable(force_control_walk src/walk.cc)
target_link_libraries(force_control_walk
  HI_COMMON
  UR_RTDE
  ATI_NETFT
  rt
  ${RUT}
  ${FORCE_CONTROLLERS}
  ${YAMLLib}
)