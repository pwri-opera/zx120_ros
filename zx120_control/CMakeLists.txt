cmake_minimum_required(VERSION 3.0.2)
project(zx120_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  transmission_interface
  controller_manager
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES upper_arm_control
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(upper_arm_control src/upper_arm_control.cpp src/upper_arm_hw.cpp)

target_link_libraries(upper_arm_control
  ${catkin_LIBRARIES}
)

# add_executable(diff_track_control src/diff_track_control.cpp src/diff_track_hw.cpp)

# target_link_libraries(diff_track_control
#   ${catkin_LIBRARIES}
# )

# add_executable(vel_control src/vel_control.cpp src/vel_hw.cpp)

# target_link_libraries(vel_control
#   ${catkin_LIBRARIES}
# )

install(TARGETS upper_arm_control
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
