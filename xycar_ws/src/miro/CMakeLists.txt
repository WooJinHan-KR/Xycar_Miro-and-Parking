cmake_minimum_required(VERSION 3.0.2)
project(miro)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

set (OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  xycar_msgs
  rosconsole
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs xycar_msgs rosconsole geometry_msgs tf2
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
 # ${YDLIDAR_SDK_PATH}/include
 # ${YDLIDAR_SDK_PATH}/src
 # ${RPLIDAR_SDK_PATH}/include
#  ${RPLIDAR_SDK_PATH}/src
)

  add_library(miro_modules
    src/go.cpp
    src/main.cpp 
  )

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
 miro_modules
#  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
 # ${OpenCV_LIBRARIES}
)