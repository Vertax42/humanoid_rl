cmake_minimum_required(VERSION 3.10)
project(humanoid_rl)

set(CMAKE_BUILD_TYPE "Debug")
## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall")
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp  
  rospy
  std_msgs
  geometry_msgs
  roslib
  rosbag
  sensor_msgs
)

find_package(yaml-cpp REQUIRED)

set(HOME_DIR $ENV{HOME})
find_package(Torch REQUIRED HINTS "${HOME_DIR}/libtorch")
find_package(Eigen3 REQUIRED)

set(ONNXRUNTIME_INCLUDE_DIRS "/usr/local/include/onnxruntime")  # 修改为你的实际路径
set(ONNXRUNTIME_LIB_DIR "/usr/local/lib")

# Catkin-specific configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp geometry_msgs std_msgs sensor_msgs rosbag
)

# Include directories
include_directories(
  include
  include/common_tools/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${TORCH_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIRS}
)

# Declare the executable
add_executable(${PROJECT_NAME}_node
  src/humanoid_rl_inference.cpp
  src/humanoid_rl_controller.cpp
  src/humanoid_utils.cpp
  src/humanoid_rl_bag.cpp
  include/common_tools/log4z.cpp
  include/common_tools/utils_common.cpp
)

# Add target dependencies
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${TORCH_LIBRARIES}
  yaml-cpp
  ${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so
)

# Install targets
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

