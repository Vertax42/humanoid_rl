cmake_minimum_required(VERSION 3.10)
project(joy_stick)

set(CMAKE_BUILD_TYPE "Debug")
## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall")
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  message_generation
)

find_package(SDL2 REQUIRED)
# Catkin-specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy roscpp std_msgs
  DEPENDS SDL2
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)

# Declare the executable
add_executable(${PROJECT_NAME}_node
  src/joy.cpp
  src/joy_stick_impl.cpp
  src/log4z.cpp
)
find_package(yaml-cpp REQUIRED)
# Add target dependencies
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${SDL2_LIBRARIES}
  yaml-cpp
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

