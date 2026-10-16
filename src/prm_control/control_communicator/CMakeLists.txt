cmake_minimum_required(VERSION 3.5)
project(control_communicator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.

include_directories(/usr/include)
include_directories(include)

add_executable(ControlCommunicatorNode 
	src/ControlCommunicatorNode.cpp
	src/AimPredictor.cpp
)

target_include_directories(ControlCommunicatorNode PRIVATE ${OpenCV_INCLUDE_DIRS})

add_library(ControlCommunicator STATIC 
  src/ControlCommunicator.cpp
  src/PitchLookupModel.cpp
)

ament_target_dependencies(ControlCommunicatorNode 
	rclcpp
	std_msgs
  geometry_msgs
	vision_msgs
  tf2_ros
  tf2
  nav_msgs
)

# GTest setup
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_ControlCommunicator test/test_ControlCommunicator.cpp)
target_include_directories(test_ControlCommunicator PRIVATE include)
target_link_libraries(test_ControlCommunicator ControlCommunicator)
target_link_libraries(ControlCommunicatorNode Eigen3::Eigen ControlCommunicator ${OpenCV_LIBS})
ament_target_dependencies(test_ControlCommunicator rclcpp std_msgs geometry_msgs vision_msgs tf2_ros tf2 nav_msgs)

install(TARGETS
ControlCommunicatorNode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
