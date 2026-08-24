cmake_minimum_required(VERSION 3.5)
project(prm_autobot_2023)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(ScanLimitNode src/ScanLimitNode.cpp)
ament_target_dependencies(ScanLimitNode rclcpp sensor_msgs)

install(TARGETS
  ScanLimitNode
  DESTINATION lib/${PROJECT_NAME}
)


nav2_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(DIRECTORY prm_autobot_2023 DESTINATION share/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/my_ammo_publisher.py
  scripts/my_health_publisher.py
  scripts/subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
