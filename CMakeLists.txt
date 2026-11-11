cmake_minimum_required(VERSION 3.20)
project(sipr_lab2)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# Budowanie węzła sipr_lab2_node
add_executable(sipr_lab2_node
  src/sipr_lab2_node.cpp
  src/sipr_lab2/RobotController.cpp
)
target_include_directories(sipr_lab2_node PRIVATE include)
target_link_libraries(sipr_lab2_node
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  angles::angles
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)
install(TARGETS sipr_lab2_node DESTINATION lib/${PROJECT_NAME})

# Pliki launch, model robota i świat symulatora
install(DIRECTORY launch urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
