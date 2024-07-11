cmake_minimum_required(VERSION 3.8)
project(road_quality_core VERSION 0.1.0)


if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif() 

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(road_quality_msgs REQUIRED)


add_library(nucleus_node src/nucleus.cpp)
target_include_directories(nucleus_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nucleus_node ament_index_cpp::ament_index_cpp)
ament_target_dependencies(nucleus_node rclcpp tf2_ros road_quality_msgs)

add_executable(nucleus src/main.cpp)
target_include_directories(nucleus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nucleus nucleus_node)
ament_target_dependencies(nucleus rclcpp tf2_ros road_quality_msgs)


install(TARGETS
  nucleus
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
