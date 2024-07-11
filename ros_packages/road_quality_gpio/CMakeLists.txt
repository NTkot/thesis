cmake_minimum_required(VERSION 3.8)
project(road_quality_gpio VERSION 0.1.0)

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
find_package(yaml_cpp_vendor REQUIRED)
find_package(road_quality_msgs REQUIRED)


find_path(pigpiod_if2_INCLUDE_DIR 
	NAMES pigpiod_if2.h
	HINTS /usr/local/include)
find_library(pigpiod_if2_LIBRARY 
  NAMES libpigpiod_if2.so
	HINTS /usr/local/lib)


add_library(pi_gpio_node src/pi_gpio.cpp)
target_include_directories(pi_gpio_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${pigpiod_if2_INCLUDE_DIR})
target_link_libraries(pi_gpio_node ${pigpiod_if2_LIBRARY})
ament_target_dependencies(pi_gpio_node rclcpp yaml_cpp_vendor road_quality_msgs)

add_executable(gpio_handler src/main.cpp)
target_include_directories(gpio_handler PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${pigpiod_if2_INCLUDE_DIR})
target_link_libraries(gpio_handler pi_gpio_node ament_index_cpp::ament_index_cpp ${pigpiod_if2_LIBRARY})
ament_target_dependencies(gpio_handler rclcpp yaml_cpp_vendor road_quality_msgs)


install(TARGETS
  gpio_handler 
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/gpio.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
