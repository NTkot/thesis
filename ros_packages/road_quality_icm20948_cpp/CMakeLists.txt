cmake_minimum_required(VERSION 3.8)
project(road_quality_icm20948_cpp VERSION 0.1.0)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif() 

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(road_quality_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(mraa REQUIRED mraa)


add_library(icm20948_utils icm20948_i2c/icm20948_utils.cpp)
target_include_directories(icm20948_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/icm20948_i2c)
ament_target_dependencies(icm20948_utils yaml_cpp_vendor)

add_library(icm20948_i2c icm20948_i2c/icm20948_i2c.cpp)
target_include_directories(icm20948_i2c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/icm20948_i2c)
target_link_libraries(icm20948_i2c icm20948_utils mraa)

add_library(icm20948_node src/icm20948_reader.cpp)
target_include_directories(icm20948_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/icm20948_i2c 
                                                ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(icm20948_node icm20948_i2c)
ament_target_dependencies(icm20948_node rclcpp tf2 yaml_cpp_vendor road_quality_msgs)

add_executable(icm20948_cpp_reader src/main.cpp)
target_include_directories(icm20948_cpp_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/icm20948_i2c 
                                                      ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(icm20948_cpp_reader icm20948_node ament_index_cpp::ament_index_cpp)
ament_target_dependencies(icm20948_cpp_reader rclcpp)


install(TARGETS
  icm20948_cpp_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/imu.yaml
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
