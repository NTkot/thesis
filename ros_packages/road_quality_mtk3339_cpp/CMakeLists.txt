cmake_minimum_required(VERSION 3.8)
project(road_quality_mtk3339_cpp VERSION 0.1.0)

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
find_package(road_quality_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)


find_path(LIBNMEA_INCLUDE_DIR 
  NAMES nmea.h
  HINTS /usr/include)
find_library(LIBNMEA_LIBRARY 
  NAMES libnmea.so
  HINTS /usr/lib)


add_library(mtk_utils mtk3339_uart/mtk_utils.cpp)
target_include_directories(mtk_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mtk3339_uart)
ament_target_dependencies(mtk_utils yaml_cpp_vendor)

add_library(mtk_uart mtk3339_uart/mtk_uart.cpp)
target_include_directories(mtk_uart PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mtk3339_uart)
target_link_libraries(mtk_uart ${LIBNMEA_LIBRARY} mtk_utils)

add_library(mtk3339_node src/mtk3339_reader.cpp)
target_include_directories(mtk3339_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mtk3339_uart 
                                               ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mtk3339_node ${LIBNMEA_LIBRARY} mtk_uart)
ament_target_dependencies(mtk3339_node rclcpp yaml_cpp_vendor road_quality_msgs)

add_executable(mtk3339_cpp_reader src/main.cpp)
target_include_directories(mtk3339_cpp_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/mtk3339_uart 
                                                     ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mtk3339_cpp_reader mtk3339_node ament_index_cpp::ament_index_cpp)
ament_target_dependencies(mtk3339_cpp_reader rclcpp yaml_cpp_vendor road_quality_msgs)


install(TARGETS
  mtk3339_cpp_reader 
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/config/gps.yaml
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
