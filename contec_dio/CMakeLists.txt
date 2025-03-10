cmake_minimum_required(VERSION 3.8)
project(contec_dio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(contec_dio_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(contec_dio_node src/node.cpp)
ament_target_dependencies(contec_dio_node contec_dio_interfaces rclcpp std_msgs)
target_link_libraries(contec_dio_node cdio)

install(TARGETS
        contec_dio_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
