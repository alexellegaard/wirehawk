cmake_minimum_required(VERSION 3.8)
project(wirehawk_ethercat_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wirehawk_msgs REQUIRED)

# SOEM is not a ROS2 package; resolve it by path like the standalone Makefile.
# It must be built first:  cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
set(SOEM_DIR "$ENV{HOME}/SOEM" CACHE PATH "SOEM source tree")
set(SOEM_INCLUDE_DIRS
    ${SOEM_DIR}/include
    ${SOEM_DIR}/build/include
    ${SOEM_DIR}/osal
    ${SOEM_DIR}/osal/linux
    ${SOEM_DIR}/oshw/linux)
set(SOEM_LIB ${SOEM_DIR}/build/libsoem.a)

add_executable(ethercat_bridge_node
  src/ethercat_bridge_node.cpp
  src/soem_rt_loop.cpp)

target_include_directories(ethercat_bridge_node PRIVATE
  include
  ${SOEM_INCLUDE_DIRS})

target_link_libraries(ethercat_bridge_node
  ${SOEM_LIB}
  pthread)

target_compile_definitions(ethercat_bridge_node PRIVATE _GNU_SOURCE)

ament_target_dependencies(ethercat_bridge_node
  rclcpp
  wirehawk_msgs)

install(TARGETS ethercat_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
