cmake_minimum_required(VERSION 3.8)
project(wirehawk_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Find Gazebo Harmonic libraries
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

# Compile C++ plugins 
add_library(CableDummyPlugin SHARED src/CableDummyPlugin.cc)
target_include_directories(CableDummyPlugin PUBLIC ${gz-sim8_INCLUDE_DIRS})
target_link_libraries(CableDummyPlugin PUBLIC gz-sim8::core gz-plugin2::gz-plugin2 gz-math7::gz-math7)

add_library(CableSpringPlugin SHARED src/CableSpringPlugin.cc)
target_include_directories(CableSpringPlugin PUBLIC ${gz-sim8_INCLUDE_DIRS})
target_link_libraries(CableSpringPlugin PUBLIC gz-sim8::core gz-plugin2::gz-plugin2 gz-math7::gz-math7)

install(TARGETS CableDummyPlugin CableSpringPlugin DESTINATION lib)
# ------------------------------

install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()