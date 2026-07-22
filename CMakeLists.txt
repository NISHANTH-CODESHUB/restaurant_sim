cmake_minimum_required(VERSION 3.8)
project(restaurant_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

# ===== Lift controller plugin (drives the dynamic elevator) =====
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(lift_controller_plugin SHARED src/lift_controller_plugin.cpp)
target_link_libraries(lift_controller_plugin ${GAZEBO_LIBRARIES})

install(
  TARGETS lift_controller_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# Install models and worlds directories
install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Register environment hook to set GAZEBO_MODEL_PATH and GAZEBO_RESOURCE_PATH
ament_environment_hooks(
  "hooks/${PROJECT_NAME}.dsv.in"
)

ament_package()
