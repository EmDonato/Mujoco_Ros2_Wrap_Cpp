cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# DEPENDENCIES
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(interfaces REQUIRED) 
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
# =====================
# EXECUTABLE
# =====================
add_executable(mj_ros2_wrap
  src/main.cpp
  src/mj_ros2_wrap.cpp
)

# =====================
# MUJOCO
# =====================
set(MUJOCO_PATH $ENV{MUJOCO_PATH})

if(NOT DEFINED ENV{MUJOCO_PATH})
  message(FATAL_ERROR "MUJOCO_PATH is not set")
endif()

# include headers
target_include_directories(mj_ros2_wrap PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MUJOCO_PATH}/include
)

# link libraries
target_link_directories(mj_ros2_wrap PUBLIC
  ${MUJOCO_PATH}/lib
)

target_link_libraries(mj_ros2_wrap
  mujoco
  glfw
  Eigen3::Eigen
)

set_target_properties(mj_ros2_wrap PROPERTIES
  BUILD_RPATH ${MUJOCO_PATH}/lib
  INSTALL_RPATH ${MUJOCO_PATH}/lib
)

# =====================
# ROS2 DEPENDENCIES
# =====================
ament_target_dependencies(mj_ros2_wrap
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  interfaces
  tf2_ros
)

# =====================
# INSTALL
# =====================
install(TARGETS mj_ros2_wrap
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# =====================
# TESTING
# =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()