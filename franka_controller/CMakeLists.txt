cmake_minimum_required(VERSION 3.8)
project(franka_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(toppra CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(TARGET toppra::toppra)
  set(TOPPRA_TARGET toppra::toppra)
elseif(TARGET toppra)
  set(TOPPRA_TARGET toppra)
else()
  message(FATAL_ERROR "toppra target not found")
endif()

include_directories(
  include
  third_party
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(franka_controller_node
  src/config.cpp
  src/controller.cpp
  src/franka_interface.cpp
  src/joint_impedance.cpp
  src/main.cpp
  src/reporter.cpp
  src/trajectory_planner.cpp
  src/modes/cartesian_mode.cpp
  src/modes/compliant_mode.cpp
  src/modes/joint_mode.cpp
  src/modes/trajectory_mode.cpp
)

ament_target_dependencies(franka_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
)

target_link_libraries(franka_controller_node
  ${Franka_LIBRARIES}
  ${EIGEN3_LIBS}
  ${YAML_CPP_LIBRARIES}
  ${TOPPRA_TARGET}
)

target_include_directories(franka_controller_node SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
)

install(TARGETS franka_controller_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch.py" PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                                    GROUP_READ GROUP_EXECUTE
                                    WORLD_READ WORLD_EXECUTE
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
