cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Hikrobot MVS SDK
set(MVS_ROOT /opt/MVS)

set(MVS_INCLUDE_DIR ${MVS_ROOT}/include)
set(MVS_LIBRARY ${MVS_ROOT}/lib/64/libMvCameraControl.so)

# Check MVS SDK files
if(NOT EXISTS ${MVS_INCLUDE_DIR}/MvCameraControl.h)
  message(FATAL_ERROR
    "MVS header not found: ${MVS_INCLUDE_DIR}/MvCameraControl.h")
endif()

if(NOT EXISTS ${MVS_LIBRARY})
  message(FATAL_ERROR
    "MVS library not found: ${MVS_LIBRARY}")
endif()

message(STATUS "MVS include directory: ${MVS_INCLUDE_DIR}")
message(STATUS "MVS library: ${MVS_LIBRARY}")

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)

target_compile_features(camera_node PRIVATE cxx_std_17)

target_include_directories(camera_node PRIVATE
  include
  ${MVS_INCLUDE_DIR}
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

target_link_libraries(camera_node
  ${MVS_LIBRARY}
)

# Runtime library search path
set_target_properties(camera_node PROPERTIES
  BUILD_RPATH "${MVS_ROOT}/lib/64"
  INSTALL_RPATH "${MVS_ROOT}/lib/64"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()