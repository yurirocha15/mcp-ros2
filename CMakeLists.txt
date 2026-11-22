cmake_minimum_required(VERSION 3.22)

project(mcp_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_package_xml()

# The MCP C++ SDK: the installed package, as the ROS build farm provides it, or,
# for development builds without it, a copy fetched from GitHub.
find_package(mcp-cpp-sdk CONFIG QUIET)
if(mcp-cpp-sdk_FOUND)
  if(TARGET mcp::sdk)
    set(_mcp_sdk mcp::sdk)
  else()
    set(_mcp_sdk mcp::sdk_static)
  endif()
else()
  option(MCP_ROS2_FETCH_SDK "Fetch the MCP C++ SDK when it is not installed" ON)
  if(NOT MCP_ROS2_FETCH_SDK)
    message(
      FATAL_ERROR "The MCP C++ SDK (CMake package mcp-cpp-sdk) is not installed"
    )
  endif()
  set(MCP_ROS2_SDK_GIT_TAG
      "e7d0758ec8f4a2c901d3d4c9a2b16c300973a2b0"
      CACHE STRING "mcp-cpp-sdk commit or tag to fetch")
  set(MCP_CPP_SDK_BUILD_SHARED
      OFF
      CACHE BOOL "" FORCE)
  include(FetchContent)
  FetchContent_Declare(
    mcp-cpp-sdk
    GIT_REPOSITORY https://github.com/yurirocha15/mcp-cpp-sdk.git
    GIT_TAG ${MCP_ROS2_SDK_GIT_TAG})
  # The SDK's own tests are not this package's.
  set(BUILD_TESTING OFF)
  FetchContent_MakeAvailable(mcp-cpp-sdk)
  unset(BUILD_TESTING)
  set(_mcp_sdk mcp::sdk_static)
endif()

configure_file(src/version.hpp.in
               "${CMAKE_CURRENT_BINARY_DIR}/generated/version.hpp" @ONLY)

add_library(
  mcp_ros2_core STATIC src/message_json.cpp src/options.cpp src/ros_bridge.cpp
                       src/server.cpp src/type_support.cpp)
target_include_directories(
  mcp_ros2_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src"
                       "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(
  mcp_ros2_core
  PUBLIC
    ${_mcp_sdk}
    ${geometry_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
    rcl::rcl
    rcl_action::rcl_action
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rcpputils::rcpputils
    rmw::rmw
    rosidl_runtime_cpp::rosidl_runtime_cpp
    rosidl_typesupport_cpp::rosidl_typesupport_cpp
    rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
    tf2::tf2
    tf2_ros::tf2_ros)
target_compile_options(mcp_ros2_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mcp_ros2 src/main.cpp)
target_link_libraries(mcp_ros2 PRIVATE mcp_ros2_core)
target_compile_options(mcp_ros2 PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS mcp_ros2 RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
