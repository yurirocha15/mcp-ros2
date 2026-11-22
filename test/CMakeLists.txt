find_package(ament_cmake_gtest REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_msgs REQUIRED)

ament_add_gtest(mcp_ros2_unit_tests access_test.cpp message_json_test.cpp
                options_test.cpp)
target_link_libraries(mcp_ros2_unit_tests mcp_ros2_core)

# Each test starts the server and waits for discovery; the domain is kept to
# this machine.
ament_add_gtest(mcp_ros2_server_tests server_test.cpp TIMEOUT 300 ENV
                ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
target_link_libraries(mcp_ros2_server_tests mcp_ros2_core
                      ${example_interfaces_TARGETS} ${std_msgs_TARGETS})
target_compile_definitions(mcp_ros2_server_tests
                           PRIVATE MCP_ROS2_BINARY="$<TARGET_FILE:mcp_ros2>")
add_dependencies(mcp_ros2_server_tests mcp_ros2)
