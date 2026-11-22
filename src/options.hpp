#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mcp_ros2 {

/// How the server talks to its client.
enum class Transport : std::uint8_t {
    eStdio,
    eHttp,
};

/// The command line, parsed.
struct Options {
    Transport transport = Transport::eStdio;
    std::string host = "127.0.0.1";
    std::uint16_t port = 9797;
    /// The bearer token HTTP clients must send; empty for none.
    std::string token;

    /// The name of the server's own ROS node.
    std::string node_name = "mcp_ros2";
    /// Leave out the tools that change the system: publish, call_service, send_action_goal and
    /// set_parameters.
    bool read_only = false;
    /// Arrays longer than this are shortened in the messages the server returns.
    std::size_t max_array = 64;
};

/// What parse_options() decided.
struct ParseResult {
    std::optional<Options> options;  ///< Set when the server should run.
    int exit_code = 0;               ///< The exit code when it should not.
    std::string message;             ///< Text for standard output (help, version) or an error.
};

std::string usage(std::string_view program);

/// Parses the command line, without the ROS arguments rclcpp takes. MCP_ROS2_TOKEN supplies the
/// token when --token is not given.
ParseResult parse_options(const std::vector<std::string>& args);

bool is_loopback_host(std::string_view host);

}  // namespace mcp_ros2
