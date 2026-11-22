#pragma once

#include <cstdint>
#include <filesystem>
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
    /// Leave out the tools that change the system: publish, call_service, send_action_goal,
    /// cancel_action_goal and set_parameters, and action_goal_status, which follows sent goals.
    bool read_only = false;
    /// Arrays longer than this are shortened in the messages the server returns.
    std::size_t max_array = 64;
    /// Names the tools that act may touch, and names they may not, as AccessRules takes them.
    std::vector<std::string> allow;
    std::vector<std::string> deny;

    /// Seconds after start that graph queries wait for discovery to fill in the graph.
    double discovery_wait = 1.5;
    /// Seconds a read of a topic resource waits for a message.
    double resource_wait = 3;
    /// Least seconds between two notices that a subscribed topic has a new message.
    double notify_interval = 1;

    /// The HTTP server's certificate chain and private key, as PEM files; empty for plain HTTP.
    std::filesystem::path tls_certificate;
    std::filesystem::path tls_key;
    /// Authorities that client certificates must chain to; clients need one when any is given.
    std::vector<std::filesystem::path> tls_client_cas;
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
