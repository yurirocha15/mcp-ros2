#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mcp_ros2 {

/// MCP transport.
enum class Transport : std::uint8_t {
    eStdio,
    eHttp,
};

/// Parsed command-line options.
struct Options {
    Transport transport = Transport::eStdio;
    std::string host = "127.0.0.1";
    std::uint16_t port = 9797;
    /// Bearer token required from HTTP clients; empty for none.
    std::string token;

    /// Name of the server's ROS node.
    std::string node_name = "mcp_ros2";
    /// Omits the tools that change the system (publish, call_service, send_action_goal,
    /// cancel_action_goal, set_parameters) and action_goal_status, which reports on sent goals.
    bool read_only = false;
    /// Arrays longer than this are truncated in returned messages.
    std::size_t max_array = 64;
    /// Allow and deny patterns for AccessRules.
    std::vector<std::string> allow;
    std::vector<std::string> deny;

    /// Seconds after node start during which graph queries wait for discovery.
    double discovery_wait = 1.5;
    /// Seconds a topic resource read waits for a message.
    double resource_wait = 3;
    /// Minimum seconds between update notifications for one subscribed topic.
    double notify_interval = 1;

    /// PEM certificate chain and private key for HTTPS; empty for plain HTTP.
    std::filesystem::path tls_certificate;
    std::filesystem::path tls_key;
    /// PEM CA files for verifying client certificates; when set, clients must present one.
    std::vector<std::filesystem::path> tls_client_cas;
};

/// Result of parse_options().
struct ParseResult {
    std::optional<Options> options;  ///< Set when the server should run.
    int exit_code = 0;               ///< Exit code when the server should not run.
    std::string message;             ///< Text for standard output (help, version) or an error.
};

std::string usage(std::string_view program);

/// Parses the command line, with the ROS arguments already removed. Reads the token from
/// MCP_ROS2_TOKEN when --token is absent.
ParseResult parse_options(const std::vector<std::string>& args);

bool is_loopback_host(std::string_view host);

}  // namespace mcp_ros2
