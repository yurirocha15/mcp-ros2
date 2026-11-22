#include "options.hpp"

#include "version.hpp"

#include <cctype>
#include <charconv>
#include <cstdlib>
#include <sstream>

namespace mcp_ros2 {

namespace {

template <typename T>
std::optional<T> parse_number(std::string_view text) {
    T value{};
    const auto* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(text.data(), end, value);
    if (ec != std::errc() || ptr != end) {
        return std::nullopt;
    }
    return value;
}

ParseResult error(std::string message) {
    ParseResult result;
    result.exit_code = 2;
    result.message = std::move(message) + "\nRun with --help for usage.\n";
    return result;
}

bool is_node_name(std::string_view name) {
    if (name.empty() ||
        (std::isalpha(static_cast<unsigned char>(name.front())) == 0 && name.front() != '_')) {
        return false;
    }
    for (const char c : name) {
        if (std::isalnum(static_cast<unsigned char>(c)) == 0 && c != '_') {
            return false;
        }
    }
    return true;
}

}  // namespace

std::string usage(std::string_view program) {
    std::ostringstream out;
    out << "Usage: " << program << " [options] [--ros-args ...]\n"
        << "\n"
        << "An MCP server for ROS 2: it lets an agent look at a running ROS 2 system and act on\n"
        << "it, as a node of its own. Topics, services, actions, parameters and transforms of any\n"
        << "installed interface type, as JSON.\n"
        << "\n"
        << "Options:\n"
        << "  --read-only                Leave out the tools that change the system: publish,\n"
        << "                             call_service, send_action_goal and set_parameters\n"
        << "  --node-name=<name>         The server's node name (default: mcp_ros2)\n"
        << "  --max-array=<n>            Shorten arrays longer than this in returned messages\n"
        << "                             (default: 64)\n"
        << "  --transport=<stdio|http>   How clients connect (default: stdio)\n"
        << "  --host=<address>           HTTP listen address (default: 127.0.0.1)\n"
        << "  --port=<n>                 HTTP listen port (default: 9797)\n"
        << "  --token=<token>            Bearer token HTTP clients must send\n"
        << "                             (default: $MCP_ROS2_TOKEN; required off loopback)\n"
        << "  --version                  Print the version\n"
        << "  --help                     Print this help\n"
        << "\n"
        << "Arguments after --ros-args go to ROS, such as -r __ns:=/robot1 or\n"
        << "-p use_sim_time:=true. The ROS domain comes from ROS_DOMAIN_ID, as for any node.\n";
    return out.str();
}

bool is_loopback_host(std::string_view host) {
    return host == "127.0.0.1" || host == "::1" || host == "localhost" || host.starts_with("127.");
}

ParseResult parse_options(const std::vector<std::string>& args) {
    Options options;
    if (const char* token = std::getenv("MCP_ROS2_TOKEN"); token != nullptr) {
        options.token = token;
    }
    const std::string_view program = args.empty() ? "mcp-ros2" : std::string_view(args.front());

    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string_view arg = args[i];
        const auto value_of = [&arg](std::string_view prefix) -> std::optional<std::string_view> {
            if (arg.starts_with(prefix)) {
                return arg.substr(prefix.size());
            }
            return std::nullopt;
        };

        if (arg == "--help" || arg == "-h") {
            return {std::nullopt, 0, usage(program)};
        }
        if (arg == "--version") {
            return {std::nullopt, 0, std::string("mcp-ros2 ") + g_VERSION + "\n"};
        }
        if (arg == "--read-only") {
            options.read_only = true;
        } else if (auto value = value_of("--node-name=")) {
            if (!is_node_name(*value)) {
                return error("Invalid --node-name (letters, digits and underscores): " +
                             std::string(*value));
            }
            options.node_name = std::string(*value);
        } else if (auto value = value_of("--max-array=")) {
            const auto parsed = parse_number<std::size_t>(*value);
            if (!parsed || *parsed == 0) {
                return error("Invalid --max-array: " + std::string(*value));
            }
            options.max_array = *parsed;
        } else if (auto value = value_of("--transport=")) {
            if (*value == "stdio") {
                options.transport = Transport::eStdio;
            } else if (*value == "http") {
                options.transport = Transport::eHttp;
            } else {
                return error("Unknown transport: " + std::string(*value));
            }
        } else if (auto value = value_of("--host=")) {
            if (value->empty()) {
                return error("--host needs an address");
            }
            options.host = std::string(*value);
        } else if (auto value = value_of("--port=")) {
            const auto parsed = parse_number<std::uint16_t>(*value);
            if (!parsed || *parsed == 0) {
                return error("Invalid port: " + std::string(*value));
            }
            options.port = *parsed;
        } else if (auto value = value_of("--token=")) {
            options.token = std::string(*value);
        } else {
            return error("Unknown option: " + std::string(arg));
        }
    }

    if (options.transport == Transport::eHttp && !is_loopback_host(options.host) &&
        options.token.empty()) {
        return error(
            "Listening on " + options.host +
            " lets other machines call the server; set a token with --token or MCP_ROS2_TOKEN");
    }
    ParseResult result;
    result.options = std::move(options);
    return result;
}

}  // namespace mcp_ros2
