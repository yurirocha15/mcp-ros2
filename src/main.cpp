#include "options.hpp"
#include "ros_bridge.hpp"
#include "server.hpp"
#include "version.hpp"

#include <mcp/mcp.hpp>
#include <mcp/transport/http_session_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <vector>

namespace {

/// A buffered output stream over a file descriptor.
class DescriptorBuffer : public std::streambuf {
   public:
    explicit DescriptorBuffer(int fd) : fd_(fd) { setp(buffer_, buffer_ + sizeof buffer_); }
    ~DescriptorBuffer() override { flush(); }
    DescriptorBuffer(const DescriptorBuffer&) = delete;
    DescriptorBuffer& operator=(const DescriptorBuffer&) = delete;

   protected:
    int_type overflow(int_type c) override {
        if (!flush()) {
            return traits_type::eof();
        }
        if (!traits_type::eq_int_type(c, traits_type::eof())) {
            *pptr() = traits_type::to_char_type(c);
            pbump(1);
        }
        return traits_type::not_eof(c);
    }

    int sync() override { return flush() ? 0 : -1; }

   private:
    bool flush() {
        const char* data = pbase();
        while (data < pptr()) {
            const ssize_t written = ::write(fd_, data, static_cast<std::size_t>(pptr() - data));
            if (written < 0) {
                if (errno == EINTR) {
                    continue;
                }
                return false;
            }
            data += written;
        }
        setp(buffer_, buffer_ + sizeof buffer_);
        return true;
    }

    int fd_;
    char buffer_[1 << 16];
};

/// The protocol's own copy of standard output, with standard output itself sent to standard
/// error, so that what the middleware and other libraries print cannot corrupt the message stream.
class ProtocolOutput {
   public:
    ProtocolOutput() : fd_(::dup(1)) {
        std::fflush(stdout);
        if (fd_ < 0 || ::dup2(2, 1) < 0) {
            throw std::runtime_error("cannot set standard output aside for the protocol");
        }
        buffer_ = std::make_unique<DescriptorBuffer>(fd_);
        stream_ = std::make_unique<std::ostream>(buffer_.get());
    }

    std::ostream& stream() { return *stream_; }

   private:
    int fd_;
    std::unique_ptr<DescriptorBuffer> buffer_;
    std::unique_ptr<std::ostream> stream_;
};

int run(const mcp_ros2::Options& options) {
    std::unique_ptr<ProtocolOutput> protocol;
    if (options.transport == mcp_ros2::Transport::eStdio) {
        protocol = std::make_unique<ProtocolOutput>();
    }

    // Destroyed in reverse order: the tools drop their subscriptions, the server stops, its
    // handler thread ends, and the node stops spinning last.
    mcp_ros2::Ros ros(options.node_name, mcp_ros2::JsonLimits{options.max_array});

    // The handlers are coroutines that wait for the node's thread on timers, so one thread serves
    // them all.
    mcp::Runtime handlers;
    handlers.start(1);

    mcp::Server server("mcp-ros2", mcp_ros2::g_VERSION);
    server.set_handler_executor(handlers.executor());
    const mcp_ros2::RosTools tools(
        server, ros, options, [&handlers](mcp::Task<void> task) { handlers.spawn(std::move(task)); });

    if (protocol) {
        server.run_stdio(std::cin, protocol->stream());
    } else {
        std::cerr << "mcp-ros2 listening on http://" << options.host << ":" << options.port << "/mcp\n";
        const std::string token = options.token;
        server.run_http(options.host, options.port, [&token](mcp::StreamableHttpSessionManager& http) {
            if (!token.empty()) {
                http.set_bearer_token_validator(
                    [token](std::string_view sent) { return sent == token; });
            }
        });
    }
    return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        // A signal ends the process at once, as for any stdio server; rclcpp's handlers would only
        // stop ROS and leave the protocol running.
        rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    } catch (const std::exception& error) {
        std::cerr << "mcp-ros2: " << error.what() << "\n";
        return 2;
    }
    const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    const auto parsed = mcp_ros2::parse_options(args);
    if (!parsed.options) {
        (parsed.exit_code == 0 ? std::cout : std::cerr) << parsed.message;
        rclcpp::shutdown();
        return parsed.exit_code;
    }

    int code = EXIT_FAILURE;
    try {
        code = run(*parsed.options);
    } catch (const std::exception& error) {
        std::cerr << "mcp-ros2: " << error.what() << "\n";
    }
    rclcpp::shutdown();
    return code;
}
