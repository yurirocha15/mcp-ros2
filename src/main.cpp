#include "options.hpp"
#include "ros_bridge.hpp"
#include "server.hpp"
#include "version.hpp"

#include <mcp/mcp.hpp>
#include <mcp/transport/http_session_manager.hpp>
#include <mcp/transport/tls.hpp>
#include <rclcpp/rclcpp.hpp>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#else
#include <unistd.h>
#endif

#include <algorithm>
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

#ifdef _WIN32
int duplicate(int fd) { return _dup(fd); }
int replace(int from, int to) { return _dup2(from, to); }
long long write_some(int fd, const char* data, std::size_t size) {
    return _write(fd, data, static_cast<unsigned int>(std::min<std::size_t>(size, 1U << 30)));
}
/// Messages end with a bare newline, not the CRLF a text-mode descriptor writes.
void binary(int fd) { _setmode(fd, _O_BINARY); }
#else
int duplicate(int fd) { return ::dup(fd); }
int replace(int from, int to) { return ::dup2(from, to); }
long long write_some(int fd, const char* data, std::size_t size) { return ::write(fd, data, size); }
void binary(int /*fd*/) {}
#endif

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
            const long long written = write_some(fd_, data, static_cast<std::size_t>(pptr() - data));
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
    ProtocolOutput() : fd_(duplicate(1)) {
        std::fflush(stdout);
        if (fd_ < 0 || replace(2, 1) < 0) {
            throw std::runtime_error("cannot set standard output aside for the protocol");
        }
        binary(fd_);
        buffer_ = std::make_unique<DescriptorBuffer>(fd_);
        stream_ = std::make_unique<std::ostream>(buffer_.get());
    }

    std::ostream& stream() { return *stream_; }

   private:
    int fd_;
    std::unique_ptr<DescriptorBuffer> buffer_;
    std::unique_ptr<std::ostream> stream_;
};

/// Whether @p sent is @p token, compared in a time that does not depend on where they differ.
bool same_secret(std::string_view sent, std::string_view token) {
    unsigned char difference = sent.size() == token.size() ? 0 : 1;
    for (std::size_t i = 0; i < sent.size(); ++i) {
        difference |=
            static_cast<unsigned char>(sent[i] ^ token[i % std::max<std::size_t>(token.size(), 1)]);
    }
    return difference == 0 && !token.empty();
}

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
        const bool tls = !options.tls_certificate.empty();
        std::cerr << "mcp-ros2 listening on " << (tls ? "https" : "http") << "://" << options.host
                  << ":" << options.port << "/mcp\n";
        server.run_http(
            options.host, options.port, [&options, tls](mcp::StreamableHttpSessionManager& http) {
                if (!options.token.empty()) {
                    http.set_bearer_token_validator([token = options.token](std::string_view sent) {
                        return same_secret(sent, token);
                    });
                }
                if (tls) {
                    mcp::TlsServerOptions settings;
                    settings.certificate_chain_file = options.tls_certificate;
                    settings.private_key_file = options.tls_key;
                    if (!options.tls_client_cas.empty()) {
                        settings.client_auth = mcp::TlsServerOptions::ClientAuth::eRequired;
                        settings.client_ca_files = options.tls_client_cas;
                    }
                    http.set_tls(settings);
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
