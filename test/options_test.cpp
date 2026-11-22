#include "options.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

namespace mcp_ros2 {
namespace {

ParseResult parse(std::vector<std::string> args) {
    args.insert(args.begin(), "mcp_ros2");
    return parse_options(args);
}

TEST(OptionsTest, DefaultsToStdioReadWrite) {
    const auto result = parse({});
    ASSERT_TRUE(result.options);
    EXPECT_EQ(result.options->transport, Transport::eStdio);
    EXPECT_FALSE(result.options->read_only);
    EXPECT_EQ(result.options->node_name, "mcp_ros2");
    EXPECT_EQ(result.options->max_array, 64U);
}

TEST(OptionsTest, ParsesEveryOption) {
    const auto result =
        parse({"--read-only", "--node-name=agent_bridge", "--max-array=8", "--transport=http",
               "--host=127.0.0.2", "--port=8080", "--token=secret"});
    ASSERT_TRUE(result.options) << result.message;
    EXPECT_TRUE(result.options->read_only);
    EXPECT_EQ(result.options->node_name, "agent_bridge");
    EXPECT_EQ(result.options->max_array, 8U);
    EXPECT_EQ(result.options->transport, Transport::eHttp);
    EXPECT_EQ(result.options->host, "127.0.0.2");
    EXPECT_EQ(result.options->port, 8080);
    EXPECT_EQ(result.options->token, "secret");
}

TEST(OptionsTest, ParsesDeploymentOptions) {
    const auto result =
        parse({"--allow=/robot1/**", "--allow=/cmd_vel", "--deny=/robot1/arm/**", "--discovery-wait=5",
               "--resource-wait=0.5", "--notify-interval=0.2", "--transport=http", "--host=0.0.0.0",
               "--tls-cert=cert.pem", "--tls-key=key.pem", "--tls-client-ca=ca1.pem",
               "--tls-client-ca=ca2.pem"});
    ASSERT_TRUE(result.options) << result.message;
    EXPECT_EQ(result.options->allow, (std::vector<std::string>{"/robot1/**", "/cmd_vel"}));
    EXPECT_EQ(result.options->deny, (std::vector<std::string>{"/robot1/arm/**"}));
    EXPECT_DOUBLE_EQ(result.options->discovery_wait, 5.0);
    EXPECT_DOUBLE_EQ(result.options->resource_wait, 0.5);
    EXPECT_DOUBLE_EQ(result.options->notify_interval, 0.2);
    EXPECT_EQ(result.options->tls_certificate, "cert.pem");
    EXPECT_EQ(result.options->tls_key, "key.pem");
    EXPECT_EQ(result.options->tls_client_cas.size(), 2U);
}

TEST(OptionsTest, ChecksTheTlsOptionsFitTogether) {
    EXPECT_FALSE(parse({"--transport=http", "--tls-cert=cert.pem"}).options);
    EXPECT_FALSE(parse({"--transport=http", "--tls-key=key.pem"}).options);
    EXPECT_FALSE(parse({"--transport=http", "--tls-client-ca=ca.pem"}).options);
    EXPECT_FALSE(parse({"--tls-cert=cert.pem", "--tls-key=key.pem"}).options);
    EXPECT_FALSE(parse({"--token=t"}).options);
    EXPECT_TRUE(parse({"--transport=http", "--tls-cert=cert.pem", "--tls-key=key.pem"}).options);
}

TEST(OptionsTest, RejectsBadValues) {
    for (const char* bad :
         {"--node-name=has-dash", "--node-name=9lives", "--max-array=0", "--port=0",
          "--transport=pigeon", "--host=", "--frobnicate", "--allow=", "--deny=", "--discovery-wait=-1",
          "--discovery-wait=soon", "--resource-wait=301", "--notify-interval=x", "--tls-client-ca="}) {
        const auto result = parse({bad});
        EXPECT_FALSE(result.options) << bad;
        EXPECT_EQ(result.exit_code, 2) << bad;
    }
}

TEST(OptionsTest, NeedsATokenOffLoopback) {
#ifdef _WIN32
    _putenv_s("MCP_ROS2_TOKEN", "");
#else
    ::unsetenv("MCP_ROS2_TOKEN");
#endif
    EXPECT_FALSE(parse({"--transport=http", "--host=0.0.0.0"}).options);
    EXPECT_TRUE(parse({"--transport=http", "--host=0.0.0.0", "--token=t"}).options);
    // Client certificates authenticate callers as well as a token does.
    EXPECT_TRUE(parse({"--transport=http", "--host=0.0.0.0", "--tls-cert=c.pem", "--tls-key=k.pem",
                       "--tls-client-ca=ca.pem"})
                    .options);
    EXPECT_TRUE(parse({"--transport=http", "--host=localhost"}).options);
}

TEST(OptionsTest, HelpAndVersionExitCleanly) {
    const auto help = parse({"--help"});
    EXPECT_FALSE(help.options);
    EXPECT_EQ(help.exit_code, 0);
    EXPECT_NE(help.message.find("--read-only"), std::string::npos);
    const auto version = parse({"--version"});
    EXPECT_EQ(version.exit_code, 0);
    EXPECT_EQ(version.message.rfind("mcp-ros2 ", 0), 0U);
}

}  // namespace
}  // namespace mcp_ros2
