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

TEST(OptionsTest, RejectsBadValues) {
    for (const char* bad : {"--node-name=has-dash", "--node-name=9lives", "--max-array=0", "--port=0",
                            "--transport=pigeon", "--host=", "--frobnicate"}) {
        const auto result = parse({bad});
        EXPECT_FALSE(result.options) << bad;
        EXPECT_EQ(result.exit_code, 2) << bad;
    }
}

TEST(OptionsTest, NeedsATokenOffLoopback) {
    ::unsetenv("MCP_ROS2_TOKEN");
    EXPECT_FALSE(parse({"--transport=http", "--host=0.0.0.0"}).options);
    EXPECT_TRUE(parse({"--transport=http", "--host=0.0.0.0", "--token=t"}).options);
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
