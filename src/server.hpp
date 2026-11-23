#pragma once

// MCP tools, resources and server instructions backed by the ROS bridge (ros_bridge.hpp).

#include "options.hpp"
#include "ros_bridge.hpp"

#include <mcp/core/async.hpp>
#include <mcp/mcp.hpp>

#include <functional>
#include <memory>

namespace mcp_ros2 {

/// Schedules a coroutine on the handler event loop; callable from any thread.
using Spawn = std::function<void(mcp::Task<void>)>;

/// Registers the ROS tools and resources on an mcp::Server. Must stay alive while the server runs
/// and be destroyed before @p ros.
class RosTools {
   public:
    RosTools(mcp::Server& server, Ros& ros, const Options& options, Spawn spawn);
    ~RosTools();
    RosTools(const RosTools&) = delete;
    RosTools& operator=(const RosTools&) = delete;

    /// State shared by the tool handlers.
    struct Shared;
    /// Subscribed topic resources and the listeners that send their update notifications.
    struct Watches;

   private:
    std::shared_ptr<Shared> shared_;
    std::shared_ptr<Watches> watches_;
};

std::string server_instructions(const Options& options);

}  // namespace mcp_ros2
