#pragma once

// The MCP side: the tools, resources and instructions that put the ROS graph in front of an agent.

#include "options.hpp"
#include "ros_bridge.hpp"

#include <mcp/core/async.hpp>
#include <mcp/mcp.hpp>

#include <functional>
#include <memory>

namespace mcp_ros2 {

/// Runs a task on the handlers' event loop, from any thread.
using Spawn = std::function<void(mcp::Task<void>)>;

/// The tools and resources of one server. Must be destroyed before @p ros, and lives while the
/// server runs.
class RosTools {
   public:
    RosTools(mcp::Server& server, Ros& ros, const Options& options, Spawn spawn);
    ~RosTools();
    RosTools(const RosTools&) = delete;
    RosTools& operator=(const RosTools&) = delete;

    /// What the tools share.
    struct Shared;
    /// The topics clients subscribed to, with what is needed to tell them of new messages.
    struct Watches;

   private:
    std::shared_ptr<Shared> shared_;
    std::shared_ptr<Watches> watches_;
};

std::string server_instructions(const Options& options);

}  // namespace mcp_ros2
