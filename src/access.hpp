#pragma once

// Which ROS names the tools that act may touch: the topics they publish on, the services they
// call, the actions they send goals to and the nodes whose parameters they set.

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mcp_ros2 {

/// A name the access rules leave out.
class AccessDenied : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

/// Whether the fully qualified @p name matches @p pattern, in which `*` stands for any characters
/// within one part of a name and `**` for any characters across parts: `/robot1/*` matches
/// `/robot1/cmd_vel`, and `/robot1/**` also matches `/robot1/arm/joint_states`.
bool name_matches(std::string_view pattern, std::string_view name);

class AccessRules {
   public:
    AccessRules() = default;
    /// Patterns as name_matches() takes them; a pattern without a leading slash gets one.
    AccessRules(std::vector<std::string> allow, std::vector<std::string> deny);

    /// Whether @p name matches an allowed pattern, or none are given, and no denied one.
    [[nodiscard]] bool permits(const std::string& name) const;

    /// Throws AccessDenied, saying what was asked, when @p name is not permitted. @p doing is
    /// what the tool does to it, such as "publish on".
    void check(const std::string& doing, const std::string& name) const;

    /// Whether any pattern is given.
    [[nodiscard]] bool limited() const { return !allow_.empty() || !deny_.empty(); }

    /// The rules in a sentence, for the server's instructions.
    [[nodiscard]] std::string describe() const;

   private:
    std::vector<std::string> allow_;
    std::vector<std::string> deny_;
};

}  // namespace mcp_ros2
