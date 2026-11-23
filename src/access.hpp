#pragma once

// Allow and deny rules for the names used by the tools that change the system: topics published
// on, services called, actions sent goals and nodes whose parameters are set.

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace mcp_ros2 {

/// Thrown when the access rules do not permit a name.
class AccessDenied : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

/// Whether the fully qualified @p name matches @p pattern. `*` matches any characters within one
/// name segment and `**` any characters across segments: `/robot1/*` matches `/robot1/cmd_vel`;
/// `/robot1/**` also matches `/robot1/arm/joint_states`.
bool name_matches(std::string_view pattern, std::string_view name);

class AccessRules {
   public:
    AccessRules() = default;
    /// @p allow and @p deny hold name_matches() patterns; a missing leading slash is added.
    AccessRules(std::vector<std::string> allow, std::vector<std::string> deny);

    /// True when @p name matches an allow pattern (or none is set) and no deny pattern.
    [[nodiscard]] bool permits(const std::string& name) const;

    /// Throws AccessDenied when @p name is not permitted. @p doing names the operation for the
    /// error message, such as "publish on".
    void check(const std::string& doing, const std::string& name) const;

    /// True when any allow or deny pattern is set.
    [[nodiscard]] bool limited() const { return !allow_.empty() || !deny_.empty(); }

    /// One-sentence summary of the rules, for the server instructions and error messages.
    [[nodiscard]] std::string describe() const;

   private:
    std::vector<std::string> allow_;
    std::vector<std::string> deny_;
};

}  // namespace mcp_ros2
