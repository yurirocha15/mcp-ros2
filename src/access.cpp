#include "access.hpp"

#include <algorithm>
#include <iterator>

namespace mcp_ros2 {

namespace {

std::string qualified(std::string pattern) {
    return pattern.starts_with("/") ? pattern : "/" + pattern;
}

std::string listed(const std::vector<std::string>& patterns) {
    std::string out;
    for (const std::string& pattern : patterns) {
        out += (out.empty() ? "" : ", ") + pattern;
    }
    return out;
}

}  // namespace

bool name_matches(std::string_view pattern, std::string_view name) {
    while (!pattern.empty()) {
        if (pattern.starts_with("**")) {
            const std::string_view rest = pattern.substr(2);
            for (std::size_t skip = 0; skip <= name.size(); ++skip) {
                if (name_matches(rest, name.substr(skip))) {
                    return true;
                }
            }
            return false;
        }
        if (pattern.front() == '*') {
            const std::string_view rest = pattern.substr(1);
            for (std::size_t skip = 0; skip <= name.size(); ++skip) {
                if (name_matches(rest, name.substr(skip))) {
                    return true;
                }
                if (skip < name.size() && name[skip] == '/') {
                    return false;
                }
            }
            return false;
        }
        if (name.empty() || pattern.front() != name.front()) {
            return false;
        }
        pattern.remove_prefix(1);
        name.remove_prefix(1);
    }
    return name.empty();
}

AccessRules::AccessRules(std::vector<std::string> allow, std::vector<std::string> deny) {
    std::transform(allow.begin(), allow.end(), std::back_inserter(allow_), qualified);
    std::transform(deny.begin(), deny.end(), std::back_inserter(deny_), qualified);
}

bool AccessRules::permits(const std::string& name) const {
    const auto matches = [&name](const std::string& pattern) { return name_matches(pattern, name); };
    return (allow_.empty() || std::any_of(allow_.begin(), allow_.end(), matches)) &&
           std::none_of(deny_.begin(), deny_.end(), matches);
}

void AccessRules::check(const std::string& doing, const std::string& name) const {
    if (!permits(name)) {
        throw AccessDenied("This server may not " + doing + " " + name + ". " + describe());
    }
}

std::string AccessRules::describe() const {
    if (!limited()) {
        return "The tools that act may touch any name.";
    }
    std::string out = "The tools that act may only touch ";
    out += allow_.empty() ? std::string("names") : "names matching " + listed(allow_);
    if (!deny_.empty()) {
        out += ", and none matching " + listed(deny_);
    }
    return out + ".";
}

}  // namespace mcp_ros2
