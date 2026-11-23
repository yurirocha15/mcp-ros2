#pragma once

// Conversion between messages of any type and JSON, using their introspection layouts.

#include "type_support.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

namespace mcp_ros2 {

/// Thrown when JSON does not fit a message type; the error message names the field.
class MessageError : public std::invalid_argument {
   public:
    using std::invalid_argument::invalid_argument;
};

struct JsonLimits {
    /// Arrays longer than this become `{"length": n, "head": [the first values]}`.
    std::size_t max_array = 64;
};

/// The message of type @p members at @p data as a JSON object.
nlohmann::json to_json(const MessageMembers& members, const void* data, const JsonLimits& limits = {});

/// A time, in seconds and nanoseconds, for `"now"` in a builtin_interfaces/msg/Time field.
struct Stamp {
    std::int32_t sec = 0;
    std::uint32_t nanosec = 0;
};

/// Sets the fields of the message of type @p members at @p data from the JSON object @p value.
/// Fields it leaves out keep their values. A builtin_interfaces/msg/Time field may be given as
/// `"now"`, which @p now supplies. Throws MessageError for a field that does not exist or a value
/// of the wrong kind.
void from_json(const nlohmann::json& value, const MessageMembers& members, void* data,
               const std::function<Stamp()>& now);

/// The fields of the type @p members, as {field: type}, nested messages as objects of their own.
nlohmann::json describe(const MessageMembers& members);

}  // namespace mcp_ros2
