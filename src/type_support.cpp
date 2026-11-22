#include "type_support.hpp"

#include <rclcpp/typesupport_helpers.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <cstddef>
#include <new>
#include <vector>

namespace mcp_ros2 {

namespace {

constexpr const char* g_CPP = "rosidl_typesupport_cpp";
constexpr const char* g_INTROSPECTION = "rosidl_typesupport_introspection_cpp";

std::vector<std::string> split(const std::string& text, char separator) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (true) {
        const std::size_t end = text.find(separator, start);
        parts.push_back(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
        if (end == std::string::npos) {
            return parts;
        }
        start = end + 1;
    }
}

std::shared_ptr<rcpputils::SharedLibrary> library(const std::string& type, const char* typesupport) {
    try {
        return rclcpp::get_typesupport_library(type, typesupport);
    } catch (const std::exception& error) {
        throw TypeError(type +
                        " is not installed, or its package is not in the environment: " + error.what());
    }
}

std::shared_ptr<const MessageType> load_message(const std::string& name) {
    auto type = std::make_shared<MessageType>();
    type->name = name;
    type->cpp_library = library(name, g_CPP);
    type->introspection_library = library(name, g_INTROSPECTION);
    try {
        type->cpp = rclcpp::get_message_typesupport_handle(name, g_CPP, *type->cpp_library);
        const auto* introspection =
            rclcpp::get_message_typesupport_handle(name, g_INTROSPECTION, *type->introspection_library);
        type->members = static_cast<const MessageMembers*>(introspection->data);
    } catch (const std::exception& error) {
        throw TypeError("there is no message type " + name + ": " + error.what());
    }
    return type;
}

}  // namespace

std::string full_type_name(const std::string& name, const std::string& kind) {
    const std::vector<std::string> parts = split(name, '/');
    for (const std::string& part : parts) {
        if (part.empty()) {
            throw TypeError("'" + name + "' is not a type name such as std_msgs/msg/String");
        }
    }
    if (parts.size() == 2) {
        return parts[0] + "/" + kind + "/" + parts[1];
    }
    if (parts.size() == 3 && parts[1] == kind) {
        return name;
    }
    throw TypeError("'" + name + "' is not a " + kind + " type name such as package/" + kind + "/Name");
}

std::string type_name(const MessageMembers& members) {
    std::string space = members.message_namespace_;
    for (std::size_t at = space.find("::"); at != std::string::npos; at = space.find("::")) {
        space.replace(at, 2, "/");
    }
    return space + "/" + members.message_name_;
}

std::shared_ptr<const MessageType> TypeLibrary::message(const std::string& name) {
    return message_of(full_type_name(name, "msg"));
}

std::shared_ptr<const MessageType> TypeLibrary::message_of(const std::string& full) {
    const std::lock_guard lock(mutex_);
    auto& slot = messages_[full];
    if (!slot) {
        slot = load_message(full);
    }
    return slot;
}

std::shared_ptr<const ServiceType> TypeLibrary::service(const std::string& name) {
    const std::string full = full_type_name(name, "srv");
    const std::lock_guard lock(mutex_);
    auto& slot = services_[full];
    if (!slot) {
        auto type = std::make_shared<ServiceType>();
        type->name = full;
        type->introspection_library = library(full, g_INTROSPECTION);
        try {
            const auto* introspection = rclcpp::get_service_typesupport_handle(
                full, g_INTROSPECTION, *type->introspection_library);
            type->members = static_cast<const ServiceMembers*>(introspection->data);
        } catch (const std::exception& error) {
            throw TypeError("there is no service type " + full + ": " + error.what());
        }
        slot = type;
    }
    return slot;
}

std::shared_ptr<const ActionType> TypeLibrary::action(const std::string& name) {
    const std::string full = full_type_name(name, "action");
    {
        const std::lock_guard lock(mutex_);
        if (const auto found = actions_.find(full); found != actions_.end()) {
            return found->second;
        }
    }
    // The messages an action is made of are generated as messages of its package's action
    // namespace, such as example_interfaces/action/Fibonacci_Goal.
    auto type = std::make_shared<ActionType>();
    type->name = full;
    try {
        type->goal = message_of(full + "_Goal");
        type->result = message_of(full + "_Result");
        type->feedback = message_of(full + "_Feedback");
        type->send_goal_request = message_of(full + "_SendGoal_Request");
    } catch (const TypeError&) {
        throw TypeError(full + " is not an installed action type");
    }
    const std::lock_guard lock(mutex_);
    actions_[full] = type;
    return type;
}

DynamicMessage::DynamicMessage(const MessageMembers& members)
    : members_(members),
      data_(::operator new(members.size_of_, std::align_val_t{alignof(std::max_align_t)})) {
    try {
        members_.init_function(data_, rosidl_runtime_cpp::MessageInitialization::ALL);
    } catch (...) {
        ::operator delete(data_, std::align_val_t{alignof(std::max_align_t)});
        throw;
    }
}

DynamicMessage::~DynamicMessage() {
    members_.fini_function(data_);
    ::operator delete(data_, std::align_val_t{alignof(std::max_align_t)});
}

}  // namespace mcp_ros2
