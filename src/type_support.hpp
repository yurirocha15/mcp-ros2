#pragma once

// ROS 2 interface types loaded by name at run time, and messages of those types in memory.

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>
#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace mcp_ros2 {

/// A type name that does not name an installed interface.
class TypeError : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;
using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;
using ServiceMembers = rosidl_typesupport_introspection_cpp::ServiceMembers;

/// A message type: its C++ type support, for serialization, and its layout, for reading and
/// writing fields.
struct MessageType {
    std::string name;  ///< As `package/msg/Name`.
    std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
    std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
    const rosidl_message_type_support_t* cpp = nullptr;
    const MessageMembers* members = nullptr;
};

/// A service type, with the layouts of its request and response.
struct ServiceType {
    std::string name;  ///< As `package/srv/Name`.
    std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
    const ServiceMembers* members = nullptr;
};

/// An action type, through the message types it is made of.
struct ActionType {
    std::string name;  ///< As `package/action/Name`.
    std::shared_ptr<const MessageType> goal;
    std::shared_ptr<const MessageType> result;
    std::shared_ptr<const MessageType> feedback;
    std::shared_ptr<const MessageType> send_goal_request;  ///< The goal with its ID.
};

/// Loads interface types by name, once each. Thread-safe.
class TypeLibrary {
   public:
    /// @p name as `package/msg/Name`, or `package/Name`.
    std::shared_ptr<const MessageType> message(const std::string& name);
    /// @p name as `package/srv/Name`, or `package/Name`.
    std::shared_ptr<const ServiceType> service(const std::string& name);
    /// @p name as `package/action/Name`, or `package/Name`.
    std::shared_ptr<const ActionType> action(const std::string& name);

   private:
    /// @p full as `package/kind/Name`, where kind may be `action` for the messages of an action.
    std::shared_ptr<const MessageType> message_of(const std::string& full);

    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<const MessageType>> messages_;
    std::map<std::string, std::shared_ptr<const ServiceType>> services_;
    std::map<std::string, std::shared_ptr<const ActionType>> actions_;
};

/// @p name with its interface kind (`msg`, `srv` or `action`) filled in when it is left out.
/// Throws TypeError when it is not `package/Name` or `package/kind/Name`.
std::string full_type_name(const std::string& name, const std::string& kind);

/// The name of the type @p members describes, as `package/kind/Name`.
std::string type_name(const MessageMembers& members);

/// A message of a type known by its layout, in memory and initialized to its defaults.
class DynamicMessage {
   public:
    explicit DynamicMessage(const MessageMembers& members);
    ~DynamicMessage();
    DynamicMessage(const DynamicMessage&) = delete;
    DynamicMessage& operator=(const DynamicMessage&) = delete;

    [[nodiscard]] void* data() { return data_; }
    [[nodiscard]] const void* data() const { return data_; }
    [[nodiscard]] const MessageMembers& members() const { return members_; }

   private:
    const MessageMembers& members_;
    void* data_;
};

}  // namespace mcp_ros2
