#pragma once

// The server's own ROS node, and what it does on the ROS graph: looking at nodes, topics,
// services and actions, and using them with messages of any installed type. Calls that wait for
// another node start the work and return a handle to check; the caller does the waiting.

#include "message_json.hpp"
#include "type_support.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_generic_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace tf2_ros {
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace mcp_ros2 {

/// Something that cannot be done on the ROS graph, such as a topic nobody publishes without a
/// type to use, or a node that does not exist. The message says what to do instead.
class RosError : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

/// QoS settings a caller may give for a subscription or publisher.
struct QosRequest {
    std::optional<std::string> reliability;  ///< "reliable" or "best_effort".
    std::optional<std::string> durability;   ///< "volatile" or "transient_local".
    std::optional<std::size_t> depth;
};

/// Messages of one topic as they arrive, kept serialized until they are read.
class TopicTap {
   public:
    struct Sample {
        std::chrono::steady_clock::time_point received;
        std::shared_ptr<rclcpp::SerializedMessage> message;  ///< Null when the tap keeps no messages.
        std::size_t bytes = 0;
    };

    /// The number of samples received so far.
    [[nodiscard]] std::size_t count() const;
    /// The samples received so far, oldest first; at most the capacity the tap was made with.
    [[nodiscard]] std::vector<Sample> samples() const;
    /// The newest sample, if any.
    [[nodiscard]] std::optional<Sample> latest() const;
    /// Calls @p listener, on the ROS thread, after each message. Replaces the previous listener.
    void on_message(std::function<void()> listener);

    [[nodiscard]] const std::string& topic() const { return topic_; }
    [[nodiscard]] const std::shared_ptr<const MessageType>& type() const { return type_; }

   private:
    friend class Ros;
    void add(std::shared_ptr<rclcpp::SerializedMessage> message);

    std::string topic_;
    std::shared_ptr<const MessageType> type_;
    std::size_t capacity_ = 0;
    bool keep_messages_ = true;
    mutable std::mutex mutex_;
    std::deque<Sample> samples_;
    std::size_t count_ = 0;
    std::function<void()> listener_;
    rclcpp::GenericSubscription::SharedPtr subscription_;
};

/// A service request on its way.
struct ServiceCall {
    rclcpp::GenericClient::SharedPtr client;
    std::shared_ptr<const ServiceType> type;
    std::shared_future<std::shared_ptr<void>> response;
    std::int64_t request_id = 0;
};

/// An action goal sent, and what has come back for it so far.
class ActionGoal {
   public:
    /// Whether the server has answered the goal request.
    [[nodiscard]] bool answered() const;
    /// Whether the server accepted the goal; false before it answers.
    [[nodiscard]] bool accepted() const;
    /// Whether the goal has finished, with a result.
    [[nodiscard]] bool finished() const;
    [[nodiscard]] std::size_t feedback_count() const;
    /// The newest feedback, or null.
    [[nodiscard]] nlohmann::json last_feedback() const;
    /// "succeeded", "aborted", "canceled" or "unknown", once finished.
    [[nodiscard]] std::string status() const;
    /// The result, once finished.
    [[nodiscard]] nlohmann::json result() const;
    /// Asks the server to cancel the goal, once it is accepted.
    void cancel();

   private:
    friend class Ros;
    struct State {
        mutable std::mutex mutex;
        bool answered = false;
        rclcpp_action::GenericClientGoalHandle::SharedPtr handle;
        std::size_t feedback_count = 0;
        nlohmann::json last_feedback;
        bool finished = false;
        std::string status;
        nlohmann::json result;
    };
    rclcpp_action::GenericClient::SharedPtr client_;
    std::shared_ptr<const ActionType> type_;
    std::shared_ptr<State> state_ = std::make_shared<State>();
    std::shared_future<rclcpp_action::GenericClientGoalHandle::SharedPtr> handle_;
};

/// A parameter request to another node on its way, with the client that waits for it.
template <typename T>
struct ParameterCall {
    std::shared_ptr<rclcpp::AsyncParametersClient> client;
    std::shared_future<T> future;
};

/// The node, spun on a thread of its own while this lives. rclcpp must be initialized first.
class Ros {
   public:
    Ros(const std::string& node_name, JsonLimits limits);
    ~Ros();
    Ros(const Ros&) = delete;
    Ros& operator=(const Ros&) = delete;

    [[nodiscard]] const JsonLimits& limits() const { return limits_; }
    /// When the node started, from which discovery fills in the graph.
    [[nodiscard]] std::chrono::steady_clock::time_point started() const { return started_; }
    [[nodiscard]] std::string node_name() const;
    /// The node's clock now, which is simulation time when the node uses it.
    [[nodiscard]] Stamp now() const;
    TypeLibrary& types() { return types_; }

    // The graph. Names are fully qualified, such as /robot1/odom.

    nlohmann::json nodes() const;
    /// With @p hidden, also the topics and services under names starting with an underscore, such
    /// as those that make up actions.
    nlohmann::json topics(bool hidden) const;
    /// With @p all, also every node's parameter and type description services.
    nlohmann::json services(bool hidden, bool all) const;
    nlohmann::json actions() const;
    nlohmann::json node_info(const std::string& node) const;
    nlohmann::json topic_info(const std::string& topic) const;
    /// The fields of a message, service or action type.
    nlohmann::json describe_interface(const std::string& type);

    // Topics.

    /// The message type of @p topic: @p given when set, else the one type its publishers and
    /// subscribers use.
    std::shared_ptr<const MessageType> topic_type(const std::string& topic,
                                                  const std::optional<std::string>& given);
    /// Subscribes to @p topic. Keeps the newest @p capacity messages, or none, only counting
    /// them and their sizes, when @p keep_messages is false.
    std::shared_ptr<TopicTap> tap(const std::string& topic, std::shared_ptr<const MessageType> type,
                                  std::size_t capacity, bool keep_messages, const QosRequest& qos);
    /// The message in @p sample as JSON, with arrays shortened to @p limits.
    nlohmann::json message_json(const TopicTap& tap, const TopicTap::Sample& sample,
                                const JsonLimits& limits) const;
    /// A publisher for @p topic, reused for later calls with the same type and QoS.
    rclcpp::GenericPublisher::SharedPtr publisher(const std::string& topic,
                                                  const std::shared_ptr<const MessageType>& type,
                                                  const QosRequest& qos);
    /// @p message, of type @p type, serialized for a publisher.
    rclcpp::SerializedMessage serialize(const MessageType& type, const nlohmann::json& message);

    // Services and actions.

    /// The type of @p service: @p given when set, else the one the graph knows.
    std::shared_ptr<const ServiceType> service_type(const std::string& service,
                                                    const std::optional<std::string>& given);
    /// A client for @p service, reused for later calls.
    rclcpp::GenericClient::SharedPtr service_client(const std::string& service,
                                                    const ServiceType& type);
    /// Sends @p request through @p client, which must be ready.
    ServiceCall call(rclcpp::GenericClient::SharedPtr client, std::shared_ptr<const ServiceType> type,
                     const nlohmann::json& request);
    nlohmann::json response_json(const ServiceCall& call) const;

    /// The type of @p action: @p given when set, else the one the graph knows.
    std::shared_ptr<const ActionType> action_type(const std::string& action,
                                                  const std::optional<std::string>& given);
    /// A client for @p action, reused for later goals.
    rclcpp_action::GenericClient::SharedPtr action_client(const std::string& action,
                                                          const ActionType& type);
    /// Sends @p goal through @p client, whose server must be ready.
    std::shared_ptr<ActionGoal> send_goal(rclcpp_action::GenericClient::SharedPtr client,
                                          std::shared_ptr<const ActionType> type,
                                          const nlohmann::json& goal);

    // Parameters of other nodes.

    /// A parameter client for @p node, reused for later calls.
    std::shared_ptr<rclcpp::AsyncParametersClient> parameter_client(const std::string& node);
    ParameterCall<rcl_interfaces::msg::ListParametersResult> list_parameters(
        std::shared_ptr<rclcpp::AsyncParametersClient> client);
    ParameterCall<std::vector<rclcpp::Parameter>> get_parameters(
        std::shared_ptr<rclcpp::AsyncParametersClient> client, const std::vector<std::string>& names);
    ParameterCall<std::vector<rcl_interfaces::msg::SetParametersResult>> set_parameters(
        std::shared_ptr<rclcpp::AsyncParametersClient> client,
        const std::vector<rclcpp::Parameter>& parameters);

    // Transforms.

    /// Starts listening to /tf and /tf_static, once.
    void listen_to_transforms();
    /// The transform from @p source to @p target frame, newest available, if there is one.
    std::optional<nlohmann::json> transform(const std::string& target, const std::string& source);
    /// The frames heard of so far.
    std::vector<std::string> frames();

   private:
    rclcpp::QoS subscription_qos(const std::string& topic, std::size_t depth,
                                 const QosRequest& request) const;
    rclcpp::QoS publisher_qos(const std::string& topic, const QosRequest& request) const;
    std::vector<std::string> graph_types(const std::map<std::string, std::vector<std::string>>& names,
                                         const std::string& name) const;

    JsonLimits limits_;
    std::chrono::steady_clock::time_point started_ = std::chrono::steady_clock::now();
    TypeLibrary types_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spinner_;

    std::mutex mutex_;
    std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
    std::map<std::string, rclcpp::GenericClient::SharedPtr> service_clients_;
    std::map<std::string, rclcpp_action::GenericClient::SharedPtr> action_clients_;
    std::map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> parameter_clients_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/// @p name as a fully qualified ROS name: with a leading slash, and no trailing one.
std::string qualified_name(const std::string& name);

/// A parameter value as JSON: null when it is not set.
nlohmann::json parameter_json(const rclcpp::ParameterValue& value);

/// @p value as a parameter value of type @p type, or of the type its JSON kind implies when
/// @p type is PARAMETER_NOT_SET. Throws std::invalid_argument when it does not fit.
rclcpp::ParameterValue parameter_value(const nlohmann::json& value, rclcpp::ParameterType type);

}  // namespace mcp_ros2
