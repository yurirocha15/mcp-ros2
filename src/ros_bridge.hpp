#pragma once

// The server's ROS node: graph queries, topic subscriptions and publishers, service calls, action
// goals, parameters and transforms, for any installed interface type. Operations that depend on
// another node return a handle (ServiceCall, ActionGoal, ParameterCall) that the caller polls.

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

/// Thrown for a request the ROS graph cannot serve, such as an unknown node, or a topic with no
/// publishers and no given type. The error message suggests what to do instead.
class RosError : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

/// Optional QoS overrides for a subscription or publisher.
struct QosRequest {
    std::optional<std::string> reliability;  ///< "reliable" or "best_effort".
    std::optional<std::string> durability;   ///< "volatile" or "transient_local".
    std::optional<std::size_t> depth;
};

/// A subscription that buffers a topic's messages in serialized form until they are read.
class TopicTap {
   public:
    struct Sample {
        std::chrono::steady_clock::time_point received;
        std::shared_ptr<rclcpp::SerializedMessage> message;  ///< Null when the tap keeps no messages.
        std::size_t bytes = 0;
    };

    /// The number of samples received so far.
    [[nodiscard]] std::size_t count() const;
    /// The buffered samples, oldest first; at most the tap's capacity.
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

/// A pending service request.
struct ServiceCall {
    rclcpp::GenericClient::SharedPtr client;
    std::shared_ptr<const ServiceType> type;
    std::shared_future<std::shared_ptr<void>> response;
    std::int64_t request_id = 0;
};

/// A sent action goal and the responses received for it.
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
    /// "sent", "accepted", "executing" or "canceling" before the goal finishes; status() after.
    [[nodiscard]] std::string current_status() const;
    /// The goal's ID, as a UUID.
    [[nodiscard]] const std::string& id() const { return id_; }
    /// The result, once finished.
    [[nodiscard]] nlohmann::json result() const;
    /// Requests cancellation of the goal; has no effect before the goal is accepted.
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
    std::string id_;
    std::shared_ptr<State> state_ = std::make_shared<State>();
    std::shared_future<rclcpp_action::GenericClientGoalHandle::SharedPtr> handle_;
};

/// A pending parameter request, with the client that must outlive it.
template <typename T>
struct ParameterCall {
    std::shared_ptr<rclcpp::AsyncParametersClient> client;
    std::shared_future<T> future;
};

/// Owns the server's node and spins it on a dedicated thread. Requires rclcpp to be initialized.
class Ros {
   public:
    Ros(const std::string& node_name, JsonLimits limits);
    ~Ros();
    Ros(const Ros&) = delete;
    Ros& operator=(const Ros&) = delete;

    [[nodiscard]] const JsonLimits& limits() const { return limits_; }
    /// Time the node was created, which is when graph discovery starts.
    [[nodiscard]] std::chrono::steady_clock::time_point started() const { return started_; }
    [[nodiscard]] std::string node_name() const;
    /// Current time of the node's clock; simulation time when use_sim_time is set.
    [[nodiscard]] Stamp now() const;
    TypeLibrary& types() { return types_; }

    // The graph. Names are fully qualified, such as /robot1/odom.

    nlohmann::json nodes() const;
    /// @p hidden includes names with a segment starting with an underscore, such as the topics and
    /// services that implement actions.
    nlohmann::json topics(bool hidden) const;
    /// @p all includes the standard per-node parameter and type description services.
    nlohmann::json services(bool hidden, bool all) const;
    nlohmann::json actions() const;
    nlohmann::json node_info(const std::string& node) const;
    nlohmann::json topic_info(const std::string& topic) const;
    /// The fields of a message, service or action type.
    nlohmann::json describe_interface(const std::string& type);

    // Topics.

    /// The message type of @p topic: @p given when set, otherwise the single type its publishers
    /// and subscribers use.
    std::shared_ptr<const MessageType> topic_type(const std::string& topic,
                                                  const std::optional<std::string>& given);
    /// Subscribes to @p topic and keeps the newest @p capacity messages. With @p keep_messages
    /// false, it only counts messages and their sizes.
    std::shared_ptr<TopicTap> tap(const std::string& topic, std::shared_ptr<const MessageType> type,
                                  std::size_t capacity, bool keep_messages, const QosRequest& qos);
    /// The message in @p sample as JSON, with arrays truncated to @p limits.
    nlohmann::json message_json(const TopicTap& tap, const TopicTap::Sample& sample,
                                const JsonLimits& limits) const;
    /// A publisher for @p topic, reused for later calls with the same type and QoS.
    rclcpp::GenericPublisher::SharedPtr publisher(const std::string& topic,
                                                  const std::shared_ptr<const MessageType>& type,
                                                  const QosRequest& qos);
    /// Serializes @p message, of type @p type, for a publisher.
    rclcpp::SerializedMessage serialize(const MessageType& type, const nlohmann::json& message);

    // Services and actions.

    /// The type of @p service: @p given when set, otherwise the type in the graph.
    std::shared_ptr<const ServiceType> service_type(const std::string& service,
                                                    const std::optional<std::string>& given);
    /// A client for @p service, reused for later calls.
    rclcpp::GenericClient::SharedPtr service_client(const std::string& service,
                                                    const ServiceType& type);
    /// Sends @p request through @p client, which must be ready.
    ServiceCall call(rclcpp::GenericClient::SharedPtr client, std::shared_ptr<const ServiceType> type,
                     const nlohmann::json& request);
    nlohmann::json response_json(const ServiceCall& call) const;

    /// The type of @p action: @p given when set, otherwise the type in the graph.
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
    /// The latest available transform from @p source to @p target frame, if any.
    std::optional<nlohmann::json> transform(const std::string& target, const std::string& source);
    /// Frames received on /tf and /tf_static so far.
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
