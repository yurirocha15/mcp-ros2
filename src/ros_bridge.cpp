#include "ros_bridge.hpp"

#include <rcl/graph.h>
#include <rcl_action/graph.h>
#include <rmw/qos_string_conversions.h>
#include <action_msgs/msg/goal_status.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <random>
#include <set>

namespace mcp_ros2 {

using nlohmann::json;

namespace {

/// Whether a part of @p name starts with an underscore, which hides it from listings, as the
/// topics and services that make up an action do.
bool is_hidden(const std::string& name) {
    return name.starts_with("_") || name.find("/_") != std::string::npos;
}

/// Whether @p types are those of the services every node offers for its parameters and types.
bool is_node_service(const std::vector<std::string>& types) {
    return !types.empty() && std::all_of(types.begin(), types.end(), [](const std::string& type) {
        return type.starts_with("rcl_interfaces/srv/") ||
               type.starts_with("type_description_interfaces/srv/");
    });
}

std::string join_name(const std::string& space, const std::string& name) {
    return space.empty() || space == "/" ? "/" + name : space + "/" + name;
}

/// @p full split into namespace and name.
std::pair<std::string, std::string> split_node_name(const std::string& full) {
    const std::size_t slash = full.rfind('/');
    return {slash == 0 ? "/" : full.substr(0, slash), full.substr(slash + 1)};
}

/// Takes and frees @p names, as {name: types}.
std::map<std::string, std::vector<std::string>> take_names(rcl_names_and_types_t& names) {
    std::map<std::string, std::vector<std::string>> out;
    for (std::size_t i = 0; i < names.names.size; ++i) {
        auto& types = out[names.names.data[i]];
        for (std::size_t k = 0; k < names.types[i].size; ++k) {
            types.emplace_back(names.types[i].data[k]);
        }
    }
    if (rcl_names_and_types_fini(&names) != RCL_RET_OK) {
        rcl_reset_error();
    }
    return out;
}

json names_json(const std::map<std::string, std::vector<std::string>>& names, bool hidden,
                bool node_services) {
    json out = json::array();
    for (const auto& [name, types] : names) {
        if ((!hidden && is_hidden(name)) || (!node_services && is_node_service(types))) {
            continue;
        }
        out.push_back(json{{"name", name}, {"types", types}});
    }
    return out;
}

std::string policy(const char* name) { return name == nullptr ? "unknown" : name; }

/// @p time in seconds, or null when it is unset or infinite.
json duration_json(const rmw_time_t& time) {
    const rmw_time_t forever = RMW_DURATION_INFINITE;
    const bool infinite = time.sec == forever.sec && time.nsec == forever.nsec;
    if (infinite || (time.sec == 0 && time.nsec == 0)) {
        return nullptr;
    }
    return static_cast<double>(time.sec) + static_cast<double>(time.nsec) * 1e-9;
}

json qos_json(const rclcpp::QoS& qos) {
    const rmw_qos_profile_t& profile = qos.get_rmw_qos_profile();
    json out{{"reliability", policy(rmw_qos_reliability_policy_to_str(profile.reliability))},
             {"durability", policy(rmw_qos_durability_policy_to_str(profile.durability))},
             {"liveliness", policy(rmw_qos_liveliness_policy_to_str(profile.liveliness))}};
    if (const json deadline = duration_json(profile.deadline); !deadline.is_null()) {
        out["deadline"] = deadline;
    }
    if (const json lifespan = duration_json(profile.lifespan); !lifespan.is_null()) {
        out["lifespan"] = lifespan;
    }
    return out;
}

/// Whether the middleware knows the node of @p endpoint yet. Node names arrive a moment after the
/// node's endpoints are discovered; until then they read as placeholders.
bool node_known(const rclcpp::TopicEndpointInfo& endpoint) {
    return endpoint.node_name().find("_NODE_NAME_UNKNOWN_") == std::string::npos;
}

json endpoints_json(const std::vector<rclcpp::TopicEndpointInfo>& endpoints) {
    json out = json::array();
    for (const auto& endpoint : endpoints) {
        out.push_back(json{{"node", node_known(endpoint) ? json(join_name(endpoint.node_namespace(),
                                                                          endpoint.node_name()))
                                                         : json(nullptr)},
                           {"type", endpoint.topic_type()},
                           {"qos", qos_json(endpoint.qos_profile())}});
    }
    return out;
}

void apply(rclcpp::QoS& qos, const QosRequest& request) {
    if (request.reliability) {
        if (*request.reliability == "reliable") {
            qos.reliable();
        } else if (*request.reliability == "best_effort") {
            qos.best_effort();
        } else {
            throw std::invalid_argument("reliability is reliable or best_effort, not " +
                                        *request.reliability);
        }
    }
    if (request.durability) {
        if (*request.durability == "volatile") {
            qos.durability_volatile();
        } else if (*request.durability == "transient_local") {
            qos.transient_local();
        } else {
            throw std::invalid_argument("durability is volatile or transient_local, not " +
                                        *request.durability);
        }
    }
}

std::string single_type(const std::string& kind, const std::string& name,
                        const std::vector<std::string>& types) {
    if (types.size() != 1) {
        std::string list;
        for (const std::string& type : types) {
            list += (list.empty() ? "" : ", ") + type;
        }
        throw RosError(name + " has more than one " + kind + " type (" + list +
                       "); give the one to use");
    }
    return types.front();
}

const MessageMember& member_named(const MessageMembers& members, const char* name) {
    for (std::uint32_t i = 0; i < members.member_count_; ++i) {
        if (std::string_view(members.members_[i].name_) == name) {
            return members.members_[i];
        }
    }
    throw TypeError(type_name(members) + " has no field " + name);
}

std::string result_status(rclcpp_action::GenericClientGoalHandle::ResultCode code) {
    switch (code) {
        case rclcpp_action::GenericClientGoalHandle::ResultCode::SUCCEEDED:
            return "succeeded";
        case rclcpp_action::GenericClientGoalHandle::ResultCode::ABORTED:
            return "aborted";
        case rclcpp_action::GenericClientGoalHandle::ResultCode::CANCELED:
            return "canceled";
        default:
            return "unknown";
    }
}

double degrees(double radians) {
    return std::round(radians * 180.0 / std::numbers::pi * 1000.0) / 1000.0;
}

}  // namespace

std::string qualified_name(const std::string& name) {
    std::string out = name.starts_with("/") ? name : "/" + name;
    while (out.size() > 1 && out.back() == '/') {
        out.pop_back();
    }
    return out;
}

// TopicTap

std::size_t TopicTap::count() const {
    const std::lock_guard lock(mutex_);
    return count_;
}

std::vector<TopicTap::Sample> TopicTap::samples() const {
    const std::lock_guard lock(mutex_);
    return {samples_.begin(), samples_.end()};
}

std::optional<TopicTap::Sample> TopicTap::latest() const {
    const std::lock_guard lock(mutex_);
    if (samples_.empty()) {
        return std::nullopt;
    }
    return samples_.back();
}

void TopicTap::on_message(std::function<void()> listener) {
    const std::lock_guard lock(mutex_);
    listener_ = std::move(listener);
}

void TopicTap::add(std::shared_ptr<rclcpp::SerializedMessage> message) {
    Sample sample{std::chrono::steady_clock::now(), nullptr, message->size()};
    if (keep_messages_) {
        sample.message = std::move(message);
    }
    std::function<void()> listener;
    {
        const std::lock_guard lock(mutex_);
        ++count_;
        if (capacity_ > 0) {
            samples_.push_back(std::move(sample));
            while (samples_.size() > capacity_) {
                samples_.pop_front();
            }
        }
        listener = listener_;
    }
    if (listener) {
        listener();
    }
}

// ActionGoal

bool ActionGoal::answered() const {
    const std::lock_guard lock(state_->mutex);
    return state_->answered;
}

bool ActionGoal::accepted() const {
    const std::lock_guard lock(state_->mutex);
    return state_->handle != nullptr;
}

bool ActionGoal::finished() const {
    const std::lock_guard lock(state_->mutex);
    return state_->finished;
}

std::size_t ActionGoal::feedback_count() const {
    const std::lock_guard lock(state_->mutex);
    return state_->feedback_count;
}

json ActionGoal::last_feedback() const {
    const std::lock_guard lock(state_->mutex);
    return state_->last_feedback;
}

std::string ActionGoal::status() const {
    const std::lock_guard lock(state_->mutex);
    return state_->status;
}

json ActionGoal::result() const {
    const std::lock_guard lock(state_->mutex);
    return state_->result;
}

std::string ActionGoal::current_status() const {
    rclcpp_action::GenericClientGoalHandle::SharedPtr handle;
    {
        const std::lock_guard lock(state_->mutex);
        if (state_->finished) {
            return state_->status;
        }
        if (!state_->answered) {
            return "sent";
        }
        handle = state_->handle;
    }
    if (!handle) {
        return "rejected";
    }
    switch (handle->get_status()) {
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
            return "executing";
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
            return "canceling";
        default:
            return "accepted";
    }
}

void ActionGoal::cancel() {
    rclcpp_action::GenericClientGoalHandle::SharedPtr handle;
    {
        const std::lock_guard lock(state_->mutex);
        handle = state_->handle;
    }
    if (handle) {
        try {
            client_->async_cancel_goal(handle);
        } catch (const std::exception&) {
            // The goal finished in the meantime, and the client forgot it.
        }
    }
}

// Ros

Ros::Ros(const std::string& node_name, JsonLimits limits) : limits_(limits) {
    rclcpp::NodeOptions options;
    // The node is a client of the graph; nobody needs to set its parameters.
    options.start_parameter_services(false);
    node_ = std::make_shared<rclcpp::Node>(node_name, options);
    executor_.add_node(node_);
    spinner_ = std::thread([this] { executor_.spin(); });
}

Ros::~Ros() {
    executor_.cancel();
    spinner_.join();
}

std::string Ros::node_name() const { return node_->get_fully_qualified_name(); }

Stamp Ros::now() const {
    const rclcpp::Time time = node_->now();
    const std::int64_t nanoseconds = time.nanoseconds();
    return Stamp{static_cast<std::int32_t>(nanoseconds / 1'000'000'000),
                 static_cast<std::uint32_t>(nanoseconds % 1'000'000'000)};
}

json Ros::nodes() const {
    std::vector<std::string> names;
    const std::string self = node_->get_fully_qualified_name();
    for (const auto& [name, space] :
         node_->get_node_graph_interface()->get_node_names_and_namespaces()) {
        if (std::string full = join_name(space, name); full != self) {
            names.push_back(std::move(full));
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

json Ros::topics(bool hidden) const {
    json out = json::array();
    for (const auto& [name, types] : node_->get_topic_names_and_types()) {
        if (!hidden && is_hidden(name)) {
            continue;
        }
        out.push_back(json{{"name", name},
                           {"types", types},
                           {"publishers", node_->count_publishers(name)},
                           {"subscribers", node_->count_subscribers(name)}});
    }
    return out;
}

json Ros::services(bool hidden, bool all) const {
    return names_json(node_->get_service_names_and_types(), hidden, all);
}

json Ros::actions() const {
    rcl_names_and_types_t names = rcl_get_zero_initialized_names_and_types();
    rcl_allocator_t allocator = rcl_get_default_allocator();
    if (rcl_action_get_names_and_types(node_->get_node_base_interface()->get_rcl_node_handle(),
                                       &allocator, &names) != RCL_RET_OK) {
        rcl_reset_error();
        throw RosError("cannot list the actions of the graph");
    }
    return names_json(take_names(names), true, true);
}

json Ros::node_info(const std::string& node) const {
    const std::string full = qualified_name(node);
    const json running = nodes();
    if (std::find(running.begin(), running.end(), full) == running.end()) {
        throw RosError("There is no node " + full +
                       " in the graph; list_nodes shows the running nodes. A node that just "
                       "started can take a moment to appear.");
    }
    const auto [space, name] = split_node_name(full);
    const auto graph = node_->get_node_graph_interface();
    const rcl_node_t* handle = node_->get_node_base_interface()->get_rcl_node_handle();
    rcl_allocator_t allocator = rcl_get_default_allocator();
    const auto action_names = [&](auto query) {
        rcl_names_and_types_t names = rcl_get_zero_initialized_names_and_types();
        if (query(handle, &allocator, name.c_str(), space.c_str(), &names) != RCL_RET_OK) {
            rcl_reset_error();
            return json::array();
        }
        return names_json(take_names(names), true, true);
    };
    try {
        return json{
            {"node", full},
            {"publishers",
             names_json(graph->get_publisher_names_and_types_by_node(name, space), false, true)},
            {"subscribers",
             names_json(graph->get_subscriber_names_and_types_by_node(name, space), false, true)},
            {"services",
             names_json(graph->get_service_names_and_types_by_node(name, space), false, false)},
            {"clients",
             names_json(graph->get_client_names_and_types_by_node(name, space), false, false)},
            {"action_servers", action_names(rcl_action_get_server_names_and_types_by_node)},
            {"action_clients", action_names(rcl_action_get_client_names_and_types_by_node)}};
    } catch (const std::runtime_error& error) {
        // The node can leave between the check and the queries.
        throw RosError("cannot read the interfaces of " + full + ": " + error.what());
    }
}

json Ros::topic_info(const std::string& topic) const {
    const std::string name = qualified_name(topic);
    const auto names = node_->get_topic_names_and_types();
    const auto found = names.find(name);
    if (found == names.end()) {
        throw RosError("No node publishes or subscribes to " + name +
                       "; list_topics shows the topics in use");
    }
    const auto publishers = node_->get_publishers_info_by_topic(name);
    const auto subscribers = node_->get_subscriptions_info_by_topic(name);
    json out{{"topic", name},
             {"types", found->second},
             {"publishers", endpoints_json(publishers)},
             {"subscribers", endpoints_json(subscribers)}};
    if (!std::all_of(publishers.begin(), publishers.end(), node_known) ||
        !std::all_of(subscribers.begin(), subscribers.end(), node_known)) {
        out["note"] =
            "Some nodes are null because their names have not been discovered yet; ask again in a "
            "moment.";
    }
    return out;
}

json Ros::describe_interface(const std::string& type) {
    const std::size_t parts = std::count(type.begin(), type.end(), '/') + 1;
    const auto message = [&](const std::string& name) {
        const auto found = types_.message(name);
        return json{{"type", found->name}, {"kind", "message"}, {"fields", describe(*found->members)}};
    };
    const auto service = [&](const std::string& name) {
        const auto found = types_.service(name);
        return json{{"type", found->name},
                    {"kind", "service"},
                    {"request", describe(*found->members->request_members_)},
                    {"response", describe(*found->members->response_members_)}};
    };
    const auto action = [&](const std::string& name) {
        const auto found = types_.action(name);
        return json{{"type", found->name},
                    {"kind", "action"},
                    {"goal", describe(*found->goal->members)},
                    {"result", describe(*found->result->members)},
                    {"feedback", describe(*found->feedback->members)}};
    };
    if (parts == 3) {
        const std::string kind = type.substr(type.find('/') + 1, type.rfind('/') - type.find('/') - 1);
        if (kind == "msg") {
            return message(type);
        }
        if (kind == "srv") {
            return service(type);
        }
        if (kind == "action") {
            return action(type);
        }
        throw TypeError("'" + type +
                        "' is not a type name such as std_msgs/msg/String, std_srvs/srv/Trigger or "
                        "example_interfaces/action/Fibonacci");
    }
    // A name without its kind, such as std_srvs/Trigger: try each kind.
    try {
        return message(type);
    } catch (const TypeError&) {
    }
    try {
        return service(type);
    } catch (const TypeError&) {
    }
    try {
        return action(type);
    } catch (const TypeError&) {
    }
    throw TypeError("There is no installed message, service or action type " + type +
                    ", or its package is not in the environment");
}

std::shared_ptr<const MessageType> Ros::topic_type(const std::string& topic,
                                                   const std::optional<std::string>& given) {
    if (given) {
        return types_.message(*given);
    }
    const std::string name = qualified_name(topic);
    const auto names = node_->get_topic_names_and_types();
    const auto found = names.find(name);
    if (found == names.end()) {
        throw RosError("No node publishes or subscribes to " + name +
                       ", so its type is unknown; give the type, such as std_msgs/msg/String");
    }
    return types_.message(single_type("message", name, found->second));
}

rclcpp::QoS Ros::subscription_qos(const std::string& topic, std::size_t depth,
                                  const QosRequest& request) const {
    rclcpp::QoS qos(depth);
    // Match every publisher: reliable or transient-local only when all of them are. With no
    // publisher yet, best effort matches whichever comes.
    const auto publishers = node_->get_publishers_info_by_topic(topic);
    bool reliable = !publishers.empty();
    bool transient_local = !publishers.empty();
    for (const auto& publisher : publishers) {
        reliable =
            reliable && publisher.qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable;
        transient_local = transient_local && publisher.qos_profile().durability() ==
                                                 rclcpp::DurabilityPolicy::TransientLocal;
    }
    if (reliable) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    if (transient_local) {
        qos.transient_local();
    } else {
        qos.durability_volatile();
    }
    apply(qos, request);
    return qos;
}

rclcpp::QoS Ros::publisher_qos(const std::string& topic, const QosRequest& request) const {
    rclcpp::QoS qos(request.depth.value_or(10));
    qos.reliable();
    // A transient-local subscriber, such as one of a map, only hears transient-local publishers.
    for (const auto& subscriber : node_->get_subscriptions_info_by_topic(topic)) {
        if (subscriber.qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal) {
            qos.transient_local();
        }
    }
    apply(qos, request);
    return qos;
}

std::shared_ptr<TopicTap> Ros::tap(const std::string& topic, std::shared_ptr<const MessageType> type,
                                   std::size_t capacity, bool keep_messages, const QosRequest& qos) {
    auto tap = std::make_shared<TopicTap>();
    tap->topic_ = qualified_name(topic);
    tap->type_ = std::move(type);
    tap->capacity_ = capacity;
    tap->keep_messages_ = keep_messages;
    const std::size_t depth = qos.depth.value_or(std::clamp<std::size_t>(capacity, 10, 1000));
    std::weak_ptr<TopicTap> weak = tap;
    tap->subscription_ = node_->create_generic_subscription(
        tap->topic_, tap->type_->name, subscription_qos(tap->topic_, depth, qos),
        [weak](std::shared_ptr<rclcpp::SerializedMessage> message) {
            if (const auto alive = weak.lock()) {
                alive->add(std::move(message));
            }
        });
    return tap;
}

json Ros::message_json(const TopicTap& tap, const TopicTap::Sample& sample,
                       const JsonLimits& limits) const {
    if (!sample.message) {
        return nullptr;
    }
    const MessageType& type = *tap.type();
    DynamicMessage message(*type.members);
    rclcpp::SerializationBase(type.cpp).deserialize_message(sample.message.get(), message.data());
    return to_json(*type.members, message.data(), limits);
}

rclcpp::GenericPublisher::SharedPtr Ros::publisher(const std::string& topic,
                                                   const std::shared_ptr<const MessageType>& type,
                                                   const QosRequest& qos) {
    const std::string name = qualified_name(topic);
    const rclcpp::QoS profile = publisher_qos(name, qos);
    const auto& rmw = profile.get_rmw_qos_profile();
    const std::string key = name + " " + type->name + " " + std::to_string(rmw.reliability) + " " +
                            std::to_string(rmw.durability) + " " + std::to_string(rmw.depth);
    const std::lock_guard lock(mutex_);
    auto& slot = publishers_[key];
    if (!slot) {
        slot = node_->create_generic_publisher(name, type->name, profile);
    }
    return slot;
}

rclcpp::SerializedMessage Ros::serialize(const MessageType& type, const json& message) {
    DynamicMessage dynamic(*type.members);
    from_json(message, *type.members, dynamic.data(), [this] { return now(); });
    rclcpp::SerializedMessage out;
    rclcpp::SerializationBase(type.cpp).serialize_message(dynamic.data(), &out);
    return out;
}

std::shared_ptr<const ServiceType> Ros::service_type(const std::string& service,
                                                     const std::optional<std::string>& given) {
    if (given) {
        return types_.service(*given);
    }
    const std::string name = qualified_name(service);
    const auto names = node_->get_service_names_and_types();
    const auto found = names.find(name);
    if (found == names.end()) {
        throw RosError("No node offers the service " + name +
                       "; list_services shows the services offered");
    }
    return types_.service(single_type("service", name, found->second));
}

rclcpp::GenericClient::SharedPtr Ros::service_client(const std::string& service,
                                                     const ServiceType& type) {
    const std::string name = qualified_name(service);
    const std::lock_guard lock(mutex_);
    auto& slot = service_clients_[name + " " + type.name];
    if (!slot) {
        slot = node_->create_generic_client(name, type.name);
    }
    return slot;
}

ServiceCall Ros::call(rclcpp::GenericClient::SharedPtr client, std::shared_ptr<const ServiceType> type,
                      const json& request) {
    const MessageMembers& members = *type->members->request_members_;
    DynamicMessage message(members);
    from_json(request, members, message.data(), [this] { return now(); });
    // The request is serialized before async_send_request returns.
    auto sent = client->async_send_request(message.data());
    ServiceCall out;
    out.client = std::move(client);
    out.type = std::move(type);
    out.request_id = sent.request_id;
    out.response = sent.future.share();
    return out;
}

json Ros::response_json(const ServiceCall& call) const {
    return to_json(*call.type->members->response_members_, call.response.get().get(), limits_);
}

std::shared_ptr<const ActionType> Ros::action_type(const std::string& action,
                                                   const std::optional<std::string>& given) {
    if (given) {
        return types_.action(*given);
    }
    const std::string name = qualified_name(action);
    rcl_names_and_types_t names = rcl_get_zero_initialized_names_and_types();
    rcl_allocator_t allocator = rcl_get_default_allocator();
    if (rcl_action_get_names_and_types(node_->get_node_base_interface()->get_rcl_node_handle(),
                                       &allocator, &names) != RCL_RET_OK) {
        rcl_reset_error();
        throw RosError("cannot list the actions of the graph");
    }
    const auto all = take_names(names);
    const auto found = all.find(name);
    if (found == all.end()) {
        throw RosError("No node offers the action " + name +
                       "; list_actions shows the actions offered");
    }
    return types_.action(single_type("action", name, found->second));
}

rclcpp_action::GenericClient::SharedPtr Ros::action_client(const std::string& action,
                                                           const ActionType& type) {
    const std::string name = qualified_name(action);
    const std::lock_guard lock(mutex_);
    auto& slot = action_clients_[name + " " + type.name];
    if (!slot) {
        slot = rclcpp_action::create_generic_client(node_, name, type.name);
    }
    return slot;
}

std::shared_ptr<ActionGoal> Ros::send_goal(rclcpp_action::GenericClient::SharedPtr client,
                                           std::shared_ptr<const ActionType> type, const json& goal) {
    const MessageMembers& members = *type->send_goal_request->members;
    DynamicMessage request(members);

    // A random goal ID, as UUID version 4.
    std::random_device device;
    std::array<std::uint8_t, 16> uuid{};
    for (auto& byte : uuid) {
        byte = static_cast<std::uint8_t>(device());
    }
    uuid[6] = static_cast<std::uint8_t>((uuid[6] & 0x0F) | 0x40);
    uuid[8] = static_cast<std::uint8_t>((uuid[8] & 0x3F) | 0x80);
    const auto clock = [this] { return now(); };
    from_json(json{{"goal_id", {{"uuid", uuid}}}}, members, request.data(), clock);
    const MessageMember& goal_member = member_named(members, "goal");
    from_json(goal, *static_cast<const MessageMembers*>(goal_member.members_->data),
              static_cast<char*>(request.data()) + goal_member.offset_, clock);

    auto out = std::make_shared<ActionGoal>();
    out->client_ = client;
    out->type_ = type;
    for (std::size_t i = 0; i < uuid.size(); ++i) {
        static constexpr const char* g_HEX = "0123456789abcdef";
        if (i == 4 || i == 6 || i == 8 || i == 10) {
            out->id_ += '-';
        }
        out->id_ += g_HEX[uuid[i] >> 4];
        out->id_ += g_HEX[uuid[i] & 0x0F];
    }
    rclcpp_action::GenericClient::SendGoalOptions options;
    const JsonLimits limits = limits_;
    options.goal_response_callback =
        [state = out->state_](rclcpp_action::GenericClientGoalHandle::SharedPtr handle) {
            const std::lock_guard lock(state->mutex);
            state->answered = true;
            state->handle = std::move(handle);
        };
    options.feedback_callback = [state = out->state_, type, limits](
                                    rclcpp_action::GenericClientGoalHandle::SharedPtr,
                                    const void* feedback) {
        json value = to_json(*type->feedback->members, feedback, limits);
        const std::lock_guard lock(state->mutex);
        ++state->feedback_count;
        state->last_feedback = std::move(value);
    };
    options.result_callback = [state = out->state_, type, limits](
                                  const rclcpp_action::GenericClientGoalHandle::WrappedResult& result) {
        json value = result.result != nullptr ? to_json(*type->result->members, result.result, limits)
                                              : json(nullptr);
        const std::lock_guard lock(state->mutex);
        state->finished = true;
        state->status = result_status(result.code);
        state->result = std::move(value);
    };
    // The request is serialized before async_send_goal returns.
    out->handle_ = client->async_send_goal(request.data(), options);
    return out;
}

std::shared_ptr<rclcpp::AsyncParametersClient> Ros::parameter_client(const std::string& node) {
    const std::string name = qualified_name(node);
    const std::lock_guard lock(mutex_);
    auto& slot = parameter_clients_[name];
    if (!slot) {
        slot = std::make_shared<rclcpp::AsyncParametersClient>(node_, name);
    }
    return slot;
}

ParameterCall<rcl_interfaces::msg::ListParametersResult> Ros::list_parameters(
    std::shared_ptr<rclcpp::AsyncParametersClient> client) {
    auto future =
        client->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
    return {std::move(client), std::move(future)};
}

ParameterCall<std::vector<rclcpp::Parameter>> Ros::get_parameters(
    std::shared_ptr<rclcpp::AsyncParametersClient> client, const std::vector<std::string>& names) {
    auto future = client->get_parameters(names);
    return {std::move(client), std::move(future)};
}

ParameterCall<std::vector<rcl_interfaces::msg::SetParametersResult>> Ros::set_parameters(
    std::shared_ptr<rclcpp::AsyncParametersClient> client,
    const std::vector<rclcpp::Parameter>& parameters) {
    auto future = client->set_parameters(parameters);
    return {std::move(client), std::move(future)};
}

void Ros::listen_to_transforms() {
    const std::lock_guard lock(mutex_);
    if (!tf_buffer_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        // The node's executor delivers the transforms.
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
    }
}

std::optional<json> Ros::transform(const std::string& target, const std::string& source) {
    listen_to_transforms();
    std::shared_ptr<tf2_ros::Buffer> buffer;
    {
        const std::lock_guard lock(mutex_);
        buffer = tf_buffer_;
    }
    std::string error;
    if (!buffer->canTransform(target, source, tf2::TimePointZero, &error)) {
        return std::nullopt;
    }
    const geometry_msgs::msg::TransformStamped stamped =
        buffer->lookupTransform(target, source, tf2::TimePointZero);
    const auto& t = stamped.transform.translation;
    const auto& q = stamped.transform.rotation;
    // Roll, pitch and yaw of the rotation, about the fixed x, y and z axes.
    const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const bool is_static = stamped.header.stamp.sec == 0 && stamped.header.stamp.nanosec == 0;
    json out{
        {"target_frame", target},
        {"source_frame", source},
        {"translation", {{"x", t.x}, {"y", t.y}, {"z", t.z}}},
        {"rotation", {{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}}},
        {"rpy_degrees", {{"roll", degrees(roll)}, {"pitch", degrees(pitch)}, {"yaw", degrees(yaw)}}},
        {"stamp", {{"sec", stamped.header.stamp.sec}, {"nanosec", stamped.header.stamp.nanosec}}}};
    if (!is_static) {
        const double age =
            (node_->now() - rclcpp::Time(stamped.header.stamp, node_->get_clock()->get_clock_type()))
                .seconds();
        out["age_seconds"] = std::round(age * 1000.0) / 1000.0;
    }
    return out;
}

std::vector<std::string> Ros::frames() {
    listen_to_transforms();
    const std::lock_guard lock(mutex_);
    std::vector<std::string> names = tf_buffer_->getAllFrameNames();
    std::sort(names.begin(), names.end());
    return names;
}

// Parameters

json parameter_json(const rclcpp::ParameterValue& value) {
    switch (value.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return value.get<bool>();
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return value.get<std::int64_t>();
        case rclcpp::ParameterType::PARAMETER_DOUBLE: {
            const double number = value.get<double>();
            if (!std::isfinite(number)) {
                return std::isnan(number) ? "nan" : (number > 0 ? "inf" : "-inf");
            }
            return number;
        }
        case rclcpp::ParameterType::PARAMETER_STRING:
            return value.get<std::string>();
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
            return value.get<std::vector<std::uint8_t>>();
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return value.get<std::vector<bool>>();
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return value.get<std::vector<std::int64_t>>();
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return value.get<std::vector<double>>();
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return value.get<std::vector<std::string>>();
        default:
            return nullptr;
    }
}

namespace {

double parameter_double(const json& value) {
    if (value.is_number()) {
        return value.get<double>();
    }
    if (value.is_string()) {
        const std::string text = value.get<std::string>();
        if (text == "nan") {
            return std::nan("");
        }
        if (text == "inf") {
            return INFINITY;
        }
        if (text == "-inf") {
            return -INFINITY;
        }
    }
    throw std::invalid_argument("expected a number, not " + value.dump());
}

std::int64_t parameter_integer(const json& value) {
    if (!value.is_number_integer()) {
        throw std::invalid_argument("expected an integer, not " + value.dump());
    }
    if (value.is_number_unsigned() &&
        value.get<std::uint64_t>() > static_cast<std::uint64_t>(INT64_MAX)) {
        throw std::invalid_argument("the integer " + value.dump() + " is out of range");
    }
    return value.get<std::int64_t>();
}

template <typename T, typename Convert>
std::vector<T> parameter_array(const json& value, Convert convert) {
    if (!value.is_array()) {
        throw std::invalid_argument("expected an array, not " + value.dump());
    }
    std::vector<T> out;
    out.reserve(value.size());
    for (const json& item : value) {
        out.push_back(convert(item));
    }
    return out;
}

rclcpp::ParameterType implied_type(const json& value) {
    using rclcpp::ParameterType;
    if (value.is_boolean()) {
        return ParameterType::PARAMETER_BOOL;
    }
    if (value.is_number_integer()) {
        return ParameterType::PARAMETER_INTEGER;
    }
    if (value.is_number()) {
        return ParameterType::PARAMETER_DOUBLE;
    }
    if (value.is_string()) {
        return ParameterType::PARAMETER_STRING;
    }
    if (value.is_array() && !value.empty()) {
        const json& first = value.front();
        if (first.is_boolean()) {
            return ParameterType::PARAMETER_BOOL_ARRAY;
        }
        if (first.is_string()) {
            return ParameterType::PARAMETER_STRING_ARRAY;
        }
        if (first.is_number()) {
            const bool all_integers = std::all_of(
                value.begin(), value.end(), [](const json& item) { return item.is_number_integer(); });
            return all_integers ? ParameterType::PARAMETER_INTEGER_ARRAY
                                : ParameterType::PARAMETER_DOUBLE_ARRAY;
        }
    }
    throw std::invalid_argument("cannot tell the parameter type of " + value.dump() +
                                "; the parameter is not set yet, so give a value whose type is clear");
}

}  // namespace

rclcpp::ParameterValue parameter_value(const json& value, rclcpp::ParameterType type) {
    using rclcpp::ParameterType;
    if (type == ParameterType::PARAMETER_NOT_SET) {
        type = implied_type(value);
    }
    switch (type) {
        case ParameterType::PARAMETER_BOOL:
            if (!value.is_boolean()) {
                throw std::invalid_argument("expected true or false, not " + value.dump());
            }
            return rclcpp::ParameterValue(value.get<bool>());
        case ParameterType::PARAMETER_INTEGER:
            return rclcpp::ParameterValue(parameter_integer(value));
        case ParameterType::PARAMETER_DOUBLE:
            return rclcpp::ParameterValue(parameter_double(value));
        case ParameterType::PARAMETER_STRING:
            if (!value.is_string()) {
                throw std::invalid_argument("expected a string, not " + value.dump());
            }
            return rclcpp::ParameterValue(value.get<std::string>());
        case ParameterType::PARAMETER_BYTE_ARRAY:
            return rclcpp::ParameterValue(parameter_array<std::uint8_t>(value, [](const json& item) {
                const std::int64_t number = parameter_integer(item);
                if (number < 0 || number > 255) {
                    throw std::invalid_argument("a byte is 0 to 255, not " + item.dump());
                }
                return static_cast<std::uint8_t>(number);
            }));
        case ParameterType::PARAMETER_BOOL_ARRAY:
            return rclcpp::ParameterValue(parameter_array<bool>(value, [](const json& item) {
                if (!item.is_boolean()) {
                    throw std::invalid_argument("expected true or false, not " + item.dump());
                }
                return item.get<bool>();
            }));
        case ParameterType::PARAMETER_INTEGER_ARRAY:
            return rclcpp::ParameterValue(parameter_array<std::int64_t>(value, parameter_integer));
        case ParameterType::PARAMETER_DOUBLE_ARRAY:
            return rclcpp::ParameterValue(parameter_array<double>(value, parameter_double));
        case ParameterType::PARAMETER_STRING_ARRAY:
            return rclcpp::ParameterValue(parameter_array<std::string>(value, [](const json& item) {
                if (!item.is_string()) {
                    throw std::invalid_argument("expected a string, not " + item.dump());
                }
                return item.get<std::string>();
            }));
        default:
            throw std::invalid_argument("unknown parameter type");
    }
}

}  // namespace mcp_ros2
