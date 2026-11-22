#include "server.hpp"

#include <boost/asio/steady_timer.hpp>
#include <boost/asio/this_coro.hpp>
#include <boost/asio/use_awaitable.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mcp_ros2 {

namespace {

using nlohmann::json;
using Clock = std::chrono::steady_clock;

constexpr auto g_POLL_INTERVAL = std::chrono::milliseconds(20);
constexpr auto g_NOTIFY_INTERVAL = std::chrono::seconds(1);
constexpr auto g_CLOSE_WAIT = std::chrono::seconds(5);
constexpr double g_RESOURCE_WAIT_SECONDS = 3;
constexpr double g_DISCOVERY_SECONDS = 1.5;
constexpr std::size_t g_HZ_SAMPLES = 200000;
constexpr std::size_t g_PROGRESS_TEXT = 200;

const std::string g_GRAPH_URI = "ros://graph";
const std::string g_TOPIC_URI = "ros://topic";

// Schemas.

json property(const char* type, const char* description) {
    return json{{"type", type}, {"description", description}};
}

json string_list_property(const char* description) {
    return json{{"type", "array"}, {"items", {{"type", "string"}}}, {"description", description}};
}

json qos_property() {
    return json{
        {"type", "object"},
        {"description",
         "QoS to use instead of the one chosen to match the other side: reliability (reliable or "
         "best_effort), durability (volatile or transient_local) and depth"},
        {"properties",
         {{"reliability", {{"type", "string"}, {"enum", {"reliable", "best_effort"}}}},
          {"durability", {{"type", "string"}, {"enum", {"volatile", "transient_local"}}}},
          {"depth", {{"type", "integer"}, {"minimum", 1}, {"maximum", 1000}}}}},
        {"additionalProperties", false}};
}

json object_schema(json properties, std::initializer_list<const char*> required = {}) {
    json schema{{"type", "object"}, {"properties", std::move(properties)}};
    if (required.size() > 0) {
        schema["required"] = json::array();
        for (const char* name : required) {
            schema["required"].push_back(name);
        }
    }
    return schema;
}

struct Hints {
    bool read_only;
    bool destructive;
};

/// Looks at the graph.
constexpr Hints g_READS{true, false};
/// Changes the running system, which may move a robot.
constexpr Hints g_ACTS{false, true};

mcp::Tool tool(const char* name, const char* title, const char* description, json schema, Hints hints) {
    mcp::Tool out;
    out.name = name;
    out.title = title;
    out.description = description;
    out.inputSchema = std::move(schema);
    mcp::ToolAnnotations annotations;
    annotations.title = title;
    annotations.readOnlyHint = hints.read_only;
    if (!hints.read_only) {
        annotations.destructiveHint = hints.destructive;
    }
    annotations.openWorldHint = !hints.read_only;
    out.annotations = std::move(annotations);
    return out;
}

// Arguments. The server checks their JSON types against the schema before a handler runs.

std::optional<std::string> string_arg(const json& args, const char* name) {
    if (const auto it = args.find(name); it != args.end() && it->is_string()) {
        return it->get<std::string>();
    }
    return std::nullopt;
}

std::string required_string(const json& args, const char* name) {
    auto value = string_arg(args, name);
    if (!value || value->empty()) {
        throw std::invalid_argument(std::string(name) + " is required");
    }
    return *value;
}

std::optional<double> number_arg(const json& args, const char* name, double low, double high) {
    if (const auto it = args.find(name); it != args.end() && it->is_number()) {
        const double value = it->get<double>();
        if (!(value >= low && value <= high)) {
            throw std::invalid_argument(std::string(name) + " must be between " + json(low).dump() +
                                        " and " + json(high).dump());
        }
        return value;
    }
    return std::nullopt;
}

std::optional<std::size_t> count_arg(const json& args, const char* name, std::size_t low,
                                     std::size_t high) {
    if (const auto it = args.find(name); it != args.end() && it->is_number_integer()) {
        const auto value = it->get<std::int64_t>();
        if (value < static_cast<std::int64_t>(low) || value > static_cast<std::int64_t>(high)) {
            throw std::invalid_argument(std::string(name) + " must be between " + std::to_string(low) +
                                        " and " + std::to_string(high));
        }
        return static_cast<std::size_t>(value);
    }
    return std::nullopt;
}

bool bool_arg(const json& args, const char* name) {
    const auto it = args.find(name);
    return it != args.end() && it->is_boolean() && it->get<bool>();
}

json object_arg(const json& args, const char* name) {
    const auto it = args.find(name);
    if (it == args.end() || it->is_null()) {
        return json::object();
    }
    if (!it->is_object()) {
        throw std::invalid_argument(std::string(name) + " must be an object of fields");
    }
    return *it;
}

QosRequest qos_arg(const json& args) {
    QosRequest out;
    const auto it = args.find("qos");
    if (it == args.end() || !it->is_object()) {
        return out;
    }
    out.reliability = string_arg(*it, "reliability");
    out.durability = string_arg(*it, "durability");
    out.depth = count_arg(*it, "depth", 1, 1000);
    return out;
}

JsonLimits limits_arg(const json& args, JsonLimits limits) {
    if (const auto max_array = count_arg(args, "max_array", 1, 100000)) {
        limits.max_array = *max_array;
    }
    return limits;
}

Clock::duration seconds(double value) {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(value));
}

double rounded(double value, int digits) {
    const double scale = std::pow(10.0, digits);
    return std::round(value * scale) / scale;
}

std::string number_text(double value) { return json(rounded(value, 3)).dump(); }

std::string shortened(std::string text) {
    if (text.size() > g_PROGRESS_TEXT) {
        text.resize(g_PROGRESS_TEXT);
        text += "...";
    }
    return text;
}

// Waiting.

struct Progress {
    double value = 0;
    std::optional<double> total;
    std::string message;
};

using ProgressFn = std::function<std::optional<Progress>()>;

/// Waits, without blocking the handler thread, until @p ready returns true, @p deadline passes or
/// the client cancels the call. Reports @p progress to the client as it grows. Returns whether
/// @p ready returned true.
template <typename Ready>
mcp::Task<bool> wait_until(mcp::Context& ctx, Clock::time_point deadline, Ready ready,
                           ProgressFn progress = {}) {
    auto executor = co_await boost::asio::this_coro::executor;
    boost::asio::steady_timer timer(executor);
    double reported = -1;
    while (true) {
        if (ready()) {
            co_return true;
        }
        if (ctx.is_cancelled() || Clock::now() >= deadline) {
            co_return false;
        }
        if (progress) {
            if (const auto current = progress(); current && current->value > reported) {
                reported = current->value;
                co_await ctx.report_progress(current->value, current->total, current->message);
            }
        }
        timer.expires_after(std::min<Clock::duration>(g_POLL_INTERVAL, deadline - Clock::now()));
        co_await timer.async_wait(boost::asio::use_awaitable);
    }
}

template <typename T>
bool is_ready(const std::shared_future<T>& future) {
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

/// Why a wait ended early, for an error message.
std::string stopped(const mcp::Context& ctx, double timeout) {
    return ctx.is_cancelled() ? "the call was cancelled"
                              : "nothing came within " + number_text(timeout) + " s";
}

/// Waits out the first moments of the node, in which discovery is still filling in the graph, so
/// that the first listing an agent asks for is not empty.
mcp::Task<void> settle(mcp::Context& ctx, const Ros& ros) {
    co_await wait_until(ctx, ros.started() + seconds(g_DISCOVERY_SECONDS), [] { return false; });
}

using GraphQuery = std::function<json(const json&)>;

mcp::Task<json> query_graph(mcp::Context& ctx, Ros& ros, GraphQuery query, json args) {
    co_await settle(ctx, ros);
    co_return query(args);
}

// Messages.

/// The part of @p message at the dotted @p path, such as pose.position or ranges.0.
json select_field(const json& message, const std::string& path) {
    const json* at = &message;
    std::string walked;
    std::size_t start = 0;
    while (start <= path.size()) {
        const std::size_t end = std::min(path.find('.', start), path.size());
        const std::string part = path.substr(start, end - start);
        start = end + 1;
        if (part.empty()) {
            throw std::invalid_argument("field '" + path + "' has an empty part");
        }
        // A shortened array keeps its first values under "head".
        if (at->is_object() && at->contains("head") && at->contains("length") && at->size() == 2) {
            at = &(*at)["head"];
        }
        if (at->is_array()) {
            std::size_t index = 0;
            const auto [ptr, ec] = std::from_chars(part.data(), part.data() + part.size(), index);
            if (ec != std::errc() || ptr != part.data() + part.size() || index >= at->size()) {
                throw std::invalid_argument(walked + part + " is not one of the " +
                                            std::to_string(at->size()) + " values shown of " +
                                            (walked.empty() ? "the message" : walked));
            }
            at = &(*at)[index];
        } else if (at->is_object()) {
            const auto found = at->find(part);
            if (found == at->end()) {
                std::string names;
                for (const auto& [key, value] : at->items()) {
                    names += (names.empty() ? "" : ", ") + key;
                }
                throw std::invalid_argument((walked.empty() ? std::string("the message") : walked) +
                                            " has no field " + part + "; its fields are " + names);
            }
            at = &*found;
        } else {
            throw std::invalid_argument(walked + " has no fields");
        }
        walked += part + ".";
        if (end == path.size()) {
            break;
        }
    }
    return *at;
}

// The tools. Each runs as a coroutine on the handler thread; the ROS node's own thread delivers
// what they wait for.

mcp::Task<json> echo(mcp::Context& ctx, Ros& ros, json args) {
    const std::string topic = qualified_name(required_string(args, "topic"));
    co_await settle(ctx, ros);
    const auto type = ros.topic_type(topic, string_arg(args, "type"));
    const std::size_t count = count_arg(args, "count", 1, 100).value_or(1);
    const double timeout = number_arg(args, "timeout", 0.1, 300).value_or(5);
    const auto field = string_arg(args, "field");
    const JsonLimits limits = limits_arg(args, ros.limits());

    const auto tap = ros.tap(topic, type, count, true, qos_arg(args));
    const bool complete = co_await wait_until(
        ctx, Clock::now() + seconds(timeout), [&tap, count] { return tap->count() >= count; },
        [&tap, count]() -> std::optional<Progress> {
            const std::size_t received = tap->count();
            return Progress{static_cast<double>(received), static_cast<double>(count),
                            "Received " + std::to_string(received) + " of " + std::to_string(count)};
        });

    json messages = json::array();
    for (const auto& sample : tap->samples()) {
        json message = ros.message_json(*tap, sample, limits);
        messages.push_back(field ? select_field(message, *field) : std::move(message));
    }
    json out{{"topic", topic}, {"type", type->name}, {"messages", std::move(messages)}};
    if (!complete) {
        out["note"] = ctx.is_cancelled()
                          ? std::string("The call was cancelled.")
                          : "Timed out after " + number_text(timeout) + " s with " +
                                std::to_string(out["messages"].size()) + " of " +
                                std::to_string(count) +
                                " messages. topic_info shows who publishes, and with which QoS.";
    }
    co_return out;
}

mcp::Task<json> topic_hz(mcp::Context& ctx, Ros& ros, json args) {
    const std::string topic = qualified_name(required_string(args, "topic"));
    co_await settle(ctx, ros);
    const auto type = ros.topic_type(topic, string_arg(args, "type"));
    const double window = number_arg(args, "seconds", 0.5, 60).value_or(5);

    const auto tap = ros.tap(topic, type, g_HZ_SAMPLES, false, qos_arg(args));
    const auto start = Clock::now();
    co_await wait_until(
        ctx, start + seconds(window), [] { return false; },
        [&]() -> std::optional<Progress> {
            const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            return Progress{rounded(elapsed, 1), window, std::to_string(tap->count()) + " messages"};
        });
    const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();

    const auto samples = tap->samples();
    json out{{"topic", topic},
             {"type", type->name},
             {"seconds", rounded(elapsed, 3)},
             {"messages", samples.size()}};
    std::size_t bytes = 0;
    for (const auto& sample : samples) {
        bytes += sample.bytes;
    }
    if (!samples.empty()) {
        out["bytes_per_second"] = rounded(static_cast<double>(bytes) / elapsed, 1);
        out["mean_bytes"] =
            rounded(static_cast<double>(bytes) / static_cast<double>(samples.size()), 1);
    }
    if (samples.size() >= 2) {
        std::vector<double> periods;
        for (std::size_t i = 1; i < samples.size(); ++i) {
            periods.push_back(
                std::chrono::duration<double>(samples[i].received - samples[i - 1].received).count());
        }
        double sum = 0;
        for (const double period : periods) {
            sum += period;
        }
        const double mean = sum / static_cast<double>(periods.size());
        double variance = 0;
        for (const double period : periods) {
            variance += (period - mean) * (period - mean);
        }
        variance /= static_cast<double>(periods.size());
        out["rate_hz"] = rounded(1.0 / mean, 3);
        out["period_seconds"] = {{"mean", rounded(mean, 6)},
                                 {"min", rounded(*std::min_element(periods.begin(), periods.end()), 6)},
                                 {"max", rounded(*std::max_element(periods.begin(), periods.end()), 6)},
                                 {"stddev", rounded(std::sqrt(variance), 6)}};
    } else {
        out["note"] = ctx.is_cancelled() ? std::string("The call was cancelled.")
                                         : "Too few messages in " + number_text(elapsed) +
                                               " s to measure a rate. topic_info shows who publishes.";
    }
    co_return out;
}

mcp::Task<json> publish(mcp::Context& ctx, Ros& ros, json args) {
    const std::string topic = qualified_name(required_string(args, "topic"));
    co_await settle(ctx, ros);
    const auto type = ros.topic_type(topic, string_arg(args, "type"));
    const json message = object_arg(args, "message");
    const std::size_t times = count_arg(args, "times", 1, 1000).value_or(1);
    const double rate = number_arg(args, "rate", 0.1, 1000).value_or(10);
    const double wait = number_arg(args, "wait_for_subscribers", 0, 30).value_or(1);

    // Serializing checks the message before anything goes out.
    const rclcpp::SerializedMessage first = ros.serialize(*type, message);
    const auto publisher = ros.publisher(topic, type, qos_arg(args));
    co_await wait_until(ctx, Clock::now() + seconds(wait),
                        [&publisher] { return publisher->get_subscription_count() > 0; });

    std::size_t published = 0;
    auto next = Clock::now();
    while (published < times && !ctx.is_cancelled()) {
        // Each message gets its own "now".
        publisher->publish(published == 0 ? first : ros.serialize(*type, message));
        ++published;
        if (times > 1) {
            co_await ctx.report_progress(
                static_cast<double>(published), static_cast<double>(times),
                "Published " + std::to_string(published) + " of " + std::to_string(times));
        }
        if (published < times) {
            next += seconds(1.0 / rate);
            co_await wait_until(ctx, next, [] { return false; });
        }
    }
    const std::size_t subscribers = publisher->get_subscription_count();
    json out{
        {"topic", topic}, {"type", type->name}, {"published", published}, {"subscribers", subscribers}};
    if (subscribers == 0) {
        out["note"] = "No node subscribes to " + topic + ", so nobody received it.";
    }
    co_return out;
}

mcp::Task<json> call_service(mcp::Context& ctx, Ros& ros, json args) {
    const std::string service = qualified_name(required_string(args, "service"));
    co_await settle(ctx, ros);
    const auto type = ros.service_type(service, string_arg(args, "type"));
    const json request = object_arg(args, "request");
    const double timeout = number_arg(args, "timeout", 0.1, 300).value_or(10);
    const auto deadline = Clock::now() + seconds(timeout);

    const auto client = ros.service_client(service, *type);
    if (!co_await wait_until(ctx, deadline, [&client] { return client->service_is_ready(); })) {
        throw RosError("The service " + service + " is not available: " + stopped(ctx, timeout));
    }
    const ServiceCall call = ros.call(client, type, request);
    if (!co_await wait_until(ctx, deadline, [&call] { return is_ready(call.response); })) {
        client->remove_pending_request(call.request_id);
        throw RosError("No response from " + service + ": " + stopped(ctx, timeout));
    }
    co_return json{{"service", service}, {"type", type->name}, {"response", ros.response_json(call)}};
}

mcp::Task<json> send_action_goal(mcp::Context& ctx, Ros& ros, json args) {
    const std::string action = qualified_name(required_string(args, "action"));
    co_await settle(ctx, ros);
    const auto type = ros.action_type(action, string_arg(args, "type"));
    const json goal_fields = object_arg(args, "goal");
    const double timeout = number_arg(args, "timeout", 1, 3600).value_or(120);
    const auto deadline = Clock::now() + seconds(timeout);

    const auto client = ros.action_client(action, *type);
    if (!co_await wait_until(ctx, deadline, [&client] { return client->action_server_is_ready(); })) {
        throw RosError("The action server of " + action +
                       " is not available: " + stopped(ctx, timeout));
    }
    const auto goal = ros.send_goal(client, type, goal_fields);
    if (!co_await wait_until(ctx, deadline, [&goal] { return goal->answered(); })) {
        throw RosError("The action server of " + action +
                       " did not answer the goal: " + stopped(ctx, timeout));
    }
    json out{{"action", action}, {"type", type->name}, {"accepted", goal->accepted()}};
    if (!goal->accepted()) {
        out["note"] = "The action server rejected the goal.";
        co_return out;
    }

    const bool finished = co_await wait_until(
        ctx, deadline, [&goal] { return goal->finished(); },
        [&goal]() -> std::optional<Progress> {
            const std::size_t count = goal->feedback_count();
            if (count == 0) {
                return std::nullopt;
            }
            return Progress{static_cast<double>(count), std::nullopt,
                            shortened(goal->last_feedback().dump())};
        });
    if (!finished) {
        // A goal nobody waits for any more is cancelled rather than left running.
        out["note"] = std::string(ctx.is_cancelled() ? "The call was cancelled"
                                                     : "The goal did not finish within " +
                                                           number_text(timeout) + " s") +
                      ", so the server was asked to cancel it.";
        goal->cancel();
        co_await wait_until(ctx, Clock::now() + std::chrono::seconds(5),
                            [&goal] { return goal->finished(); });
    }
    out["status"] = goal->finished() ? goal->status() : std::string("cancelling");
    out["result"] = goal->result();
    out["feedback_count"] = goal->feedback_count();
    if (goal->feedback_count() > 0) {
        out["last_feedback"] = goal->last_feedback();
    }
    co_return out;
}

/// A parameter client for @p node whose services are up.
mcp::Task<std::shared_ptr<rclcpp::AsyncParametersClient>> ready_parameter_client(
    mcp::Context& ctx, Ros& ros, const std::string& node, Clock::time_point deadline, double timeout) {
    auto client = ros.parameter_client(node);
    if (!co_await wait_until(ctx, deadline, [&client] { return client->service_is_ready(); })) {
        throw RosError("The node " + node + " does not answer parameter requests (" +
                       stopped(ctx, timeout) + "); list_nodes shows the running nodes");
    }
    co_return client;
}

template <typename T>
mcp::Task<T> parameter_result(mcp::Context& ctx, ParameterCall<T> call, const std::string& node,
                              Clock::time_point deadline, double timeout) {
    if (!co_await wait_until(ctx, deadline, [&call] { return is_ready(call.future); })) {
        throw RosError("No parameter response from " + node + ": " + stopped(ctx, timeout));
    }
    co_return call.future.get();
}

mcp::Task<json> get_parameters(mcp::Context& ctx, Ros& ros, json args) {
    const std::string node = qualified_name(required_string(args, "node"));
    std::vector<std::string> names;
    if (const auto it = args.find("names"); it != args.end() && it->is_array()) {
        for (const json& name : *it) {
            if (name.is_string()) {
                names.push_back(name.get<std::string>());
            }
        }
    }
    const double timeout = number_arg(args, "timeout", 0.1, 60).value_or(5);
    const auto deadline = Clock::now() + seconds(timeout);

    const auto client = co_await ready_parameter_client(ctx, ros, node, deadline, timeout);
    if (names.empty()) {
        const auto listed =
            co_await parameter_result(ctx, ros.list_parameters(client), node, deadline, timeout);
        names = listed.names;
        std::sort(names.begin(), names.end());
    }
    json values = json::object();
    if (!names.empty()) {
        const auto parameters =
            co_await parameter_result(ctx, ros.get_parameters(client, names), node, deadline, timeout);
        for (const auto& parameter : parameters) {
            values[parameter.get_name()] = parameter_json(parameter.get_parameter_value());
        }
    }
    co_return json{{"node", node}, {"parameters", std::move(values)}};
}

mcp::Task<json> set_parameters(mcp::Context& ctx, Ros& ros, json args) {
    const std::string node = qualified_name(required_string(args, "node"));
    const json wanted = object_arg(args, "parameters");
    if (wanted.empty()) {
        throw std::invalid_argument("parameters must name at least one parameter");
    }
    const double timeout = number_arg(args, "timeout", 0.1, 60).value_or(5);
    const auto deadline = Clock::now() + seconds(timeout);

    const auto client = co_await ready_parameter_client(ctx, ros, node, deadline, timeout);
    std::vector<std::string> names;
    for (const auto& [name, value] : wanted.items()) {
        names.push_back(name);
    }
    // Values take the types the parameters have, so 1 sets a double parameter to 1.0.
    const auto current =
        co_await parameter_result(ctx, ros.get_parameters(client, names), node, deadline, timeout);
    std::map<std::string, rclcpp::ParameterType> types;
    for (const auto& parameter : current) {
        types[parameter.get_name()] = parameter.get_type();
    }
    std::vector<rclcpp::Parameter> parameters;
    for (const auto& name : names) {
        try {
            parameters.emplace_back(name, parameter_value(wanted[name], types[name]));
        } catch (const std::invalid_argument& error) {
            throw std::invalid_argument(name + ": " + error.what());
        }
    }
    const auto results =
        co_await parameter_result(ctx, ros.set_parameters(client, parameters), node, deadline, timeout);
    json out = json::object();
    for (std::size_t i = 0; i < parameters.size() && i < results.size(); ++i) {
        json entry{{"successful", results[i].successful}};
        if (!results[i].reason.empty()) {
            entry["reason"] = results[i].reason;
        }
        if (results[i].successful) {
            entry["value"] = parameter_json(parameters[i].get_parameter_value());
        }
        out[parameters[i].get_name()] = std::move(entry);
    }
    co_return json{{"node", node}, {"results", std::move(out)}};
}

mcp::Task<json> lookup_transform(mcp::Context& ctx, Ros& ros, json args) {
    const std::string target = required_string(args, "target_frame");
    const std::string source = required_string(args, "source_frame");
    const double timeout = number_arg(args, "timeout", 0, 60).value_or(2);
    std::optional<json> found;
    co_await wait_until(ctx, Clock::now() + seconds(timeout), [&] {
        found = ros.transform(target, source);
        return found.has_value();
    });
    if (!found) {
        throw RosError("No transform from " + source + " to " + target + ": " + stopped(ctx, timeout) +
                       ". list_frames shows the frames heard of.");
    }
    co_return *found;
}

mcp::Task<json> list_frames(mcp::Context& ctx, Ros& ros, json args) {
    const double wait = number_arg(args, "wait", 0, 30).value_or(1);
    ros.listen_to_transforms();
    // Frames arrive as their transforms are published; static ones at once.
    co_await wait_until(ctx, Clock::now() + seconds(wait), [] { return false; });
    co_return json{{"frames", ros.frames()}};
}

}  // namespace

// Resources: the graph, and the newest message on each topic.

struct RosTools::Watches : std::enable_shared_from_this<RosTools::Watches> {
    Watches(mcp::Server& server_ref, Ros& ros_ref, Spawn spawn_fn)
        : server(server_ref), ros(ros_ref), spawn(std::move(spawn_fn)) {}

    struct Watch {
        std::size_t subscribers = 0;
        std::shared_ptr<TopicTap> tap;
    };

    /// The topic a ros://topic URI names.
    static std::optional<std::string> topic_of(const std::string& uri) {
        if (!uri.starts_with(g_TOPIC_URI + "/") || uri.size() <= g_TOPIC_URI.size() + 1) {
            return std::nullopt;
        }
        return qualified_name(uri.substr(g_TOPIC_URI.size()));
    }

    /// The watched tap of @p uri, made when it is watched but has none yet, as when its topic was
    /// unknown at first.
    std::shared_ptr<TopicTap> tap(const std::string& uri) {
        const std::lock_guard lock(mutex);
        const auto found = by_uri.find(uri);
        if (found == by_uri.end()) {
            return nullptr;
        }
        if (!found->second.tap) {
            found->second.tap = make_tap(uri);
        }
        return found->second.tap;
    }

    void subscribe(const std::string& uri) {
        if (!topic_of(uri)) {
            return;
        }
        const std::lock_guard lock(mutex);
        Watch& watch = by_uri[uri];
        ++watch.subscribers;
        if (!watch.tap) {
            watch.tap = make_tap(uri);
        }
    }

    void unsubscribe(const std::string& uri) {
        const std::lock_guard lock(mutex);
        const auto found = by_uri.find(uri);
        if (found != by_uri.end() && --found->second.subscribers == 0) {
            by_uri.erase(found);
        }
    }

    /// Stops watching, and waits for the notices on their way, which use the server.
    void close() {
        {
            const std::lock_guard lock(notice_mutex);
            open = false;
        }
        {
            const std::lock_guard lock(mutex);
            by_uri.clear();
        }
        std::unique_lock lock(notice_mutex);
        notices_idle.wait_for(lock, g_CLOSE_WAIT, [this] { return notices == 0; });
    }

    void notice_done() {
        const std::lock_guard lock(notice_mutex);
        --notices;
        notices_idle.notify_all();
    }

    mcp::Server& server;
    Ros& ros;
    Spawn spawn;
    std::mutex mutex;
    std::map<std::string, Watch> by_uri;
    std::mutex notice_mutex;
    std::condition_variable notices_idle;
    bool open = true;
    std::size_t notices = 0;

   private:
    static mcp::Task<void> notify(std::shared_ptr<Watches> watches, std::string uri) {
        try {
            co_await watches->server.notify_resource_updated(uri);
        } catch (const std::exception&) {
            // The client has gone.
        }
        watches->notice_done();
    }

    std::shared_ptr<TopicTap> make_tap(const std::string& uri) {
        try {
            const std::string topic = *topic_of(uri);
            auto tap = ros.tap(topic, ros.topic_type(topic, std::nullopt), 1, true, {});
            // Tell subscribers about new messages, at most once per interval.
            auto last = std::make_shared<std::pair<std::mutex, Clock::time_point>>();
            tap->on_message([self = weak_from_this(), uri, last] {
                const auto watches = self.lock();
                if (!watches) {
                    return;
                }
                {
                    const std::lock_guard lock(last->first);
                    const auto now = Clock::now();
                    if (last->second != Clock::time_point{} && now - last->second < g_NOTIFY_INTERVAL) {
                        return;
                    }
                    last->second = now;
                }
                {
                    const std::lock_guard lock(watches->notice_mutex);
                    if (!watches->open) {
                        return;
                    }
                    ++watches->notices;
                }
                try {
                    watches->spawn(notify(watches, uri));
                } catch (const std::exception&) {
                    watches->notice_done();
                }
            });
            return tap;
        } catch (const std::exception& error) {
            std::cerr << "mcp-ros2: cannot watch " << uri << " yet: " << error.what() << "\n";
            return nullptr;
        }
    }
};

namespace {

mcp::Task<mcp::ReadResourceResult> read_topic(mcp::Context& ctx,
                                              std::shared_ptr<RosTools::Watches> watches,
                                              std::string uri) {
    Ros& ros = watches->ros;
    json body;
    try {
        const auto topic = RosTools::Watches::topic_of(uri);
        if (!topic) {
            throw std::invalid_argument(
                "A topic resource is ros://topic/<name>, such as ros://topic/odom");
        }
        auto tap = watches->tap(uri);
        if (!tap || !tap->latest()) {
            tap = ros.tap(*topic, ros.topic_type(*topic, std::nullopt), 1, true, {});
            co_await wait_until(ctx, Clock::now() + seconds(g_RESOURCE_WAIT_SECONDS),
                                [&tap] { return tap->count() > 0; });
        }
        body = json{{"topic", *topic}, {"type", tap->type()->name}};
        if (const auto latest = tap->latest()) {
            body["message"] = ros.message_json(*tap, *latest, ros.limits());
        } else {
            body["message"] = nullptr;
            body["note"] = "No message within " + number_text(g_RESOURCE_WAIT_SECONDS) + " s";
        }
    } catch (const std::exception& error) {
        body = json{{"error", error.what()}};
    }
    mcp::TextResourceContents content;
    content.uri = uri;
    content.text = body.dump(2);
    content.mimeType = "application/json";
    mcp::ReadResourceResult result;
    result.contents.emplace_back(std::move(content));
    co_return result;
}

}  // namespace

RosTools::RosTools(mcp::Server& server, Ros& ros, const Options& options, Spawn spawn)
    : watches_(std::make_shared<Watches>(server, ros, std::move(spawn))) {
    server.set_instructions(server_instructions(options));
    const auto add = [&server, &ros](mcp::Tool definition,
                                     mcp::Task<json> (*run)(mcp::Context&, Ros&, json)) {
        server.add_tool(std::move(definition),
                        [&ros, run](mcp::Context& ctx, const json& args) -> mcp::Task<json> {
                            return run(ctx, ros, args);
                        });
    };
    const auto add_query = [&server, &ros](mcp::Tool definition, GraphQuery query) {
        server.add_tool(
            std::move(definition),
            [&ros, query = std::move(query)](mcp::Context& ctx, const json& args) -> mcp::Task<json> {
                return query_graph(ctx, ros, query, args);
            });
    };

    // The graph.
    add_query(
        tool("list_nodes", "List nodes", "The nodes running in the ROS graph, by fully qualified name.",
             object_schema(json::object()), g_READS),
        [&ros](const json&) { return json{{"nodes", ros.nodes()}}; });
    add_query(
        tool("list_topics", "List topics",
             "The topics in use, with their message types and how many publishers and "
             "subscribers each has.",
             object_schema({{"hidden", property("boolean",
                                                "Also list hidden topics, such as those that make up "
                                                "actions (default false)")}}),
             g_READS),
        [&ros](const json& args) { return json{{"topics", ros.topics(bool_arg(args, "hidden"))}}; });
    add_query(
        tool(
            "list_services", "List services",
            "The services offered, with their types. Every node's parameter and type "
            "description services are left out unless all is true.",
            object_schema({{"hidden", property("boolean", "Also list hidden services (default false)")},
                           {"all", property("boolean",
                                            "Also list every node's parameter and type description "
                                            "services (default false)")}}),
            g_READS),
        [&ros](const json& args) {
            return json{{"services", ros.services(bool_arg(args, "hidden"), bool_arg(args, "all"))}};
        });
    add_query(tool("list_actions", "List actions", "The actions offered, with their types.",
                   object_schema(json::object()), g_READS),
              [&ros](const json&) { return json{{"actions", ros.actions()}}; });
    add_query(
        tool("node_info", "Describe a node",
             "What a node publishes and subscribes to, the services and actions it offers, "
             "and those it uses.",
             object_schema({{"node", property("string", "The node, such as /turtlesim")}}, {"node"}),
             g_READS),
        [&ros](const json& args) { return ros.node_info(required_string(args, "node")); });
    add_query(
        tool("topic_info", "Describe a topic",
             "Who publishes and subscribes to a topic, with each one's QoS. Use it when "
             "echo hears nothing: a reliable subscriber hears no best-effort publisher, "
             "and a transient-local one no volatile publisher.",
             object_schema({{"topic", property("string", "The topic, such as /odom")}}, {"topic"}),
             g_READS),
        [&ros](const json& args) { return ros.topic_info(required_string(args, "topic")); });
    server.add_tool(
        tool("describe_interface", "Describe a type",
             "The fields of a message, service or action type, nested types expanded, "
             "with each field's type and array bounds. Use it to write the JSON for "
             "publish, call_service and send_action_goal.",
             object_schema({{"type", property("string",
                                              "The type, such as geometry_msgs/msg/Twist, "
                                              "std_srvs/srv/SetBool or nav2_msgs/action/"
                                              "NavigateToPose; the middle part may be left out")}},
                           {"type"}),
             g_READS),
        [&ros](const json& args) { return ros.describe_interface(required_string(args, "type")); });

    // Topics.
    add(tool("echo", "Read messages",
             "Waits for the next messages on a topic and returns them as JSON. Arrays longer than "
             "max_array come back as {length, head}. field picks part of each message, such as "
             "pose.pose.position.",
             object_schema(
                 {{"topic", property("string", "The topic, such as /odom")},
                  {"type", property("string", "The message type, when no node uses the topic yet")},
                  {"count",
                   {{"type", "integer"},
                    {"minimum", 1},
                    {"maximum", 100},
                    {"description", "How many messages to wait for (default 1)"}}},
                  {"timeout",
                   {{"type", "number"},
                    {"minimum", 0.1},
                    {"maximum", 300},
                    {"description", "Seconds to wait (default 5)"}}},
                  {"field", property("string",
                                     "A dotted path into each message, such as "
                                     "pose.position or ranges.0")},
                  {"max_array",
                   {{"type", "integer"},
                    {"minimum", 1},
                    {"description", "Show at most this many values of each array"}}},
                  {"qos", qos_property()}},
                 {"topic"}),
             g_READS),
        echo);
    add(tool("topic_hz", "Measure a topic",
             "Listens to a topic for a while and reports its message rate, the spread of the time "
             "between messages, and its bandwidth.",
             object_schema(
                 {{"topic", property("string", "The topic, such as /scan")},
                  {"type", property("string", "The message type, when no node uses the topic yet")},
                  {"seconds",
                   {{"type", "number"},
                    {"minimum", 0.5},
                    {"maximum", 60},
                    {"description", "How long to listen (default 5)"}}},
                  {"qos", qos_property()}},
                 {"topic"}),
             g_READS),
        topic_hz);

    // Transforms.
    add(tool("lookup_transform", "Look up a transform",
             "The newest transform from source_frame to target_frame from /tf and /tf_static: "
             "translation, rotation as a quaternion and as roll, pitch and yaw in degrees, and its "
             "age.",
             object_schema(
                 {{"target_frame", property("string", "The frame to express the pose in, such as map")},
                  {"source_frame",
                   property("string", "The frame whose pose to look up, such as base_link")},
                  {"timeout",
                   {{"type", "number"},
                    {"minimum", 0},
                    {"maximum", 60},
                    {"description", "Seconds to wait for the transform (default 2)"}}}},
                 {"target_frame", "source_frame"}),
             g_READS),
        lookup_transform);
    add(tool("list_frames", "List frames",
             "The coordinate frames heard of on /tf and /tf_static. The first call listens for "
             "wait seconds to hear them.",
             object_schema({{"wait",
                             {{"type", "number"},
                              {"minimum", 0},
                              {"maximum", 30},
                              {"description", "Seconds to listen first (default 1)"}}}}),
             g_READS),
        list_frames);

    // Parameters.
    add(tool("get_parameters", "Read parameters", "The parameters of a node and their values.",
             object_schema({{"node", property("string", "The node, such as /turtlesim")},
                            {"names", string_list_property("The parameters to read (default: all)")},
                            {"timeout",
                             {{"type", "number"},
                              {"minimum", 0.1},
                              {"maximum", 60},
                              {"description", "Seconds to wait for the node (default 5)"}}}},
                           {"node"}),
             g_READS),
        get_parameters);

    if (!options.read_only) {
        add(tool(
                "publish", "Publish a message",
                "Publishes a message on a topic, once or several times at a rate. The message is a "
                "JSON object of the type's fields; fields left out keep their defaults, and a "
                "builtin_interfaces/msg/Time field may be \"now\". This acts on the running system: "
                "a velocity command moves a robot.",
                object_schema(
                    {{"topic", property("string", "The topic, such as /cmd_vel")},
                     {"type", property("string", "The message type, when no node uses the topic yet")},
                     {"message", property("object", "The message, such as {\"linear\": {\"x\": 0.2}}")},
                     {"times",
                      {{"type", "integer"},
                       {"minimum", 1},
                       {"maximum", 1000},
                       {"description", "How many times to publish it (default 1)"}}},
                     {"rate",
                      {{"type", "number"},
                       {"minimum", 0.1},
                       {"maximum", 1000},
                       {"description", "Messages per second when times > 1 (default 10)"}}},
                     {"wait_for_subscribers",
                      {{"type", "number"},
                       {"minimum", 0},
                       {"maximum", 30},
                       {"description", "Seconds to wait for a subscriber first (default 1)"}}},
                     {"qos", qos_property()}},
                    {"topic"}),
                g_ACTS),
            publish);
        add(tool("call_service", "Call a service",
                 "Calls a service and returns its response. The request is a JSON object of the "
                 "type's request fields. This acts on the running system.",
                 object_schema(
                     {{"service", property("string", "The service, such as /reset")},
                      {"type", property("string", "The service type, when the graph does not know it")},
                      {"request", property("object", "The request fields (default: all defaults)")},
                      {"timeout",
                       {{"type", "number"},
                        {"minimum", 0.1},
                        {"maximum", 300},
                        {"description",
                         "Seconds to wait for the service and its response "
                         "(default 10)"}}}},
                     {"service"}),
                 g_ACTS),
            call_service);
        add(tool("send_action_goal", "Send an action goal",
                 "Sends a goal to an action server and waits for its result, reporting feedback as "
                 "progress. A goal still running at the timeout, or when the call is cancelled, is "
                 "cancelled. This acts on the running system: a navigation goal moves a robot.",
                 object_schema(
                     {{"action", property("string", "The action, such as /navigate_to_pose")},
                      {"type", property("string", "The action type, when the graph does not know it")},
                      {"goal", property("object", "The goal fields")},
                      {"timeout",
                       {{"type", "number"},
                        {"minimum", 1},
                        {"maximum", 3600},
                        {"description", "Seconds to wait for the result (default 120)"}}}},
                     {"action"}),
                 g_ACTS),
            send_action_goal);
        add(tool("set_parameters", "Set parameters",
                 "Sets parameters of a node. Values take the parameter's current type. This acts on "
                 "the running system.",
                 object_schema({{"node", property("string", "The node, such as /turtlesim")},
                                {"parameters",
                                 property("object", "The new values, such as {\"background_r\": 255}")},
                                {"timeout",
                                 {{"type", "number"},
                                  {"minimum", 0.1},
                                  {"maximum", 60},
                                  {"description", "Seconds to wait for the node (default 5)"}}}},
                               {"node", "parameters"}),
                 g_ACTS),
            set_parameters);
    }

    mcp::Resource graph;
    graph.uri = g_GRAPH_URI;
    graph.name = "ROS graph";
    graph.description = "The nodes, topics, services and actions of the ROS graph";
    graph.mimeType = "application/json";
    server.add_text_resource(graph, [&ros]() -> std::string {
        try {
            return json{{"nodes", ros.nodes()},
                        {"topics", ros.topics(false)},
                        {"services", ros.services(false, false)},
                        {"actions", ros.actions()}}
                .dump(2);
        } catch (const std::exception& error) {
            return json{{"error", error.what()}}.dump(2);
        }
    });

    mcp::ResourceTemplate topic_template;
    topic_template.uriTemplate = g_TOPIC_URI + "{+topic}";
    topic_template.name = "ROS topic";
    topic_template.description =
        "The newest message on a topic, such as ros://topic/odom. Subscribe to hear when new messages "
        "arrive, at most once a second.";
    topic_template.mimeType = "application/json";
    server.add_resource_template<mcp::ReadResourceRequestParams, mcp::ReadResourceResult>(
        topic_template,
        [watches = watches_](mcp::Context& ctx, const mcp::ReadResourceRequestParams& params) {
            return read_topic(ctx, watches, params.uri);
        });
    server.on_subscribe([watches = watches_](const std::string& uri) { watches->subscribe(uri); });
    server.on_unsubscribe([watches = watches_](const std::string& uri) { watches->unsubscribe(uri); });
    server.capabilities().resources->subscribe = true;
}

RosTools::~RosTools() { watches_->close(); }

std::string server_instructions(const Options& options) {
    std::string text =
        "A running ROS 2 system, seen from a node of its own. Start with list_nodes, list_topics, "
        "list_services and list_actions; node_info and topic_info show who talks to whom and with "
        "which QoS, and describe_interface shows the fields of any installed message, service or "
        "action type. echo reads messages, topic_hz measures rates, get_parameters reads a node's "
        "parameters, and lookup_transform and list_frames read /tf. Messages are JSON objects with "
        "the type's field names; long arrays in returned messages are shortened to {length, head}.";
    if (options.read_only) {
        text +=
            " This server is read-only: it does not publish, call services, send goals or set "
            "parameters.";
    } else {
        text +=
            " publish, call_service, send_action_goal and set_parameters act on the running system, "
            "which may move a robot: check the topic, type and values with the user before moving "
            "real hardware.";
    }
    return text;
}

}  // namespace mcp_ros2
