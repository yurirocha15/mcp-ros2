// The server binary end to end: an MCP client launches it over stdio, as an agent's client does,
// next to a node that stands in for a robot, with a topic it publishes, topics it listens to, a
// service, an action, parameters and a static transform.

#include <mcp/mcp.hpp>

#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace {

using nlohmann::json;
using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using FibonacciHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

/// A node with a bit of everything a robot has.
class Robot {
   public:
    Robot() : node_(std::make_shared<rclcpp::Node>("test_robot")) {
        node_->declare_parameter("speed", 1.0);
        node_->declare_parameter("name", "robot");
        node_->declare_parameter("count", 3);
        node_->declare_parameter("flags", std::vector<bool>{true, false});

        chatter_ = node_->create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = node_->create_wall_timer(50ms, [this] {
            std_msgs::msg::String message;
            message.data = "hello " + std::to_string(sent_++);
            chatter_->publish(message);
        });
        commands_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "commands", 10, [this](const geometry_msgs::msg::Twist& message) {
                const std::lock_guard lock(mutex_);
                last_command_ = message;
            });
        stamped_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
            "stamped", 10, [this](const geometry_msgs::msg::TwistStamped& message) {
                const std::lock_guard lock(mutex_);
                last_stamped_ = message;
            });
        adder_ = node_->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            [](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
               std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
                response->sum = request->a + request->b;
            });
        fibonacci_ = rclcpp_action::create_server<Fibonacci>(
            node_, "fibonacci",
            [](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Fibonacci::Goal>& goal) {
                return goal->order >= 0 ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                                        : rclcpp_action::GoalResponse::REJECT;
            },
            [](const std::shared_ptr<FibonacciHandle>&) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<FibonacciHandle>& handle) {
                const std::lock_guard lock(mutex_);
                workers_.emplace_back([this, handle] { run_fibonacci(handle); });
            });

        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 2.0;
        transform.transform.rotation.z = std::sin(M_PI / 4);
        transform.transform.rotation.w = std::cos(M_PI / 4);
        broadcaster_->sendTransform(transform);

        executor_.add_node(node_);
        spinner_ = std::thread([this] { executor_.spin(); });
    }

    ~Robot() {
        stopping_ = true;
        {
            const std::lock_guard lock(mutex_);
            for (auto& worker : workers_) {
                worker.join();
            }
        }
        executor_.cancel();
        spinner_.join();
    }

    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    rclcpp::Node& node() { return *node_; }

    std::optional<geometry_msgs::msg::Twist> last_command() {
        const std::lock_guard lock(mutex_);
        return last_command_;
    }

    std::optional<geometry_msgs::msg::TwistStamped> last_stamped() {
        const std::lock_guard lock(mutex_);
        return last_stamped_;
    }

   private:
    void run_fibonacci(const std::shared_ptr<FibonacciHandle>& handle) {
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto result = std::make_shared<Fibonacci::Result>();
        feedback->sequence = {0};
        for (int i = 1; i <= handle->get_goal()->order; ++i) {
            if (stopping_) {
                result->sequence = feedback->sequence;
                handle->abort(result);
                return;
            }
            if (handle->is_canceling()) {
                result->sequence = feedback->sequence;
                handle->canceled(result);
                return;
            }
            const auto size = feedback->sequence.size();
            feedback->sequence.push_back(
                size < 2 ? 1 : feedback->sequence[size - 1] + feedback->sequence[size - 2]);
            handle->publish_feedback(feedback);
            std::this_thread::sleep_for(20ms);
        }
        result->sequence = feedback->sequence;
        handle->succeed(result);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor executor_;
    std::thread spinner_;
    std::atomic<bool> stopping_{false};
    std::atomic<int> sent_{0};
    std::mutex mutex_;
    std::optional<geometry_msgs::msg::Twist> last_command_;
    std::optional<geometry_msgs::msg::TwistStamped> last_stamped_;
    std::vector<std::thread> workers_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr adder_;
    rclcpp_action::Server<Fibonacci>::SharedPtr fibonacci_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

Robot* g_robot = nullptr;

class RosEnvironment : public ::testing::Environment {
   public:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        robot_ = std::make_unique<Robot>();
        g_robot = robot_.get();
    }
    void TearDown() override {
        g_robot = nullptr;
        robot_.reset();
        rclcpp::shutdown();
    }

   private:
    std::unique_ptr<Robot> robot_;
};

const auto* const g_ENVIRONMENT = ::testing::AddGlobalTestEnvironment(new RosEnvironment);

bool contains_name(const json& list, const std::string& name) {
    for (const json& item : list) {
        if ((item.is_string() && item == name) ||
            (item.is_object() && item.value("name", "") == name)) {
            return true;
        }
    }
    return false;
}

template <typename Ready>
bool wait_for(Ready ready, std::chrono::seconds timeout = 10s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!ready()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(100ms);
    }
    return true;
}

class ServerTest : public ::testing::Test {
   protected:
    /// Starts the server and waits until it sees the robot's topics, service and action.
    void connect(const std::vector<std::string>& extra_args = {},
                 mcp::ClientProtocol protocol = mcp::ClientProtocol::automatic()) {
        std::vector<std::string> command{MCP_ROS2_BINARY};
        command.insert(command.end(), extra_args.begin(), extra_args.end());
        mcp::ClientOptions options;
        options.request_timeout = std::chrono::minutes(2);
        options.protocol = std::move(protocol);
        client_ = std::make_unique<mcp::Client>(mcp::Endpoint::stdio(command), options);
        client_->sync().connect("mcp-ros2-test", "1.0.0");
        ASSERT_TRUE(wait_for([this] {
            return contains_name(call("list_topics")["topics"], "/chatter") &&
                   contains_name(call("list_topics")["topics"], "/commands") &&
                   contains_name(call("list_services")["services"], "/add_two_ints") &&
                   contains_name(call("list_actions")["actions"], "/fibonacci");
        })) << "the server did not discover the robot";
    }

    void TearDown() override { client_.reset(); }

    json call(const std::string& name, const json& args = json::object()) {
        const mcp::CallToolResult result = client_->sync().call_tool(name, args);
        EXPECT_FALSE(result.isError.value_or(false)) << name << ": " << mcp::result_text(result);
        return result.structuredContent.value_or(json::object());
    }

    std::string call_error(const std::string& name, const json& args = json::object()) {
        const mcp::CallToolResult result = client_->sync().call_tool(name, args);
        EXPECT_TRUE(result.isError.value_or(false))
            << name << " succeeded: " << mcp::result_text(result);
        return mcp::result_text(result);
    }

    std::vector<std::string> tool_names() {
        std::vector<std::string> names;
        for (const auto& tool : client_->sync().list_tools().tools) {
            names.push_back(tool.name);
        }
        return names;
    }

    std::unique_ptr<mcp::Client> client_;
};

TEST_F(ServerTest, ListsTheGraph) {
    connect();
    EXPECT_TRUE(contains_name(call("list_nodes")["nodes"], "/test_robot"));
    const json topics = call("list_topics")["topics"];
    for (const json& topic : topics) {
        if (topic["name"] == "/chatter") {
            EXPECT_EQ(topic["types"], json({"std_msgs/msg/String"}));
            EXPECT_GE(topic["publishers"], 1);
        }
    }
    // Hidden topics and every node's parameter services are left out unless asked for.
    EXPECT_FALSE(contains_name(topics, "/fibonacci/_action/feedback"));
    EXPECT_TRUE(contains_name(call("list_topics", {{"hidden", true}})["topics"],
                              "/fibonacci/_action/feedback"));
    EXPECT_FALSE(contains_name(call("list_services")["services"], "/test_robot/get_parameters"));
    EXPECT_TRUE(contains_name(call("list_services", {{"all", true}})["services"],
                              "/test_robot/get_parameters"));
    const json actions = call("list_actions")["actions"];
    ASSERT_TRUE(contains_name(actions, "/fibonacci"));
}

TEST_F(ServerTest, DescribesNodesAndTopics) {
    connect();
    const json node = call("node_info", {{"node", "test_robot"}});
    EXPECT_EQ(node["node"], "/test_robot");
    EXPECT_TRUE(contains_name(node["publishers"], "/chatter"));
    EXPECT_TRUE(contains_name(node["subscribers"], "/commands"));
    EXPECT_TRUE(contains_name(node["services"], "/add_two_ints"));
    EXPECT_TRUE(contains_name(node["action_servers"], "/fibonacci"));
    EXPECT_NE(call_error("node_info", {{"node", "/nobody"}}).find("There is no node /nobody"),
              std::string::npos);

    const json topic = call("topic_info", {{"topic", "/chatter"}});
    ASSERT_FALSE(topic["publishers"].empty());
    const json& publisher = topic["publishers"][0];
    EXPECT_EQ(publisher["node"], "/test_robot");
    EXPECT_EQ(publisher["type"], "std_msgs/msg/String");
    EXPECT_EQ(publisher["qos"]["reliability"], "reliable");
    EXPECT_EQ(publisher["qos"]["durability"], "volatile");
}

TEST_F(ServerTest, DescribesInterfaces) {
    connect();
    const json twist = call("describe_interface", {{"type", "geometry_msgs/msg/Twist"}});
    EXPECT_EQ(twist["kind"], "message");
    EXPECT_EQ(twist["fields"]["linear"]["fields"]["x"], "float64");

    const json adder = call("describe_interface", {{"type", "example_interfaces/AddTwoInts"}});
    EXPECT_EQ(adder["kind"], "service");
    EXPECT_EQ(adder["type"], "example_interfaces/srv/AddTwoInts");
    EXPECT_EQ(adder["request"]["a"], "int64");
    EXPECT_EQ(adder["response"]["sum"], "int64");

    const json fibonacci =
        call("describe_interface", {{"type", "example_interfaces/action/Fibonacci"}});
    EXPECT_EQ(fibonacci["goal"]["order"], "int32");
    EXPECT_EQ(fibonacci["result"]["sequence"], "int32[]");

    EXPECT_NE(call_error("describe_interface", {{"type", "no_such_pkg/Thing"}}).find("no installed"),
              std::string::npos);
}

TEST_F(ServerTest, EchoesMessages) {
    connect();
    const json echoed = call("echo", {{"topic", "/chatter"}, {"count", 3}});
    EXPECT_EQ(echoed["type"], "std_msgs/msg/String");
    ASSERT_EQ(echoed["messages"].size(), 3U);
    EXPECT_EQ(echoed["messages"][0]["data"].get<std::string>().rfind("hello ", 0), 0U);

    const json field = call("echo", {{"topic", "chatter"}, {"field", "data"}});
    ASSERT_EQ(field["messages"].size(), 1U);
    EXPECT_TRUE(field["messages"][0].is_string());

    const json silent =
        call("echo", {{"topic", "/silent"}, {"type", "std_msgs/msg/String"}, {"timeout", 0.3}});
    EXPECT_TRUE(silent["messages"].empty());
    EXPECT_NE(silent["note"].get<std::string>().find("Timed out"), std::string::npos);

    EXPECT_NE(call_error("echo", {{"topic", "/nothing"}}).find("give the type"), std::string::npos);
}

TEST_F(ServerTest, MeasuresARate) {
    connect();
    const json stats = call("topic_hz", {{"topic", "/chatter"}, {"seconds", 2}});
    EXPECT_GT(stats["rate_hz"].get<double>(), 10.0) << stats.dump();
    EXPECT_LT(stats["rate_hz"].get<double>(), 40.0) << stats.dump();
    EXPECT_GT(stats["bytes_per_second"].get<double>(), 0.0);
}

TEST_F(ServerTest, PublishesMessages) {
    connect();
    const json published =
        call("publish", {{"topic", "/commands"}, {"message", {{"linear", {{"x", 0.5}}}}}});
    EXPECT_EQ(published["published"], 1);
    EXPECT_EQ(published["type"], "geometry_msgs/msg/Twist");
    ASSERT_TRUE(wait_for([] {
        const auto command = g_robot->last_command();
        return command && command->linear.x == 0.5;
    }));

    call("publish", {{"topic", "/stamped"},
                     {"message", {{"header", {{"stamp", "now"}, {"frame_id", "base_link"}}}}},
                     {"times", 3},
                     {"rate", 20}});
    ASSERT_TRUE(wait_for([] { return g_robot->last_stamped().has_value(); }));
    EXPECT_GT(g_robot->last_stamped()->header.stamp.sec, 0);
    EXPECT_EQ(g_robot->last_stamped()->header.frame_id, "base_link");

    EXPECT_NE(call_error("publish", {{"topic", "/commands"}, {"message", {{"linear", {{"q", 1}}}}}})
                  .find("linear.q: geometry_msgs/msg/Vector3 has no such field"),
              std::string::npos);
}

TEST_F(ServerTest, CallsServices) {
    connect();
    const json called =
        call("call_service", {{"service", "/add_two_ints"}, {"request", {{"a", 2}, {"b", 40}}}});
    EXPECT_EQ(called["response"]["sum"], 42);
    EXPECT_NE(call_error("call_service", {{"service", "/nobody_home"},
                                          {"type", "example_interfaces/srv/AddTwoInts"},
                                          {"timeout", 0.5}})
                  .find("is not available"),
              std::string::npos);
}

TEST_F(ServerTest, RunsActionGoals) {
    connect();
    const json done = call("send_action_goal", {{"action", "/fibonacci"}, {"goal", {{"order", 5}}}});
    EXPECT_EQ(done["accepted"], true);
    EXPECT_EQ(done["status"], "succeeded");
    EXPECT_EQ(done["result"]["sequence"], json({0, 1, 1, 2, 3, 5}));
    EXPECT_GE(done["feedback_count"], 1);

    // A goal still running at the timeout is cancelled.
    const json cut = call("send_action_goal",
                          {{"action", "/fibonacci"}, {"goal", {{"order", 1000}}}, {"timeout", 1}});
    EXPECT_EQ(cut["status"], "canceled") << cut.dump();
    EXPECT_NE(cut["note"].get<std::string>().find("cancel"), std::string::npos);

    const json rejected =
        call("send_action_goal", {{"action", "/fibonacci"}, {"goal", {{"order", -1}}}});
    EXPECT_EQ(rejected["accepted"], false);
}

TEST_F(ServerTest, ReadsAndSetsParameters) {
    connect();
    const json all = call("get_parameters", {{"node", "/test_robot"}})["parameters"];
    EXPECT_EQ(all["speed"], 1.0);
    EXPECT_EQ(all["name"], "robot");
    EXPECT_EQ(all["count"], 3);
    EXPECT_EQ(all["flags"], json({true, false}));

    // An integer for a double parameter becomes a double.
    const json set = call("set_parameters", {{"node", "/test_robot"}, {"parameters", {{"speed", 2}}}});
    EXPECT_EQ(set["results"]["speed"]["successful"], true);
    EXPECT_DOUBLE_EQ(g_robot->node().get_parameter("speed").as_double(), 2.0);
    const json named =
        call("get_parameters", {{"node", "/test_robot"}, {"names", {"speed"}}})["parameters"];
    EXPECT_EQ(named, json({{"speed", 2.0}}));
    call("set_parameters", {{"node", "/test_robot"}, {"parameters", {{"speed", 1.0}}}});

    EXPECT_NE(call_error("set_parameters", {{"node", "/test_robot"}, {"parameters", {{"name", 5}}}})
                  .find("name: expected a string"),
              std::string::npos);
    EXPECT_NE(
        call_error("get_parameters", {{"node", "/nobody"}, {"timeout", 0.5}}).find("does not answer"),
        std::string::npos);
}

TEST_F(ServerTest, LooksUpTransforms) {
    connect();
    const json transform = call(
        "lookup_transform", {{"target_frame", "map"}, {"source_frame", "base_link"}, {"timeout", 5}});
    EXPECT_DOUBLE_EQ(transform["translation"]["x"].get<double>(), 1.0);
    EXPECT_DOUBLE_EQ(transform["translation"]["y"].get<double>(), 2.0);
    EXPECT_NEAR(transform["rpy_degrees"]["yaw"].get<double>(), 90.0, 0.01);
    const json frames = call("list_frames")["frames"];
    EXPECT_TRUE(contains_name(frames, "map"));
    EXPECT_TRUE(contains_name(frames, "base_link"));
    EXPECT_NE(call_error("lookup_transform",
                         {{"target_frame", "map"}, {"source_frame", "moon"}, {"timeout", 0.3}})
                  .find("No transform from moon to map"),
              std::string::npos);
}

TEST_F(ServerTest, ReadOnlyLeavesOutWhatActs) {
    connect({"--read-only"});
    const auto names = tool_names();
    for (const char* acting : {"publish", "call_service", "send_action_goal", "set_parameters"}) {
        EXPECT_EQ(std::find(names.begin(), names.end(), acting), names.end()) << acting;
    }
    for (const char* reading : {"echo", "get_parameters", "lookup_transform"}) {
        EXPECT_NE(std::find(names.begin(), names.end(), reading), names.end()) << reading;
    }
}

TEST_F(ServerTest, ServesTopicsAsResources) {
    connect();
    const auto read = client_->sync().read_resource("ros://topic/chatter");
    ASSERT_EQ(read.contents.size(), 1U);
    const auto* text = std::get_if<mcp::TextResourceContents>(&read.contents[0]);
    ASSERT_NE(text, nullptr);
    const json body = json::parse(text->text);
    EXPECT_EQ(body["type"], "std_msgs/msg/String");
    EXPECT_EQ(body["message"]["data"].get<std::string>().rfind("hello ", 0), 0U);

    const auto graph = client_->sync().read_resource("ros://graph");
    const auto* graph_text = std::get_if<mcp::TextResourceContents>(&graph.contents[0]);
    ASSERT_NE(graph_text, nullptr);
    EXPECT_TRUE(contains_name(json::parse(graph_text->text)["nodes"], "/test_robot"));
}

TEST_F(ServerTest, NotifiesSubscribersOfNewMessages) {
    connect({}, mcp::ClientProtocol::legacy());
    std::atomic<int> updates{0};
    client_->on_notification("notifications/resources/updated", [&updates](const json& params) {
        if (params.value("uri", "") == "ros://topic/chatter") {
            ++updates;
        }
    });
    client_->sync().send_request("resources/subscribe", json{{"uri", "ros://topic/chatter"}});
    // At most one notice a second.
    EXPECT_TRUE(wait_for([&updates] { return updates >= 2; }, 5s)) << updates;
    client_->sync().send_request("resources/unsubscribe", json{{"uri", "ros://topic/chatter"}});
}

}  // namespace
