// Messages of real interface types as JSON and back: every field type, arrays of each kind, bounds,
// nested messages, and the errors that name the field at fault.

#include "message_json.hpp"
#include "type_support.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>

namespace mcp_ros2 {
namespace {

using nlohmann::json;

Stamp fixed_now() { return Stamp{1700000000, 250}; }

class MessageJsonTest : public ::testing::Test {
   protected:
    /// @p input written into a fresh message of @p type, read back as JSON.
    json round_trip(const std::string& type, const json& input, const JsonLimits& limits = {}) {
        const auto found = types_.message(type);
        DynamicMessage message(*found->members);
        from_json(input, *found->members, message.data(), fixed_now);
        return to_json(*found->members, message.data(), limits);
    }

    /// The error that writing @p input into a message of @p type gives.
    std::string error(const std::string& type, const json& input) {
        const auto found = types_.message(type);
        DynamicMessage message(*found->members);
        try {
            from_json(input, *found->members, message.data(), fixed_now);
        } catch (const MessageError& caught) {
            return caught.what();
        }
        ADD_FAILURE() << "no error for " << input.dump();
        return {};
    }

    TypeLibrary types_;
};

TEST_F(MessageJsonTest, ReadsDefaults) {
    const json defaults = round_trip("test_msgs/msg/Defaults", json::object());
    EXPECT_EQ(defaults["bool_value"], true);
    EXPECT_EQ(defaults["byte_value"], 50);
    EXPECT_EQ(defaults["int8_value"], -50);
    EXPECT_EQ(defaults["uint64_value"], 50000000);
    EXPECT_DOUBLE_EQ(defaults["float64_value"].get<double>(), 1.125);

    const json strings = round_trip("test_msgs/msg/Strings", json::object());
    EXPECT_EQ(strings["string_value_default3"], "Hello\"world!");
}

TEST_F(MessageJsonTest, WritesEveryBasicType) {
    const json input{
        {"bool_value", true},           {"byte_value", 255},           {"char_value", 65},
        {"float32_value", 1.5},         {"float64_value", -2.25},      {"int8_value", -128},
        {"uint8_value", 255},           {"int16_value", -32768},       {"uint16_value", 65535},
        {"int32_value", -2147483648LL}, {"uint32_value", 4294967295U}, {"int64_value", INT64_MIN},
        {"uint64_value", UINT64_MAX}};
    EXPECT_EQ(round_trip("test_msgs/msg/BasicTypes", input), input);
}

TEST_F(MessageJsonTest, WritesEveryKindOfArray) {
    const json fixed =
        round_trip("test_msgs/msg/Arrays", {{"int32_values", {1, 2, 3}},
                                            {"bool_values", {true, false, true}},
                                            {"string_values", {"a", "b", "c"}},
                                            {"basic_types_values", {{{"int8_value", 7}}}}});
    EXPECT_EQ(fixed["int32_values"], json({1, 2, 3}));
    EXPECT_EQ(fixed["bool_values"], json({true, false, true}));
    EXPECT_EQ(fixed["string_values"], json({"a", "b", "c"}));
    EXPECT_EQ(fixed["basic_types_values"][0]["int8_value"], 7);
    // Values left out of a fixed array keep their defaults.
    EXPECT_EQ(fixed["basic_types_values"][1]["int8_value"], 0);

    const json bounded = round_trip("test_msgs/msg/BoundedSequences",
                                    {{"float64_values", {0.5, 1.5}}, {"bool_values", {false, true}}});
    EXPECT_EQ(bounded["float64_values"], json({0.5, 1.5}));
    EXPECT_EQ(bounded["bool_values"], json({false, true}));
    EXPECT_EQ(bounded["int8_values"], json::array());

    const json unbounded =
        round_trip("test_msgs/msg/UnboundedSequences", {{"uint64_values", {1, 2, 3, 4, 5}},
                                                        {"string_values", {"x"}},
                                                        {"defaults_values", {json::object()}}});
    EXPECT_EQ(unbounded["uint64_values"], json({1, 2, 3, 4, 5}));
    EXPECT_EQ(unbounded["string_values"], json({"x"}));
    EXPECT_EQ(unbounded["defaults_values"][0]["int8_value"], -50);
}

TEST_F(MessageJsonTest, ShortensLongArrays) {
    json values = json::array();
    for (int i = 0; i < 10; ++i) {
        values.push_back(i);
    }
    const json out =
        round_trip("test_msgs/msg/UnboundedSequences", {{"int32_values", values}}, JsonLimits{4});
    EXPECT_EQ(out["int32_values"], json({{"length", 10}, {"head", {0, 1, 2, 3}}}));
}

TEST_F(MessageJsonTest, WritesNestedMessages) {
    const json out =
        round_trip("geometry_msgs/msg/PoseStamped",
                   {{"header", {{"frame_id", "map"}}}, {"pose", {{"position", {{"x", 1.0}}}}}});
    EXPECT_EQ(out["header"]["frame_id"], "map");
    EXPECT_DOUBLE_EQ(out["pose"]["position"]["x"].get<double>(), 1.0);
    EXPECT_DOUBLE_EQ(out["pose"]["orientation"]["w"].get<double>(), 1.0);
}

TEST_F(MessageJsonTest, FillsInNowForATime) {
    const json out = round_trip("std_msgs/msg/Header", {{"stamp", "now"}});
    EXPECT_EQ(out["stamp"], json({{"sec", 1700000000}, {"nanosec", 250}}));
}

TEST_F(MessageJsonTest, ConvertsWideStrings) {
    const json input{{"wstring_value", "Hellö ハロー 🙂"},
                     {"unbounded_sequence_of_wstrings", {"ä", "😀"}}};
    const json out = round_trip("test_msgs/msg/WStrings", input);
    EXPECT_EQ(out["wstring_value"], input["wstring_value"]);
    EXPECT_EQ(out["unbounded_sequence_of_wstrings"], input["unbounded_sequence_of_wstrings"]);
    EXPECT_EQ(out["wstring_value_default3"], "ハローワールド");
}

TEST_F(MessageJsonTest, ReplacesInvalidUtf8) {
    const json out = round_trip("std_msgs/msg/String", {{"data", std::string("ok \xFF\xC0 end")}});
    EXPECT_EQ(out["data"], "ok \xEF\xBF\xBD\xEF\xBF\xBD end");
    EXPECT_NO_THROW((void)out.dump());
}

TEST_F(MessageJsonTest, TakesNonFiniteNumbersAsText) {
    const auto found = types_.message("std_msgs/msg/Float64");
    DynamicMessage message(*found->members);
    from_json({{"data", "inf"}}, *found->members, message.data(), fixed_now);
    EXPECT_TRUE(std::isinf(*static_cast<const double*>(message.data())));
    EXPECT_EQ(to_json(*found->members, message.data()), json({{"data", "inf"}}));
    EXPECT_EQ(round_trip("std_msgs/msg/Float32", {{"data", "-inf"}}), json({{"data", "-inf"}}));
    EXPECT_EQ(round_trip("std_msgs/msg/Float64", {{"data", "nan"}}), json({{"data", "nan"}}));
}

TEST_F(MessageJsonTest, ShowsFloatsInTheirShortestForm) {
    EXPECT_EQ(round_trip("std_msgs/msg/Float32", {{"data", 0.1}}).dump(), R"({"data":0.1})");
    EXPECT_EQ(round_trip("std_msgs/msg/Float32", {{"data", 5.5444446}}).dump(),
              R"({"data":5.5444446})");
    EXPECT_EQ(round_trip("std_msgs/msg/Float64", {{"data", 0.1}}).dump(), R"({"data":0.1})");
}

TEST_F(MessageJsonTest, ErrorsNameTheField) {
    EXPECT_EQ(error("geometry_msgs/msg/Twist", {{"linear", {{"q", 1}}}}),
              "linear.q: geometry_msgs/msg/Vector3 has no such field; its fields are x, y, z");
    EXPECT_EQ(error("geometry_msgs/msg/Twist", {{"angular", {{"z", "fast"}}}}),
              "angular.z: expected a number, not \"fast\"");
    EXPECT_EQ(error("test_msgs/msg/BasicTypes", {{"uint8_value", 256}}),
              "uint8_value: the value 256 is out of range");
    EXPECT_EQ(error("test_msgs/msg/BasicTypes", {{"int8_value", -129}}),
              "int8_value: the value -129 is out of range");
    EXPECT_EQ(error("test_msgs/msg/BasicTypes", {{"bool_value", 1}}),
              "bool_value: expected true or false, not 1");
    EXPECT_EQ(error("test_msgs/msg/Arrays", {{"int32_values", {1, 2, 3, 4}}}),
              "int32_values: holds 3 values, not 4");
    EXPECT_EQ(error("test_msgs/msg/BoundedSequences", {{"int32_values", {1, 2, 3, 4}}}),
              "int32_values: holds at most 3 values, not 4");
    EXPECT_EQ(error("test_msgs/msg/Strings", {{"bounded_string_value", std::string(23, 'x')}}),
              "bounded_string_value: longer than its bound of 22");
    EXPECT_EQ(
        error("test_msgs/msg/UnboundedSequences", {{"basic_types_values", {{{"int8_value", "x"}}}}}),
        "basic_types_values[0].int8_value: expected an integer, not \"x\"");
    EXPECT_EQ(error("std_msgs/msg/String", json::array()),
              "the message: expected an object of fields for std_msgs/msg/String, not []");
}

TEST_F(MessageJsonTest, DescribesTypes) {
    const json twist = describe(*types_.message("geometry_msgs/Twist")->members);
    EXPECT_EQ(twist["linear"]["type"], "geometry_msgs/msg/Vector3");
    EXPECT_EQ(twist["linear"]["fields"]["x"], "float64");

    const json bounded = describe(*types_.message("test_msgs/msg/BoundedSequences")->members);
    EXPECT_EQ(bounded["int32_values"], "int32[<=3]");
    EXPECT_EQ(bounded["byte_values"], "byte[<=3]");
    const json strings = describe(*types_.message("test_msgs/msg/Strings")->members);
    EXPECT_EQ(strings["bounded_string_value"], "string<=22");
    const json arrays = describe(*types_.message("test_msgs/msg/Arrays")->members);
    EXPECT_EQ(arrays["float32_values"], "float32[3]");
    EXPECT_EQ(arrays["basic_types_values"]["type"], "test_msgs/msg/BasicTypes[3]");
}

TEST(TypeLibraryTest, NamesTypes) {
    EXPECT_EQ(full_type_name("std_msgs/String", "msg"), "std_msgs/msg/String");
    EXPECT_EQ(full_type_name("std_srvs/srv/Trigger", "srv"), "std_srvs/srv/Trigger");
    EXPECT_THROW((void)full_type_name("std_msgs", "msg"), TypeError);
    EXPECT_THROW((void)full_type_name("std_msgs//String", "msg"), TypeError);
    EXPECT_THROW((void)full_type_name("std_msgs/srv/String", "msg"), TypeError);

    TypeLibrary types;
    EXPECT_EQ(types.message("std_msgs/String")->name, "std_msgs/msg/String");
    EXPECT_THROW((void)types.message("no_such_package/msg/Nothing"), TypeError);
    EXPECT_EQ(types.action("example_interfaces/Fibonacci")->name,
              "example_interfaces/action/Fibonacci");
    EXPECT_NE(types.service("example_interfaces/srv/AddTwoInts")->members, nullptr);
}

}  // namespace
}  // namespace mcp_ros2
