#include "access.hpp"

#include <gtest/gtest.h>

namespace mcp_ros2 {
namespace {

TEST(NameMatchesTest, StarStaysWithinAPart) {
    EXPECT_TRUE(name_matches("/cmd_vel", "/cmd_vel"));
    EXPECT_FALSE(name_matches("/cmd_vel", "/cmd_vel2"));
    EXPECT_TRUE(name_matches("/robot1/*", "/robot1/cmd_vel"));
    EXPECT_FALSE(name_matches("/robot1/*", "/robot1/arm/joint_states"));
    EXPECT_TRUE(name_matches("/*_vel", "/cmd_vel"));
    EXPECT_TRUE(name_matches("/*/cmd_vel", "/robot2/cmd_vel"));
    EXPECT_FALSE(name_matches("/*/cmd_vel", "/cmd_vel"));
}

TEST(NameMatchesTest, DoubleStarCrossesParts) {
    EXPECT_TRUE(name_matches("/robot1/**", "/robot1/arm/joint_states"));
    EXPECT_TRUE(name_matches("/robot1/**", "/robot1/cmd_vel"));
    EXPECT_FALSE(name_matches("/robot1/**", "/robot1"));
    EXPECT_FALSE(name_matches("/robot1/**", "/robot2/cmd_vel"));
    EXPECT_TRUE(name_matches("/**/set_parameters", "/a/b/set_parameters"));
    EXPECT_TRUE(name_matches("/**", "/anything/at/all"));
}

TEST(AccessRulesTest, AllowsWhatMatchesAndDeniesFirst) {
    const AccessRules open;
    EXPECT_FALSE(open.limited());
    EXPECT_TRUE(open.permits("/cmd_vel"));
    EXPECT_NO_THROW(open.check("publish on", "/cmd_vel"));

    // Patterns without a leading slash get one.
    const AccessRules rules({"robot1/**", "/cmd_vel"}, {"/robot1/arm/**"});
    EXPECT_TRUE(rules.limited());
    EXPECT_TRUE(rules.permits("/cmd_vel"));
    EXPECT_TRUE(rules.permits("/robot1/base/cmd_vel"));
    EXPECT_FALSE(rules.permits("/robot1/arm/trajectory"));
    EXPECT_FALSE(rules.permits("/robot2/cmd_vel"));
    try {
        rules.check("publish on", "/robot2/cmd_vel");
        ADD_FAILURE() << "no AccessDenied";
    } catch (const AccessDenied& denied) {
        EXPECT_STREQ(
            denied.what(),
            "This server may not publish on /robot2/cmd_vel. The tools that act may only touch names "
            "matching /robot1/**, /cmd_vel, and none matching /robot1/arm/**.");
    }

    const AccessRules deny_only({}, {"/emergency_stop"});
    EXPECT_TRUE(deny_only.permits("/cmd_vel"));
    EXPECT_FALSE(deny_only.permits("/emergency_stop"));
    EXPECT_EQ(deny_only.describe(),
              "The tools that act may only touch names, and none matching /emergency_stop.");
}

}  // namespace
}  // namespace mcp_ros2
