// web_ui/settings.hpp: the Mower tab's settings <-> ROS parameter file.
#include "web_ui/settings.hpp"

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/allocator.h>

#include <cstdio>
#include <string>

using namespace open_mower_next::web_ui;

namespace
{
std::string tmpPath()
{
  return "/tmp/web_ui_settings_test_" + std::to_string(::getpid()) + ".yaml";
}
}  // namespace

TEST(WebUiSettings, FormatKeepsDoublesDouble)
{
  EXPECT_EQ(formatValue(3.0, false), "3.0");
  EXPECT_EQ(formatValue(0.25, false), "0.25");
  EXPECT_EQ(formatValue(0.1, false), "0.1");
  EXPECT_EQ(formatValue(1600.4, true), "1600");
}

TEST(WebUiSettings, BladeIsTwoParameters)
{
  auto p = settingsToParams({{"blade_pwm", 1600}, {"blade_reverse", 1}});
  EXPECT_EQ(p["worx_hardware"]["mow_pwm"], 1600);
  EXPECT_EQ(p["worx_hardware"]["manual_mow_pwm"], -1600);
  std::map<std::string, double> s;
  paramsToSettings("worx_hardware", {{"mow_pwm", 1500}, {"manual_mow_pwm", 1500}}, s);
  EXPECT_EQ(s["blade_pwm"], 1500);
  EXPECT_EQ(s["blade_reverse"], 0);
}

TEST(WebUiSettings, RoundTrip)
{
  const std::map<std::string, double> in{{"speed_straight", 0.25}, {"speed_curve", 0.1},  {"dock_start", 2.0},
                                         {"feel_speed", 0.12},     {"manual_speed", 0.2}, {"blade_pwm", 1600},
                                         {"blade_reverse", 1},     {"transit_speed", 0.3}};
  const auto path = tmpPath();
  std::string error;
  ASSERT_TRUE(writeSettings(path, in, error)) << error;
  EXPECT_EQ(readSettings(path), in);

  // rcl reads it as the nodes will: doubles stay doubles, dotted names nest.
  auto * params = rcl_yaml_node_struct_init(rcutils_get_default_allocator());
  ASSERT_TRUE(rcl_parse_yaml_file(path.c_str(), params)) << settingsFileText(in);
  auto * v = rcl_yaml_node_struct_get("controller_server", "FTC.speed_fast", params);
  ASSERT_NE(v, nullptr);
  ASSERT_NE(v->double_value, nullptr);
  EXPECT_DOUBLE_EQ(*v->double_value, 0.25);
  v = rcl_yaml_node_struct_get("docking_helper", "line_dock.approach_length", params);
  ASSERT_NE(v, nullptr);
  ASSERT_NE(v->double_value, nullptr);
  EXPECT_DOUBLE_EQ(*v->double_value, 2.0);
  v = rcl_yaml_node_struct_get("worx_hardware", "manual_mow_pwm", params);
  ASSERT_NE(v, nullptr);
  ASSERT_NE(v->integer_value, nullptr);
  EXPECT_EQ(*v->integer_value, -1600);
  rcl_yaml_node_struct_fini(params);
  std::remove(path.c_str());
}

TEST(WebUiSettings, MissingFileIsEmpty)
{
  EXPECT_TRUE(readSettings("/nonexistent/settings.yaml").empty());
}
