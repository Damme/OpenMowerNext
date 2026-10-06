// Mower settings of the web UI's Mower tab: speeds, blade, docking start point.
//
// Each setting is a ROS parameter of some node. The web UI sets it at runtime
// (SetParameters) and keeps it in one ROS parameter file (default
// /data/mower_settings.yaml), which om_launch loads into the nav and core processes
// at start (`--params-file ?path`, after the config files) and worx_hardware into its
// own node: a saved setting survives a restart. Only settings changed in the UI
// are in the file; everything else keeps its config default.
#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace open_mower_next::web_ui
{

struct SettingDef
{
  const char * key;    // name in the page and in messages
  const char * node;   // node that owns the parameter ("web_ui": this node)
  const char * param;  // its parameter
  bool integer;
  double min, max;
};

// The blade is two settings on worx_hardware's two parameters: mow_pwm (missions,
// magnitude) and manual_mow_pwm (by hand, sign = direction of both). blade_reverse
// has no parameter of its own: it is the sign of manual_mow_pwm.
inline const std::vector<SettingDef> & settingDefs()
{
  // Speeds stay <= 0.3 m/s: the velocity smoother's max_velocity (worx_nav2.yaml).
  static const std::vector<SettingDef> defs{
    {"speed_straight", "controller_server", "FTC.speed_fast", false, 0.05, 0.3},
    {"speed_curve", "controller_server", "FTC.speed_slow", false, 0.05, 0.3},
    {"turn_speed", "controller_server", "FTC.speed_angular", false, 10.0, 90.0},
    {"catchup_speed", "controller_server", "FTC.catchup_speed", false, 0.0, 0.15},
    {"transit_speed", "controller_server", "FollowPath.desired_linear_vel", false, 0.1, 0.3},
    {"feel_speed", "mower_logic", "feel_speed", false, 0.08, 0.3},
    {"manual_speed", "web_ui", "max_linear", false, 0.05, 0.3},
    {"dock_start", "docking_helper", "line_dock.approach_length", false, 1.7, 5.0},
    {"blade_pwm", "worx_hardware", "mow_pwm", true, 500, 1850},
    {"blade_reverse", "worx_hardware", "manual_mow_pwm", true, 0, 1},
  };
  return defs;
}

inline const SettingDef * findSetting(const std::string & key)
{
  for (const auto & d : settingDefs()) {
    if (key == d.key) return &d;
  }
  return nullptr;
}

// Settings (key -> value) as parameters of each node.
inline std::map<std::string, std::map<std::string, double>> settingsToParams(const std::map<std::string, double> & s)
{
  std::map<std::string, std::map<std::string, double>> out;
  for (const auto & [key, value] : s) {
    const auto * d = findSetting(key);
    if (!d || key == "blade_pwm" || key == "blade_reverse") continue;
    out[d->node][d->param] = value;
  }
  // Both blade parameters are always written together: the direction needs the PWM.
  const auto pwm = s.find("blade_pwm"), rev = s.find("blade_reverse");
  if (pwm != s.end() && rev != s.end()) {
    out["worx_hardware"]["mow_pwm"] = pwm->second;
    out["worx_hardware"]["manual_mow_pwm"] = rev->second > 0.5 ? -pwm->second : pwm->second;
  }
  return out;
}

// Parameters of a node (name -> value) back to settings.
inline void paramsToSettings(const std::string & node, const std::map<std::string, double> & params,
                             std::map<std::string, double> & s)
{
  for (const auto & d : settingDefs()) {
    if (node != d.node) continue;
    const auto it = params.find(d.param);
    if (it == params.end()) continue;
    if (std::string(d.key) == "blade_reverse") {
      s[d.key] = it->second < 0 ? 1.0 : 0.0;
    } else {
      s[d.key] = it->second;
    }
  }
}

inline std::string formatValue(double v, bool integer)
{
  char buf[32];
  if (integer) {
    std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(std::llround(v)));
    return buf;
  }
  // Always with a decimal point: a ROS parameter file types "3" as an integer.
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  std::string t = buf;
  while (t.size() > 1 && t.back() == '0' && t[t.size() - 2] != '.') t.pop_back();
  return t;
}

namespace detail
{
inline void flatten(const YAML::Node & n, const std::string & prefix, std::map<std::string, double> & out)
{
  for (const auto & kv : n) {
    const std::string name = prefix + kv.first.as<std::string>();
    if (kv.second.IsMap()) {
      flatten(kv.second, name + ".", out);
    } else if (kv.second.IsScalar()) {
      try {
        out[name] = kv.second.as<double>();
      } catch (const YAML::Exception &) {
      }
    }
  }
}
}  // namespace detail

// The settings in the file (missing/unreadable file: none).
inline std::map<std::string, double> readSettings(const std::string & path, std::string * error = nullptr)
{
  std::map<std::string, double> s;
  if (path.empty() || !std::ifstream(path).good()) return s;
  try {
    const YAML::Node root = YAML::LoadFile(path);
    for (const auto & kv : root) {
      const auto ros = kv.second["ros__parameters"];
      if (!ros || !ros.IsMap()) continue;
      std::map<std::string, double> params;
      detail::flatten(ros, "", params);
      paramsToSettings(kv.first.as<std::string>(), params, s);
    }
  } catch (const YAML::Exception & e) {
    if (error) *error = e.what();
  }
  return s;
}

// Text of the parameter file for these settings.
inline std::string settingsFileText(const std::map<std::string, double> & s)
{
  std::string out =
    "# Mower settings from the web UI (Mower tab), loaded at start by om_launch (nav, core)\n"
    "# and worx_hardware. Written by web_ui: change them there.\n";
  for (const auto & [node, params] : settingsToParams(s)) {
    out += node + ":\n  ros__parameters:\n";
    // a.b -> a: {b: ...}, grouped
    std::map<std::string, std::vector<std::pair<std::string, std::string>>> groups;
    for (const auto & [name, value] : params) {
      bool integer = false;
      for (const auto & d : settingDefs()) {
        if (node == d.node && name == d.param) integer = d.integer;
      }
      const auto dot = name.find('.');
      const std::string group = dot == std::string::npos ? std::string() : name.substr(0, dot);
      groups[group].emplace_back(dot == std::string::npos ? name : name.substr(dot + 1), formatValue(value, integer));
    }
    for (const auto & [group, entries] : groups) {
      const std::string indent = group.empty() ? "    " : "      ";
      if (!group.empty()) out += "    " + group + ":\n";
      for (const auto & [name, value] : entries) out += indent + name + ": " + value + "\n";
    }
  }
  return out;
}

// Written next to it and renamed: a process starting meanwhile never reads half a file.
inline bool writeSettings(const std::string & path, const std::map<std::string, double> & s, std::string & error)
{
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    out << settingsFileText(s);
    out.flush();
    if (!out) {
      error = "cannot write " + tmp;
      return false;
    }
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    error = "cannot rename " + tmp + " to " + path;
    std::remove(tmp.c_str());
    return false;
  }
  return true;
}

}  // namespace open_mower_next::web_ui
