// Launch manifest shared by om_launch (process supervisor) and om_container
// (component host). A manifest is a YAML file:
//
//   env:                      # set for every process (after substitution)
//     MALLOC_ARENA_MAX: "2"
//   processes:
//     - name: nav             # hosted by om_container: nodes loaded in-process
//       ros_args: [--params-file, "$(find-pkg-share open_mower_next)/config/nav2_params.yaml"]
//       components:
//         - plugin: nav2_planner::PlannerServer
//           name: planner_server
//     - name: gps             # any other program
//       enabled: $(env OM_GPS_ENABLED true)
//       cmd: [/usr/bin/str2str, -in, ...]
//
// Substitutions (in the manifest and in parameter files):
//   $(env NAME)  $(env NAME default)  $(find-pkg-share pkg)  $(dirname)
// No ROS libraries here: om_launch must stay tiny.
#pragma once

#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace open_mower_next::om_launch
{

inline bool fileExists(const std::string & path)
{
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0;
}

inline std::string readFile(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

inline std::string dirName(const std::string & path)
{
  const auto slash = path.rfind('/');
  return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

// <prefix>/share/<pkg> of the first AMENT_PREFIX_PATH entry that has the package.
inline std::string findPackageShare(const std::string & pkg)
{
  const char * env = std::getenv("AMENT_PREFIX_PATH");
  std::stringstream ss(env ? env : "");
  for (std::string prefix; std::getline(ss, prefix, ':');) {
    if (!prefix.empty() && fileExists(prefix + "/share/ament_index/resource_index/packages/" + pkg)) {
      return prefix + "/share/" + pkg;
    }
  }
  throw std::runtime_error("package not found in AMENT_PREFIX_PATH: " + pkg);
}

// Expands $(...) in text. `dir` is what $(dirname) returns.
inline std::string substitute(const std::string & text, const std::string & dir)
{
  std::string out;
  size_t pos = 0;
  while (true) {
    const auto start = text.find("$(", pos);
    if (start == std::string::npos) break;
    const auto end = text.find(')', start);
    if (end == std::string::npos) throw std::runtime_error("unterminated $( in: " + text.substr(start, 40));
    out += text.substr(pos, start - pos);
    // "cmd arg [default...]"; a default of "" means empty.
    const std::string inner = text.substr(start + 2, end - start - 2);
    const auto s1 = inner.find(' ');
    const std::string cmd = inner.substr(0, s1);
    const std::string rest = s1 == std::string::npos ? std::string() : inner.substr(s1 + 1);
    const auto s2 = rest.find(' ');
    const std::string arg = rest.substr(0, s2);
    const bool has_default = s2 != std::string::npos;
    std::string def = has_default ? rest.substr(s2 + 1) : std::string();
    if (def == "\"\"") def.clear();
    if (cmd == "env") {
      const char * v = std::getenv(arg.c_str());
      if (v) {
        out += v;
      } else if (has_default) {
        out += def;
      } else {
        throw std::runtime_error("environment variable not set: " + arg);
      }
    } else if (cmd == "find-pkg-share") {
      out += findPackageShare(arg);
    } else if (cmd == "dirname") {
      out += dir;
    } else {
      throw std::runtime_error("unknown substitution $(" + cmd + ")");
    }
    pos = end + 1;
  }
  out += text.substr(pos);
  return out;
}

// Reads the manifest: first sets its `env:` entries that aren't set already
// (so they can be overridden from outside), then parses the substituted text.
inline void substituteScalars(YAML::Node node, const std::string & dir)
{
  if (node.IsScalar()) {
    const auto v = node.Scalar();
    if (v.find("$(") != std::string::npos) node = substitute(v, dir);
  } else if (node.IsSequence()) {
    for (auto child : node) substituteScalars(child, dir);
  } else if (node.IsMap()) {
    for (auto kv : node) substituteScalars(kv.second, dir);
  }
}

inline YAML::Node loadManifest(const std::string & path)
{
  const std::string dir = dirName(path);
  YAML::Node m = YAML::LoadFile(path);
  if (m["env"]) {
    for (const auto & kv : m["env"]) {
      const auto key = kv.first.as<std::string>();
      if (!std::getenv(key.c_str())) {
        ::setenv(key.c_str(), substitute(kv.second.as<std::string>(), dir).c_str(), 1);
      }
    }
  }
  // Disabled processes go before the rest is expanded: their $(...) may name
  // packages that aren't installed.
  if (m["processes"]) {
    YAML::Node kept(YAML::NodeType::Sequence);
    for (const auto & p : m["processes"]) {
      YAML::Node proc(p);
      if (proc["enabled"]) {
        substituteScalars(proc["enabled"], dir);
        if (!proc["enabled"].as<bool>()) continue;
      }
      kept.push_back(proc);
    }
    m["processes"] = kept;
  }
  substituteScalars(m, dir);
  return m;
}

}  // namespace open_mower_next::om_launch
