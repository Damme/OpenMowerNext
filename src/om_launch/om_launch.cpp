// om_launch: starts and supervises the processes of a launch manifest (see
// manifest.hpp) instead of the python `ros2 launch`, which costs ~50 MB on
// the robot. Processes with `components:` run in om_container; others run
// their `cmd`. Output lines are prefixed with the process name.
//
//   om_launch <manifest.yaml> [--only name,name] [--skip name,name]
//
// Per process: respawn (default true), respawn_delay (s, default 2; doubles
// up to 30 s while a process keeps dying within 10 s), required (default
// false: its exit stops everything), delay (s before the first start), env,
// enabled (default true; see manifest.hpp).
// SIGINT/SIGTERM: SIGINT to all processes in reverse order, SIGKILL after 15 s.
#include "om_launch/manifest.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <set>

namespace
{
using namespace open_mower_next::om_launch;
using Clock = std::chrono::steady_clock;

int g_signal_pipe[2];

void onSignal(int sig)
{
  const char c = static_cast<char>(sig);
  (void)!::write(g_signal_pipe[1], &c, 1);
}

double secondsSince(Clock::time_point t)
{
  return std::chrono::duration<double>(Clock::now() - t).count();
}

void out(const std::string & line)
{
  (void)!::write(STDOUT_FILENO, line.data(), line.size());
}

void log(const std::string & msg)
{
  out("[om_launch] " + msg + "\n");
}

struct Process
{
  std::string name;
  std::vector<std::string> argv;
  std::map<std::string, std::string> env;
  bool respawn = true;
  bool required = false;
  double respawn_delay = 2.0;
  double backoff = 2.0;
  pid_t pid = -1;
  int fd = -1;
  std::string partial;
  Clock::time_point started;
  Clock::time_point next_start;
  bool pending = true;
  int restarts = 0;

  void flushLines(bool all)
  {
    size_t pos;
    while ((pos = partial.find('\n')) != std::string::npos) {
      out("[" + name + "] " + partial.substr(0, pos + 1));
      partial.erase(0, pos + 1);
    }
    if (all && !partial.empty()) {
      out("[" + name + "] " + partial + "\n");
      partial.clear();
    }
  }

  void start()
  {
    int p[2];
    if (::pipe2(p, O_CLOEXEC) != 0) {
      log(name + ": pipe failed: " + std::strerror(errno));
      return;
    }
    const pid_t child = ::fork();
    if (child < 0) {
      log(name + ": fork failed: " + std::strerror(errno));
      ::close(p[0]);
      ::close(p[1]);
      return;
    }
    if (child == 0) {
      ::signal(SIGINT, SIG_DFL);
      ::signal(SIGTERM, SIG_DFL);
      ::signal(SIGCHLD, SIG_DFL);
      const int devnull = ::open("/dev/null", O_RDONLY);
      ::dup2(devnull, STDIN_FILENO);
      ::dup2(p[1], STDOUT_FILENO);
      ::dup2(p[1], STDERR_FILENO);
      for (const auto & [k, v] : env) ::setenv(k.c_str(), v.c_str(), 1);
      // Children write to a pipe: keep ROS log lines unbuffered.
      ::setenv("RCUTILS_LOGGING_BUFFERED_STREAM", "0", 0);
      std::vector<char *> args;
      for (auto & a : argv) args.push_back(a.data());
      args.push_back(nullptr);
      ::execvp(args[0], args.data());
      std::fprintf(stderr, "exec %s failed: %s\n", args[0], std::strerror(errno));
      ::_exit(127);
    }
    ::close(p[1]);
    fd = p[0];
    ::fcntl(fd, F_SETFL, O_NONBLOCK);
    pid = child;
    started = Clock::now();
    pending = false;
    log("started " + name + " (pid " + std::to_string(pid) + ")");
  }
};

std::string selfDir()
{
  char buf[4096];
  const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n <= 0) return ".";
  buf[n] = '\0';
  return dirName(buf);
}

std::vector<Process> readProcesses(const YAML::Node & manifest, const std::string & manifest_path,
                                   const std::set<std::string> & only, const std::set<std::string> & skip)
{
  std::vector<Process> procs;
  const std::string container = selfDir() + "/om_container";
  for (const auto & p : manifest["processes"]) {
    Process proc;
    proc.name = p["name"].as<std::string>();
    if ((!only.empty() && !only.count(proc.name)) || skip.count(proc.name)) continue;
    if (p["components"]) {
      proc.argv = {container, manifest_path, proc.name};
    } else if (p["cmd"]) {
      proc.argv = p["cmd"].as<std::vector<std::string>>();
    } else {
      throw std::runtime_error("process " + proc.name + " has neither components nor cmd");
    }
    if (p["env"]) {
      for (const auto & kv : p["env"]) proc.env[kv.first.as<std::string>()] = kv.second.as<std::string>();
    }
    proc.respawn = p["respawn"] ? p["respawn"].as<bool>() : true;
    proc.required = p["required"] ? p["required"].as<bool>() : false;
    proc.respawn_delay = p["respawn_delay"] ? p["respawn_delay"].as<double>() : 2.0;
    proc.backoff = proc.respawn_delay;
    const double delay = p["delay"] ? p["delay"].as<double>() : 0.0;
    proc.next_start = Clock::now() + std::chrono::milliseconds(static_cast<int>(delay * 1000));
    procs.push_back(std::move(proc));
  }
  return procs;
}

std::string describeStatus(int status)
{
  if (WIFEXITED(status)) return "exit code " + std::to_string(WEXITSTATUS(status));
  if (WIFSIGNALED(status)) return std::string("signal ") + strsignal(WTERMSIG(status));
  return "status " + std::to_string(status);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: om_launch <manifest.yaml> [--only name,name] [--skip name,name]\n";
    return 2;
  }
  std::set<std::string> only, skip;
  for (int i = 2; i + 1 < argc; ++i) {
    const std::string opt = argv[i];
    if (opt == "--only" || opt == "--skip") {
      std::stringstream ss(argv[i + 1]);
      for (std::string n; std::getline(ss, n, ',');) (opt == "--only" ? only : skip).insert(n);
    }
  }

  std::vector<Process> procs;
  try {
    char resolved[4096];
    const std::string path = ::realpath(argv[1], resolved) ? resolved : argv[1];
    procs = readProcesses(loadManifest(path), path, only, skip);
  } catch (const std::exception & e) {
    std::cerr << "om_launch: " << e.what() << "\n";
    return 1;
  }

  if (::pipe2(g_signal_pipe, O_CLOEXEC | O_NONBLOCK) != 0) return 1;
  struct sigaction sa {};
  sa.sa_handler = onSignal;
  sa.sa_flags = SA_RESTART;
  ::sigaction(SIGINT, &sa, nullptr);
  ::sigaction(SIGTERM, &sa, nullptr);
  ::sigaction(SIGCHLD, &sa, nullptr);

  bool stopping = false;
  Clock::time_point stop_time;
  bool killed = false;

  auto beginStop = [&](const std::string & why) {
    if (stopping) return;
    stopping = true;
    stop_time = Clock::now();
    log("stopping (" + why + ")");
    for (auto it = procs.rbegin(); it != procs.rend(); ++it) {
      if (it->pid > 0) ::kill(it->pid, SIGINT);
    }
  };

  while (true) {
    // Start what is due.
    if (!stopping) {
      for (auto & p : procs) {
        if (p.pid < 0 && p.pending && Clock::now() >= p.next_start) p.start();
      }
    }
    const bool any_running = std::any_of(procs.begin(), procs.end(), [](const Process & p) { return p.pid > 0; });
    const bool any_pending = std::any_of(procs.begin(), procs.end(), [](const Process & p) { return p.pending; });
    if (stopping && !any_running) break;
    if (!any_running && !any_pending) {
      log("no process left to run");
      break;
    }
    if (stopping && !killed && secondsSince(stop_time) > 15.0) {
      killed = true;
      for (auto & p : procs) {
        if (p.pid > 0) {
          log("killing " + p.name);
          ::kill(p.pid, SIGKILL);
        }
      }
    }

    std::vector<pollfd> fds{{g_signal_pipe[0], POLLIN, 0}};
    std::vector<Process *> owners{nullptr};
    for (auto & p : procs) {
      if (p.fd >= 0) {
        fds.push_back({p.fd, POLLIN, 0});
        owners.push_back(&p);
      }
    }
    ::poll(fds.data(), fds.size(), 200);

    for (size_t i = 1; i < fds.size(); ++i) {
      if (!(fds[i].revents & (POLLIN | POLLHUP))) continue;
      Process & p = *owners[i];
      char buf[4096];
      ssize_t n;
      while ((n = ::read(p.fd, buf, sizeof(buf))) > 0) p.partial.append(buf, n);
      p.flushLines(false);
      if (n == 0) {  // writer closed (process gone or closed its output)
        p.flushLines(true);
        ::close(p.fd);
        p.fd = -1;
      }
    }

    if (fds[0].revents & POLLIN) {
      char sigs[64];
      const ssize_t n = ::read(g_signal_pipe[0], sigs, sizeof(sigs));
      for (ssize_t i = 0; i < n; ++i) {
        if (sigs[i] == SIGINT || sigs[i] == SIGTERM) {
          if (stopping) {
            for (auto & p : procs) {
              if (p.pid > 0) ::kill(p.pid, SIGKILL);
            }
          }
          beginStop(strsignal(sigs[i]));
        }
      }
    }

    // Reap.
    int status;
    pid_t pid;
    while ((pid = ::waitpid(-1, &status, WNOHANG)) > 0) {
      for (auto & p : procs) {
        if (p.pid != pid) continue;
        p.pid = -1;
        const double ran = secondsSince(p.started);
        log(p.name + " exited (" + describeStatus(status) + ") after " + std::to_string(static_cast<int>(ran)) + " s");
        if (stopping) break;
        if (p.required) {
          beginStop(p.name + " is required");
        } else if (p.respawn) {
          p.backoff = ran < 10.0 ? std::min(p.backoff * 2.0, 30.0) : p.respawn_delay;
          p.pending = true;
          p.next_start = Clock::now() + std::chrono::milliseconds(static_cast<int>(p.backoff * 1000));
          ++p.restarts;
          log("restarting " + p.name + " in " + std::to_string(static_cast<int>(p.backoff)) + " s (restart " +
              std::to_string(p.restarts) + ")");
        }
        break;
      }
    }
  }
  log("all processes stopped");
  return 0;
}
