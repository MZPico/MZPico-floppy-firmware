#include "testfw.hpp"

#include <cstdlib>
#include <cstring>
#include <sys/wait.h>
#include <unistd.h>
#include <string>
#include <vector>

namespace testfw {

std::vector<Case> &registry() {
  static std::vector<Case> r;
  return r;
}
int g_failures = 0;
const char *g_xfail = nullptr;

// Exit codes from the child: 0 pass, 1 fail, 2 xfail (failed as expected),
// 3 xpass (an XFAIL test passed: the bug is fixed, drop the XFAIL).
static int child_exit() {
  if (g_xfail)
    return g_failures ? 2 : 3;
  return g_failures ? 1 : 0;
}

void abort_test() {
  std::fflush(stdout);
  std::fflush(stderr);
  _exit(child_exit());
}

void note_failure(const char *file, int line, const std::string &what) {
  ++g_failures;
  std::fprintf(stderr, "  %s:%d: %s\n", file, line, what.c_str());
}

} // namespace testfw

// Tests run as parallel child processes (MZP_TEST_JOBS, default: the CPU
// count); each child's output is buffered in a temporary file and printed in
// one piece when it finishes, so the log stays readable.
int main(int argc, char **argv) {
  using namespace testfw;
  const char *pat = argc > 1 ? argv[1] : nullptr;
  if (pat && !std::strcmp(pat, "--list")) {
    for (auto &c : registry())
      std::printf("%s\n", c.name);
    return 0;
  }
  long jobs = sysconf(_SC_NPROCESSORS_ONLN);
  if (const char *e = std::getenv("MZP_TEST_JOBS"))
    jobs = std::atol(e);
  if (jobs < 1)
    jobs = 1;

  struct Running { pid_t pid; const Case *c; FILE *log; };
  std::vector<Running> running;
  std::vector<const Case *> todo;
  for (auto &c : registry())
    if (!pat || std::strstr(c.name, pat))
      todo.push_back(&c);
  int pass = 0, fail = 0, xfail = 0, xpass = 0, run = (int)todo.size();
  size_t next = 0;
  std::vector<std::string> failed;

  auto reap = [&]() {
    int st = 0;
    pid_t pid = wait(&st);
    for (size_t i = 0; i < running.size(); ++i) {
      if (running[i].pid != pid)
        continue;
      Running r = running[i];
      running.erase(running.begin() + (long)i);
      int code = WIFEXITED(st) ? WEXITSTATUS(st) : 1;
      const char *tag = code == 0 ? "  OK  " : code == 2 ? "XFAIL " : code == 3 ? "XPASS " : " FAIL ";
      std::fflush(r.log);
      std::rewind(r.log);
      char buf[4096];
      size_t n;
      while ((n = std::fread(buf, 1, sizeof buf, r.log)) > 0)
        std::fwrite(buf, 1, n, stdout);
      std::fclose(r.log);
      if (WIFSIGNALED(st))
        std::printf("  killed by signal %d\n", WTERMSIG(st));
      std::printf("[%s] %s\n", tag, r.c->name);
      std::fflush(stdout);
      if (code == 0) ++pass;
      else if (code == 2) ++xfail;
      else if (code == 3) { ++xpass; failed.push_back(std::string(r.c->name) + " (XPASS)"); }
      else { ++fail; failed.push_back(r.c->name); }
      return;
    }
  };

  while (next < todo.size() || !running.empty()) {
    while (next < todo.size() && (long)running.size() < jobs) {
      const Case *c = todo[next++];
      FILE *log = std::tmpfile();
      std::fflush(stdout);
      pid_t pid = fork();
      if (pid == 0) {
        dup2(fileno(log), 1);
        dup2(fileno(log), 2);
        alarm(600); // wall-clock backstop; virtual-time limits catch the rest
        c->fn();
        std::fflush(stdout);
        std::fflush(stderr);
        _exit(child_exit());
      }
      running.push_back({pid, c, log});
    }
    reap();
  }
  std::printf("\n%d tests: %d passed, %d failed, %d expected failures, %d "
              "unexpected passes\n",
              run, pass, fail, xfail, xpass);
  for (auto &f : failed)
    std::printf("  FAILED: %s\n", f.c_str());
  if (run == 0)
    return 1;
  return (fail || xpass) ? 1 : 0;
}
