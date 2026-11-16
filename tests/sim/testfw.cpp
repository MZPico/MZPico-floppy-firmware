#include "testfw.hpp"

#include <cstdlib>
#include <cstring>
#include <sys/wait.h>
#include <unistd.h>

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

int main(int argc, char **argv) {
  using namespace testfw;
  const char *pat = argc > 1 ? argv[1] : nullptr;
  if (pat && !std::strcmp(pat, "--list")) {
    for (auto &c : registry())
      std::printf("%s\n", c.name);
    return 0;
  }
  int pass = 0, fail = 0, xfail = 0, xpass = 0, run = 0;
  for (auto &c : registry()) {
    if (pat && !std::strstr(c.name, pat))
      continue;
    ++run;
    std::printf("[ RUN  ] %s\n", c.name);
    std::fflush(stdout);
    pid_t pid = fork();
    if (pid == 0) {
      alarm(600); // wall-clock backstop; virtual-time limits catch the rest
      c.fn();
      std::fflush(stdout);
      _exit(child_exit());
    }
    int st = 0;
    waitpid(pid, &st, 0);
    int code = WIFEXITED(st) ? WEXITSTATUS(st) : 1;
    if (WIFSIGNALED(st))
      std::fprintf(stderr, "  killed by signal %d\n", WTERMSIG(st));
    const char *tag = code == 0 ? "  OK  " : code == 2 ? "XFAIL " : code == 3 ? "XPASS " : " FAIL ";
    std::printf("[%s] %s\n", tag, c.name);
    if (code == 0)
      ++pass;
    else if (code == 2)
      ++xfail;
    else if (code == 3)
      ++xpass;
    else
      ++fail;
  }
  std::printf("\n%d tests: %d passed, %d failed, %d expected failures, %d "
              "unexpected passes\n",
              run, pass, fail, xfail, xpass);
  if (run == 0)
    return 1;
  return (fail || xpass) ? 1 : 0;
}
