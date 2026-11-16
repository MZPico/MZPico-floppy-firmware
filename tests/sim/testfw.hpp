// Minimal test framework. Each TEST runs in a forked child process, because the
// firmware under test keeps its state in file-scope statics: a fresh process
// is the only honest "power-up" between tests.
//
//   test_binary            run every test
//   test_binary PATTERN    run tests whose name contains PATTERN
//   test_binary --list     list test names
//
// Inside a test: CHECK(cond) records a failure and continues; REQUIRE(cond)
// stops the test. CHECK_EQ(a, b) prints both values. XFAIL("issue #N") marks a
// test whose failure is a known, tracked bug: it passes the suite while it
// fails, and FAILS the suite once it starts passing (so the issue gets closed).
#pragma once
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace testfw {

struct Case {
  const char *name;
  void (*fn)();
};
std::vector<Case> &registry();
struct Reg {
  Reg(const char *n, void (*f)()) { registry().push_back({n, f}); }
};

extern int g_failures;
extern const char *g_xfail;
[[noreturn]] void abort_test();
void note_failure(const char *file, int line, const std::string &what);

} // namespace testfw

#define TEST(name)                                                             \
  static void test_##name();                                                   \
  static testfw::Reg reg_##name(#name, test_##name);                           \
  static void test_##name()

#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c))                                                                  \
      testfw::note_failure(__FILE__, __LINE__, "CHECK(" #c ")");               \
  } while (0)

#define REQUIRE(c)                                                             \
  do {                                                                         \
    if (!(c)) {                                                                \
      testfw::note_failure(__FILE__, __LINE__, "REQUIRE(" #c ")");             \
      testfw::abort_test();                                                    \
    }                                                                          \
  } while (0)

#define CHECK_EQ(a, b)                                                         \
  do {                                                                         \
    auto va_ = (a);                                                            \
    auto vb_ = (b);                                                            \
    if (!(va_ == vb_))                                                         \
      testfw::note_failure(__FILE__, __LINE__,                                 \
                           "CHECK_EQ(" #a ", " #b "): " +                      \
                               std::to_string((long long)va_) + " != " +       \
                               std::to_string((long long)vb_));                \
  } while (0)

#define XFAIL(why) (testfw::g_xfail = (why))
