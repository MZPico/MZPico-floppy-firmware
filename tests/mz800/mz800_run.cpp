// Run an MZ-800 program (MZF) against the firmware's FDC on a simulated drive.
//
//   mz800_run PROGRAM.MZF [options]
//     --disk0 SPEC  --disk1 SPEC   media: dd | master | hd | dense | none |
//                                  edsk:PATH (an EXTENDED CPC DSK image)
//     --wp0 / --wp1                write-protect the disk
//     --rev0 US / --rev1 US        spindle period (default 200000)
//     --keys STRING                keys for GETKY, in order
//     --limit SECONDS              virtual-time limit (default 600)
//     --expect TEXT                exit 0 only if TEXT appears on screen
//     --quiet                      do not echo the screen
//
// Exit status: 0 = finished (and --expect matched), 1 = otherwise.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "board.hpp"
#include "machine.hpp"

using namespace sim;

static bool make_media(const std::string &spec, Disk *d) {
  std::string err;
  if (spec == "dd")
    *d = make_disk(Geometry::MZ_DD_DATA);
  else if (spec == "master")
    *d = make_disk(Geometry::MZ_DD_MASTER);
  else if (spec == "hd")
    *d = make_disk(Geometry::HD_DATA);
  else if (spec == "dense")
    *d = make_filled_disk(Geometry::MZ_DD_MASTER, 0xFF);
  else if (spec.rfind("edsk:", 0) == 0) {
    if (!load_edsk(spec.substr(5), d, &err)) {
      std::fprintf(stderr, "mz800_run: %s\n", err.c_str());
      return false;
    }
  } else {
    std::fprintf(stderr, "mz800_run: unknown media '%s'\n", spec.c_str());
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: mz800_run PROGRAM.MZF [--disk0 SPEC] ...\n");
    return 2;
  }
  static Disk disks[2];
  bool have[2] = {false, false};
  std::string keys, expect;
  double limit_s = 600;
  bool quiet = false;
  for (int i = 2; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "mz800_run: %s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--disk0" || a == "--disk1") {
      int u = a[6] - '0';
      std::string s = next();
      if (s != "none") {
        if (!make_media(s, &disks[u]))
          return 2;
        have[u] = true;
      }
    } else if (a == "--wp0" || a == "--wp1")
      disks[a[4] - '0'].write_protect = true;
    else if (a == "--rev0" || a == "--rev1")
      cable().drive[a[5] - '0'].rev_ticks = us(std::strtoull(next().c_str(), 0, 10));
    else if (a == "--keys")
      keys = next();
    else if (a == "--limit")
      limit_s = std::atof(next().c_str());
    else if (a == "--expect")
      expect = next();
    else if (a == "--quiet")
      quiet = true;
    else {
      std::fprintf(stderr, "mz800_run: unknown option %s\n", a.c_str());
      return 2;
    }
  }
  for (int u = 0; u < 2; ++u)
    if (have[u])
      insert(u, &disks[u]);
  board_boot();

  mz800::Machine m;
  m.echo = !quiet;
  std::string err;
  if (!m.load_mzf(argv[1], &err)) {
    std::fprintf(stderr, "mz800_run: %s\n", err.c_str());
    return 2;
  }
  m.keys(keys);
  bool ok = run_host(&m, [&] { return m.finished(); }, ms((uint64_t)(limit_s * 1000)));
  std::fflush(stdout);
  if (!m.error().empty()) {
    std::fprintf(stderr, "mz800_run: %s\n", m.error().c_str());
    ok = false;
  }
  if (!m.finished()) {
    std::fprintf(stderr, "mz800_run: program did not finish (%s)\n",
                 core0_hang_reason() ? core0_hang_reason() : "time limit");
    ok = false;
  }
  std::fprintf(stderr, "mz800_run: %.1f s of MZ-800 time\n", (double)now() / ms(1000));
  if (!expect.empty() && m.screen().find(expect) == std::string::npos) {
    std::fprintf(stderr, "mz800_run: expected text not on screen: %s\n", expect.c_str());
    ok = false;
  }
  return ok ? 0 : 1;
}
