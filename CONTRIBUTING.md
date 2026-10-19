# Contributing

Thank you for your interest. This repository is published from WIM's internal repository: each release arrives here as
one commit with a tag `vX.Y.Z` (the release version — tags are never moved or deleted), and documentation may be
updated between releases.

## Issues

Issues are welcome — bugs, questions and requests. Please include:

- the platform (`cat /etc/nv_tegra_release`, `uname -r`) and the tag you built;
- what you ran, what you expected and what happened;
- for a drive: the drive model file and the output of `plem drive check --json <file>`;
- logs as text, not screenshots.

The supported platform is the one in [`docs/install.md`](docs/install.md) (Jetson AGX Orin, JetPack 7.2.1, ROS 2 Jazzy). Reports from
other platforms are read, but may not be fixed.

Security problems go to `SECURITY.md`, not to an issue.

## Pull requests

Pull requests are welcome as proposals. They are not merged on GitHub — the next publish replaces this tree — so a
change we take is applied in the internal repository and appears here with the next release (or sooner, for
documentation). Its commit here credits you with a `Co-authored-by:` line, and a code change is also listed in
`CHANGELOG.md`. We close the pull request with a note saying whether it was taken.

To help us take it:

- one change per pull request, against `master` (the latest tag's code, plus any documentation updated since), with
  what it fixes and how you checked it ([`docs/install.md`](docs/install.md), section 6, builds and tests the tree);
- a change to `tools/` or `setup/rt/` should pass their shell tests, which need no root, device or build
  (`tools/test/test_bench_report.sh` needs numpy and matplotlib, and skips without them):

  ```bash
  for t in tools/test/test_*.sh setup/rt/test/test_*.sh; do bash "$t" || exit 1; done
  ```

- code and comments in English, in the style of the file around them;
- C++ formatted and checked on the lines you changed, against your colcon build (`.clang-format`, `.clang-tidy`):

  ```bash
  git diff -U0 origin/master -- '*.cpp' '*.hpp' | clang-format-diff-18 -p1 -i
  git diff -U0 origin/master -- '*.cpp' '*.hpp' | clang-tidy-diff-18.py -p1 -path build -j "$(nproc)"
  ```

  (`-path build` needs the packages' `compile_commands.json`, which they export);
- Python checked with `ruff check` (`ruff.toml`);
- sign off each commit (`git commit -s`), certifying the [Developer Certificate of Origin](https://developercertificate.org/).

Contributions are made under the Apache License 2.0 (`LICENSE`, section 5).
