# rv2_control_signal_transport — R1

The `r1` branch contains the R1 transport library, lifecycle manager and
`csm_master_node`. Legacy RV2 code remains on the frozen `master` branch and
in earlier release commits; the old library, keyboard node and legacy tests
are no longer built or installed by this branch.

## Public surface

- Headers: `include/rv2_control_signal_transport/r1/`
- C++ namespace: `rv2_interfaces::r1` (retained for R1 API compatibility;
  it does not require the legacy `rv2_interfaces` ROS package)
- Shared library/export: `r1_control_signal_transport`
- Executable: `ros2 run rv2_control_signal_transport csm_master_node`
- Workspace dependency: `r1_interfaces`; no legacy `rv2_interfaces` dependency

The full design and collaboration rules live in
[rv2_project/docs/r1_design_docs](https://github.com/cocobird231/rv2_project/tree/master/docs/r1_design_docs).
The legacy Doxyfile is preserved unchanged and is not a release version source.

## Build and test

Initialize the pinned framework, then use the package's own Docker lifecycle:

```bash
git submodule update --init --recursive
./r1_test_framework/test_build.sh
./r1_test_framework/test_deps.sh
./r1_test_framework/test_run.sh
./r1_test_framework/test_clean.sh
```

Run lint separately before a PR:

```bash
./r1_test_framework/test_lint.sh
```

All compilation, dependency installation and tests run inside the framework's
official ROS Docker image. Keep the sibling `r1_interfaces` checkout at the
release selected by the project snapshot. ROS test jobs must run serially
across owners unless DDS isolation has been explicitly demonstrated.

R1 functional coverage is unchanged: unit 72 cases and integration 41 cases
in eight gtest targets. The removed 21 cases belonged solely to legacy RV2.
ASan covers 64 cases; UBSan covers all 72 R1 unit cases. TSan remains opt-in
(`-t on`), and SKIP never means race-free.

The currently pinned framework release still expects a legacy UBSan target.
R1-only official acceptance requires the compatible framework PR to be merged,
pulled and pinned first. Do not add a fake target, skip UBSan or alter a
released framework checkout to bypass that requirement. Development override
results must be identified as preliminary.

Logs and per-case XML are preserved under `test_env/<distro>/`.
`test/unit/` tests single contracts; `test/integration/` tests manager,
lifecycle and master collaboration. Debian packaging and clean downstream
export verification use the separate `test_packages.sh` entry.

## Migration and versions

Consumers of legacy `control_signal_transport`, keyboard executables, root
legacy headers or keyboard launch/config files must remain on `master` or an
earlier release until migrated. R1 names and contracts are not renamed by
this cleanup.

The ROS version comes from `package.xml`. Functional changes and test fixes
are committed separately from the PR-ready version-only commit and annotated
`vX.Y.Z` tag. Transport PRs target `r1`; do not merge them into legacy `master`.
