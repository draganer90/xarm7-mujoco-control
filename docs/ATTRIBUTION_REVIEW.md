# Third-party attribution and public-release review

**Status (2026-11-05): visual and collision mesh provenance verified by SHA-256; URDF source revision and final release checks pending.**

This file records *verification evidence* separately from *licensing and release decisions*. Byte-identical files do not, by themselves, establish the historic download location or override third-party licence conditions.

## Verified visual meshes (STL)

On 2026-10-27, the maintainer compared local files in `meshes/xarm7/visual/` with the official MuJoCo Menagerie `ufactory_xarm7/assets` files retrieved from `main`:

- Source: https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_xarm7/assets
- Result: **8/8 SHA-256 matches**, `link_base.stl`, `link1.stl` through `link7.stl`.
- Source project/model licence: BSD-3-Clause; model notice: Copyright (c) 2018, UFACTORY Inc.
- Licence copy retained at `third_party/ufactory_xarm7/LICENSE`.
- A Menagerie revision recorded during preparation was `f3b6013629c7b94f8ee93fd977b161fa8350f9bc`; the SHA-256 comparison above was made against the then-current `main` branch, not yet explicitly against this pinned commit. Verify the eight hashes at the pinned commit before calling the test pinned/reproducible.

## Verified collision meshes (OBJ)

On 2026-11-05, the maintainer compared all eight local files in `meshes/xarm7/collision/` against files fetched from the UFACTORY `xarm_ros` `master` branch:

- Source directory: https://github.com/xArm-Developer/xarm_ros/tree/master/xarm_description/meshes/xarm7_1305/collision
- Result: **8/8 SHA-256 matches**, `link_base.obj`, `link1.obj` through `link7.obj`.
- The upstream project is identified as BSD-3-Clause. Before release, verify whether the specific asset subtree contains any additional notices and record a fixed `xarm_ros` commit SHA; the comparison currently references moving `master`.

The actively simulated `models/xarm7_actuated.xml` and `models/xarm7_base_mjcf.xml` were previously inspected and reported to use local **visual STL files** for visible and collision geometry. The URDF files reference the OBJ collision meshes. Preserve this distinction when describing dependencies.

## URDF source and local adaptation

Files:

- `models/xarm7.urdf` — source-style ROS URDF, robot name `UF_ROBOT`; comments indicate generation by `xacro` from `xarm_device.urdf.xacro`.
- `models/xarm7_mujoco.urdf` — local MuJoCo-oriented adaptation.

A line-by-line local diff on 2026-11-05 confirmed that both descriptions contain 10 links and 9 joints (7 revolute and 2 fixed: `world_joint`, `joint_eef`). The following changes were observed:

1. ROS `package://xarm_description/meshes/xarm7/...` mesh references were replaced with local `../meshes/xarm7/...` paths.
2. A MuJoCo-specific compiler element was inserted: `<mujoco><compiler discardvisual="false" balanceinertia="true" /></mujoco>`.
3. ROS transmission, Gazebo configuration/plugin blocks and generator comments were removed.
4. XML serialization/whitespace was reformatted.

**No textual changes were observed in the corresponding mass, inertia, joint-origin, axis, limit, damping or friction attribute values across the two local URDF files.** This does not prove that all effective inertial properties of the compiled MuJoCo model are unchanged; `balanceinertia="true"` can affect compilation. The exact upstream URDF/xacro revision from which `models/xarm7.urdf` was generated is **not yet established**, nor has exact identity with a published upstream URDF been demonstrated.

Candidate source projects:

- https://github.com/xArm-Developer/xarm_ros
- https://github.com/xArm-Developer/xarm_ros2

## Other MuJoCo model and controller modifications

Project-specific MJCF and Python code include seven direct torque actuators, simulation-side damping/friction and actuator limits, and the use of the `link7` body-frame origin as the reported TCP. These project changes should not be described as original UFACTORY robot geometry.

## Licence separation

- **Original Python controller/planning code:** project-root `LICENSE` (BSD-3-Clause), Copyright (c) 2026, Dragan Erceg, subject to the author's authority to license the work.
- **UFACTORY source meshes and model descriptions:** retained upstream copyright/licence obligations; project `LICENSE` does not replace them.
- **Documentation, graphs and other material:** confirm ownership and any additional third-party rights before assigning a blanket licence. Do not imply all contents are covered by the project-code licence.

See also `docs/THIRD_PARTY_NOTICES.md` and `third_party/ufactory_xarm7/LICENSE`.

## Release checklist

- [x] Check 8/8 visual STL meshes against MuJoCo Menagerie via SHA-256 (retrieved `main`).
- [x] Check 8/8 collision OBJ meshes against UFACTORY `xarm_ros` via SHA-256 (retrieved `master`).
- [x] Inspect local URDF changes and document the adaptation.
- [x] Retain the identified UFACTORY model BSD-3-Clause text and notice.
- [x] Add a separate BSD-3-Clause licence for original Python code (`LICENSE`, Dragan Erceg, 2026).
- [x] Pass the project's existing 12 unit tests (2026-11-05).
- [x] Pass MuJoCo vs Pinocchio mass/bias comparison at two reference states (2026-11-05).
- [x] Re-check eight STL hashes against a **pinned** Menagerie commit (rather than moving `main`).
- [x] Record a **pinned** `xarm_ros` commit SHA and re-check the eight OBJ files against it.
- [ ] Identify/reconcile the upstream source revision of `models/xarm7.urdf` and verify applicable source-file notices.
- [ ] Review README paths, distribution scope and third-party notices, including any additional non-code assets.
- [x] Confirm the repo can be installed and run from a fresh clone/environment (Windows 11, Python 3.11.18, MuJoCo 3.17.0; 12/12 tests PASS, dynamics verification PASS, PD+G simulation reproduced).
- [ ] Review the proposed public contents before setting GitHub repository visibility to public.

Until unresolved provenance and redistribution questions are addressed, keep any remote repository **private**. No public GitHub publication is implied by this checklist.

## Pinned upstream verification (2026-11-05)

All 16 local geometry files were compared using SHA-256 against files from immutable upstream Git revisions.

- Visual STL (8/8 MATCH): google-deepmind/mujoco_menagerie, commit 98a70c569c4c1b6da467a20012f766ee1f8dc521, path ufactory_xarm7/assets/.
- Collision OBJ (8/8 MATCH): xArm-Developer/xarm_ros, commit cf51617d98bc55a88d9aa735acfa43a0bbb8e425, path xarm_description/meshes/xarm7_1305/collision/.

These checks establish byte-for-byte correspondence with the specified revisions. The original provenance of the locally generated URDF and its exact upstream source revision remain to be established.

## URDF parameter verification (2026-11-06)

Reference: xArm-Developer/xarm_ros, commit cf51617d98bc55a88d9aa735acfa43a0bbb8e425.

- Inertial parameters: 70/70 PASS against config/link_inertial/xarm7_type7_HT_BR2.yaml (tolerance 1e-8).
- Joint origins (XYZ/RPY): 42/42 PASS against config/kinematics/default/xarm7_default_kinematics.yaml (tolerance 1e-8).
- Joint axes: 7/7 match xarm7.urdf.xacro (0 0 1).
- Joint velocity limits: 7/7 match (3.14 rad/s).
- Joint position limits: 14/14 match xarm7.urdf.xacro defaults, allowing numerical rounding of pi.
- Joint effort limits differ: local URDF [50,50,30,30,30,20,20] Nm; current upstream joint_limits.yaml [163.2,163.2,81.6,81.6,81.6,30.6,30.6] Nm. The upstream joint_limits_1305.yaml also differs.
- The exact historical source commit used to generate the local URDF remains unknown.
- No simulation model parameters or experimental results were modified.
