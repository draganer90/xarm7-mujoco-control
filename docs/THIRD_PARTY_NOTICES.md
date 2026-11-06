# Third-party notices and model provenance

## Scope of licensing

The project's original Python controller and trajectory-planning code is distributed under the repository-root [BSD-3-Clause license](../LICENSE), copyright (c) 2026, Dragan Erceg. This license does **not** replace the applicable licenses or attribution requirements for imported robot descriptions, geometry, or derivative model data. The licensing of individual documents and graphics should be reviewed separately where their authorship is not established.

The project uses a simulated, fixed-base UFACTORY xArm7. The source comparisons below identify reproducible reference revisions; a matching hash establishes identical file contents, **not**, by itself, a right to redistribute an asset.

## Visual STL meshes: MuJoCo Menagerie / UFACTORY

Eight files are present under `meshes/xarm7/visual/`: `link_base.stl` and `link1.stl` through `link7.stl`.

- Upstream repository: https://github.com/google-deepmind/mujoco_menagerie
- Pinned reference revision: `98a70c569c4c1b6da467a20012f766ee1f8dc521`
- Reference directory: `ufactory_xarm7/assets/`
- Verification: **8/8 SHA-256 matches** against the files at the revision above.
- Model license as retained in this repository: [`third_party/ufactory_xarm7/LICENSE`](../third_party/ufactory_xarm7/LICENSE); UFACTORY copyright notice, 2018.

The repository's earlier comparison against the moving `main` branch is superseded for reproducibility by the pinned-revision comparison above.

## Collision OBJ meshes: UFACTORY xarm_ros

Eight files are present under `meshes/xarm7/collision/`: `link_base.obj` and `link1.obj` through `link7.obj`.

- Upstream repository: https://github.com/xArm-Developer/xarm_ros
- Pinned reference revision: `cf51617d98bc55a88d9aa735acfa43a0bbb8e425`
- Reference directory: `xarm_description/meshes/xarm7_1305/collision/`
- Verification: **8/8 SHA-256 matches** against the files at the revision above.
- Upstream repository license copied without modification to [`third_party/xarm_ros/LICENSE`](../third_party/xarm_ros/LICENSE), with its UFACTORY copyright notice (2018). The copied file's SHA-256 matches the downloaded pinned-revision license.

The two retained UFACTORY license files have different file hashes and should not be treated as byte-identical; preserve both source-specific notices. The specific notice requirements of all included third-party files must be respected.

## URDF descriptions and MuJoCo adaptation

- `models/xarm7.urdf` identifies itself as a Xacro-generated URDF from `xarm_device.urdf.xacro`. Its ROS mesh paths and model structure are consistent with the UFACTORY `xarm_ros` xArm7 description family. **Its original generation date, exact historical source revision, and byte-for-byte provenance remain unknown.** Do not claim that this local URDF is an unmodified copy from the pinned `xarm_ros` revision.
- `models/xarm7_mujoco.urdf` is a locally adapted representation of `models/xarm7.urdf`. The reviewed adaptation changes ROS `package://` mesh references to local relative paths, removes ROS transmissions and Gazebo-related elements, adds a MuJoCo compiler declaration (`discardvisual="false"`, `balanceinertia="true"`), and changes comments/formatting. In the source URDF representation, seven revolute plus two fixed joints and their listed inertial, geometric, limit, damping, and friction values are retained.
- The MuJoCo `balanceinertia` option may affect compiled inertial quantities where applicable. Matching source values do not prove every compiled quantity is unchanged.

### Numeric checks against a pinned UFACTORY reference

All checks below use `xArm-Developer/xarm_ros` revision `cf51617d98bc55a88d9aa735acfa43a0bbb8e425`:

| Compared quantity | Reference file | Result |
| --- | --- | --- |
| Mass, center of mass, inertia, links 1-7 | `xarm_description/config/link_inertial/xarm7_type7_HT_BR2.yaml` | **70/70 match**, absolute tolerance `1e-8` |
| Joint origins (XYZ/RPY), joints 1-7 | `xarm_description/config/kinematics/default/xarm7_default_kinematics.yaml` | **42/42 match**, absolute tolerance `1e-8` |
| Joint axes | `xarm_description/urdf/xarm7/xarm7.urdf.xacro` | **7/7 match** (`0 0 1`) |
| Joint velocity limits | same Xacro | **7/7 match** (`3.14 rad/s`) |
| Joint position lower/upper limits | same Xacro default arguments | **14/14 match**, allowing rounding of pi |

**Known deviation:** Local URDF joint effort limits are `[50, 50, 30, 30, 30, 20, 20]` Nm, while the pinned `xarm_ros` `xarm_description/config/xarm7/joint_limits.yaml` contains `[163.2, 163.2, 81.6, 81.6, 81.6, 30.6, 30.6]` Nm. Its `joint_limits_1305.yaml` also differs from the local URDF. These are not equivalent parameter sets. No inference is made about which limits apply to a particular physical robot.

## Locally stored MJCF models

The repository includes `models/xarm7_base_mjcf.xml` and `models/xarm7_actuated.xml`. An XML tree comparison established that, after omitting the latter file's `<actuator>` element, the two trees are identical (ignoring XML formatting). Both contain seven bodies, seven joints, sixteen geoms, and eight STL mesh assets. The actuated model additionally contains seven `<motor>` actuators with `gear="1"` and `ctrlrange="-50 50"`; joint-level `actuatorfrcrange` values can be tighter (for example, +/-30 or +/-20 Nm).

In these MJCF files, the STL assets appear in both visual and collidable geoms. The OBJ meshes are shipped and referenced by URDF descriptions, but they are **not** the mesh assets used directly by these MJCF simulation files.

The exact historical generation workflow and original source revision of the base MJCF were **not independently established**. Its inclusion should not be interpreted as a claim of wholly original authorship of robot-model parameters or geometry. The provenance and any embedded third-party rights in these XML derivatives remain subject to release review.

## Reproducibility checks (not licensing checks)

The repository was cloned into a new local directory and tested in a separately created Windows Conda environment from `environment.yml` (Python 3.11.18, MuJoCo 3.17.0). All 12 unit tests passed, Pinocchio/MuJoCo mass and bias agreed at two reference states, and the `pd_g` simulation reproduced the reported summary metrics. This verifies the tested local workflow, not universal cross-platform reproducibility or legal redistribution rights.

## Attribution and citation

UFACTORY model and geometry copyright notices must be retained with their source-specific license texts. The numerical and model-source lineage described above should be credited when reusing the work. For the model collection, see:

K. Zakka, Y. Tassa, and MuJoCo Menagerie Contributors, *MuJoCo Menagerie: A collection of high-quality simulation models for MuJoCo*, 2022. https://github.com/google-deepmind/mujoco_menagerie

## Unresolved public-release review

1. Confirm that the complete license and notices applicable to each redistributed third-party asset (including URDF, STL, OBJ, and derivative MJCF data) are preserved and any additional attribution obligations are met.
2. Record the unknown original URDF generation revision transparently; do not present the pinned comparison revision as its historical source.
3. Review authorship, copyright, privacy-sensitive information and embedded metadata of the technical Word report, CorelDRAW source, graphics, curated datasets, and any other non-code assets.
4. Review the complete proposed public Git tree and repository history for private or extraneous material; verify that removing old installation instructions from the latest tree is sufficient for the intended publication.

**Do not change repository visibility to public until these remaining release checks are satisfied.**
