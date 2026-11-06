# GitHub publication procedure

## 1. Repository strategy

The original Git repository is retained privately as the complete development history.

A separate public-release repository will be prepared from the verified release files, with a new initial commit. Do not rewrite or delete the original repository history.

The Git working copy, not an older OneDrive or ZIP archive, is the authoritative source for the release.

## 2. Licensing and attribution

Complete `docs/RELEASE_CHECKLIST.md` and `docs/THIRD_PARTY_NOTICES.md` before publication.

Verify the provenance and redistribution conditions of the UFACTORY URDF, visual STL meshes, collision OBJ meshes, and adapted MuJoCo model files.

Preserve the required third-party license texts and copyright notices.

## 3. Release content and privacy

Include only the verified source code, robot models, required mesh files, documentation, selected experiment results, and license files.

Exclude private research archives, temporary results, credentials, local environment files, caches, and build manifests.

Check the complete release snapshot for local paths, confidential information, outdated instructions, and unnecessary metadata.

## 4. Reproducibility

Verify the environment setup using `environment.yml`.

Run unit tests, MuJoCo–Pinocchio model verification, the baseline PD+G experiment, and independent Cartesian 3D and 6D planning examples.

The independent fresh-clone tests performed on the development computer have passed. Testing on a second computer remains a separate release check.

## 5. Publication workflow

1. Complete the technical, licensing, provenance, and privacy reviews.
2. Prepare a clean release directory without the original `.git` history.
3. Initialize a new Git repository containing only approved release files.
4. Create a private GitHub repository and push the clean release candidate.
5. Clone and test the private repository, preferably on a second computer.
6. Review the final contents and Git history.
7. Make the repository public only after all mandatory checks pass.
8. Create a version tag after the release is approved.

## 6. Scope and limitations

The repository demonstrates simulation-based xArm7 joint control, model verification, and independent Cartesian 3D/6D inverse-kinematics planning.

Successful simulation and numerical validation do not establish readiness for operation on a physical robot. Collision avoidance, hardware safety, and real-robot commissioning require separate validation.
