# Patchwork (Classic) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add the original Patchwork algorithm as a parallel library inside this repo so users can A/B compare against Patchwork++ from the same Python module and ROS2 node.

**Architecture:** New CMake target `ground_seg_classic` under `cpp/patchwork/`, exposed via the existing `pypatchworkpp` Python module as a sibling class `pypatchworkpp.patchwork`. ROS2 node uses `std::variant` to dispatch on a launch parameter `algorithm`. Algorithm body is ported from `/Users/fudxo/git/patchwork` with PCL/TBB/ROS dependencies stripped; we reuse Patchwork++'s parametric Concentric Zone Model (`num_zones`, `num_sectors_each_zone`, `num_rings_each_zone`, `min_ranges`) instead of upstream's sensor-string-driven CZM.

**Tech Stack:** C++20, Eigen 3.4 (FetchContent), pybind11, scikit-build-core, CMake ≥ 3.18, ROS2 Humble/Jazzy.

**Reference:** Spec at `docs/superpowers/specs/2026-05-09-add-patchwork-classic-algorithm-design.md`. Upstream source at `/Users/fudxo/git/patchwork/include/patchwork/patchwork.hpp` (lines referenced by `up:NNN`).

**Key deliberate deviations from a literal port:**

1. CZM uses Patchwork++'s parametric form (no `sensor_configs.hpp`, no `zone_models.hpp`).
1. `tbb::parallel_for` over patches becomes a sequential `for` loop.
1. Verbose logging via `std::cout` instead of `boost::format` + `RCLCPP_INFO`.
1. Input is `Eigen::MatrixXf` (N×4), not `pcl::PointCloud<PointXYZI>`.

______________________________________________________________________

## File Structure

| Path | Status | Responsibility |
|---|---|---|
| `cpp/patchwork/CMakeLists.txt` | NEW | Builds `ground_seg_classic` static lib, exports config |
| `cpp/patchwork/include/patchwork/patchwork.h` | NEW | `PatchworkParams` struct + `PatchWork` class declaration |
| `cpp/patchwork/src/patchwork.cpp` | NEW | Algorithm body |
| `cpp/CMakeLists.txt` | MODIFY | `add_subdirectory(patchwork)` |
| `python/CMakeLists.txt` | MODIFY | Link both targets |
| `python/patchworkpp/pybinding.cpp` | MODIFY | Bind `PatchworkParams` and `patchwork` class |
| `python/tests/test_patchwork_smoke.py` | NEW | Smoke test for the new class |
| `ros/src/GroundSegmentationServer.hpp` | MODIFY | `std::variant` member |
| `ros/src/GroundSegmentationServer.cpp` | MODIFY | Dispatch via `std::visit` |
| `ros/launch/patchworkpp.launch.py` | MODIFY | Expose `algorithm` launch arg |
| `python/pyproject.toml` | MODIFY | Version bump 1.0.4 → 1.1.0 |
| `cpp/CMakeLists.txt` | MODIFY | Project version bump |
| `README.md` | MODIFY | "Choosing an algorithm" subsection |

______________________________________________________________________

## Branch

All work happens on `feat/patchwork-classic` (already created from current `master` at 306da05). One PR at the end.

______________________________________________________________________

## Phase A — Skeleton & build wiring

### Task A1: Create empty `cpp/patchwork/` skeleton

**Files:**

- Create: `cpp/patchwork/CMakeLists.txt`

- Create: `cpp/patchwork/include/patchwork/patchwork.h`

- Create: `cpp/patchwork/src/patchwork.cpp`

- \[ \] **Step 1: Write the CMakeLists.txt**

```cmake
project(patchwork_classic_src)

include(GNUInstallDirs)

set(CLASSIC_TARGET ground_seg_classic)

add_library(${CLASSIC_TARGET} STATIC src/patchwork.cpp)
set_target_properties(${CLASSIC_TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${CLASSIC_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(${CLASSIC_TARGET} Eigen3::Eigen)
add_library(${PARENT_PROJECT_NAME}::${CLASSIC_TARGET} ALIAS ${CLASSIC_TARGET})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
)
install(TARGETS ${CLASSIC_TARGET}
  EXPORT ${PARENT_PROJECT_NAME}Config
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
```

- \[ \] **Step 2: Write the minimal header**

```cpp
// cpp/patchwork/include/patchwork/patchwork.h
#ifndef PATCHWORK_CLASSIC_H
#define PATCHWORK_CLASSIC_H

#include <Eigen/Dense>
#include <vector>

namespace patchwork {

struct PatchworkParams {
  bool verbose = false;
};

class PatchWork {
 public:
  PatchWork() = default;
  explicit PatchWork(const PatchworkParams& params);

  void estimateGround(const Eigen::MatrixXf& cloud);

  Eigen::MatrixX3f getGround() const;
  Eigen::MatrixX3f getNonground() const;
  std::vector<int> getGroundIndices() const;
  std::vector<int> getNongroundIndices() const;
  double getTimeTaken() const;
  double getHeight() const;

 private:
  PatchworkParams params_;
  Eigen::MatrixX3f ground_;
  Eigen::MatrixX3f nonground_;
  std::vector<int> ground_idx_;
  std::vector<int> nonground_idx_;
  double time_taken_ = 0.0;
  double sensor_height_ = 1.723;
};

}  // namespace patchwork

#endif
```

- \[ \] **Step 3: Write a stub `.cpp`**

```cpp
// cpp/patchwork/src/patchwork.cpp
#include "patchwork/patchwork.h"

namespace patchwork {

PatchWork::PatchWork(const PatchworkParams& params) : params_(params) {}

void PatchWork::estimateGround(const Eigen::MatrixXf& /*cloud*/) {}

Eigen::MatrixX3f PatchWork::getGround() const { return ground_; }
Eigen::MatrixX3f PatchWork::getNonground() const { return nonground_; }
std::vector<int> PatchWork::getGroundIndices() const { return ground_idx_; }
std::vector<int> PatchWork::getNongroundIndices() const { return nonground_idx_; }
double PatchWork::getTimeTaken() const { return time_taken_; }
double PatchWork::getHeight() const { return sensor_height_; }

}  // namespace patchwork
```

- \[ \] **Step 4: Hook into top-level cpp/CMakeLists.txt**

Modify `cpp/CMakeLists.txt`. After the existing `add_subdirectory(patchworkpp)` line (currently line 48), add:

```cmake
set(TARGET_NAME ground_seg_classic)
add_subdirectory(patchwork)
```

The intermediate `set(TARGET_NAME ...)` is required because `patchwork/CMakeLists.txt` reads `${TARGET_NAME}` (mirrors how `patchworkpp/CMakeLists.txt` reads it). Restore the previous value if any subsequent code needs it; in this codebase there is none.

Wait — actually `patchworkpp/CMakeLists.txt` reads `TARGET_NAME` as `ground_seg_cores` set at line 46 of the parent. Read both files before modifying to confirm the pattern, then either:
(a) inline the target name inside `patchwork/CMakeLists.txt` (simpler — recommended), or
(b) set/unset around the `add_subdirectory` call.

Pick (a). Update Task A1 Step 1's CMakeLists.txt to use a literal `ground_seg_classic` instead of `${TARGET_NAME}`:

```cmake
set(CLASSIC_TARGET ground_seg_classic)
```

(already done in Step 1 above).

- \[ \] **Step 5: Configure and build**

```bash
rm -rf cpp/build
cmake -Bcpp/build cpp/
cmake --build cpp/build -j
```

Expected: builds both `libground_seg_cores.a` and `libground_seg_classic.a` without errors.

- \[ \] **Step 6: Commit**

```bash
git add cpp/patchwork/ cpp/CMakeLists.txt
git commit -m "feat(patchwork): scaffold ground_seg_classic target with empty PatchWork class"
```

______________________________________________________________________

## Phase B — C++ unit verification

### Task B1: Add a C++ smoke test that constructs PatchWork

**Files:**

- Create: `cpp/patchwork/tests/smoke_test.cpp`
- Modify: `cpp/patchwork/CMakeLists.txt`

The cpp side has no test framework currently. We add a minimal `main()` test compiled only when `BUILD_PATCHWORK_TESTS` is ON. This serves as a compile-time assurance plus a runtime assertion.

- \[ \] **Step 1: Add CMake test option**

Append to `cpp/patchwork/CMakeLists.txt`:

```cmake
option(BUILD_PATCHWORK_TESTS "Build C++ smoke test for PatchWork" OFF)
if (BUILD_PATCHWORK_TESTS)
  add_executable(patchwork_smoke tests/smoke_test.cpp)
  target_link_libraries(patchwork_smoke PRIVATE ${CLASSIC_TARGET})
endif()
```

- \[ \] **Step 2: Write the smoke test**

```cpp
// cpp/patchwork/tests/smoke_test.cpp
#include <Eigen/Dense>
#include <cassert>
#include <iostream>

#include "patchwork/patchwork.h"

int main() {
  patchwork::PatchworkParams params;
  patchwork::PatchWork pw(params);

  Eigen::MatrixXf cloud(0, 4);
  pw.estimateGround(cloud);

  assert(pw.getGround().rows() == 0);
  assert(pw.getNonground().rows() == 0);
  std::cout << "patchwork smoke: ok" << std::endl;
  return 0;
}
```

- \[ \] **Step 3: Build with tests on, run**

```bash
cmake -Bcpp/build cpp/ -DBUILD_PATCHWORK_TESTS=ON
cmake --build cpp/build -j
./cpp/build/patchwork/patchwork_smoke
```

Expected: `patchwork smoke: ok`.

- \[ \] **Step 4: Commit**

```bash
git add cpp/patchwork/CMakeLists.txt cpp/patchwork/tests/smoke_test.cpp
git commit -m "test(patchwork): add C++ smoke test for empty input handling"
```

______________________________________________________________________

## Phase C — Algorithm port

The algorithm body comes from `/Users/fudxo/git/patchwork/include/patchwork/patchwork.hpp`. Each task ports one logical group of functions and verifies the build still succeeds. The runtime smoke test (`patchwork_smoke`) keeps passing because we feed empty input until Phase D.

When this plan says "port lines X–Y from upstream", the engineer should:

1. Open `/Users/fudxo/git/patchwork/include/patchwork/patchwork.hpp` at those lines
1. Translate the body, replacing the patterns in the table below
1. Compile after each function

### Translation cheat sheet

| Upstream | Replacement |
|---|---|
| `pcl::PointCloud<PointT>` | `std::vector<patchwork::PointXYZ>` |
| `pcl::PointCloud<PointT>::Ptr` | `std::shared_ptr<std::vector<patchwork::PointXYZ>>` (rare in core algo) |
| `pcl::compute3DCentroid(cloud, centroid)` | `Eigen::Vector3f centroid = mean_xyz(points);` (helper we add) |
| `pcl::getMinMax3D(cloud, min, max)` | manual loop over xyz |
| `tbb::parallel_for(blocked_range<int>(0, N), [&](auto r) { for (int i = r.begin(); i != r.end(); ++i) ... })` | `for (int i = 0; i < N; ++i) ...` |
| `boost::format("...") % a % b` | `std::cout << "..." << a << " " << b << ...` |
| `RCLCPP_INFO(node->get_logger(), fmt, args)` | `if (verbose_) std::cout << fmt << ...` |
| `cloud.points[i]` | `cloud[i]` |
| `cloud.size()` | `cloud.size()` (same) |
| `cloud.push_back(p)` | `cloud.push_back(p)` (same) |

### Reuse `patchwork::PointXYZ`

Already declared in `cpp/patchworkpp/include/patchwork/patchworkpp.h:22-29`. Do **not** duplicate. In `cpp/patchwork/include/patchwork/patchwork.h`, add a forward-declared shared header or `#include <patchwork/patchworkpp.h>` directly. Recommended: include patchworkpp.h's POD struct only. Since the patchworkpp header is a single file, include it; the linker will dedupe.

Add at the top of `cpp/patchwork/include/patchwork/patchwork.h`:

```cpp
#include "patchwork/patchworkpp.h"  // for patchwork::PointXYZ
```

This adds Patchwork++ as a public-include compile dependency of the classic target. Update `cpp/patchwork/CMakeLists.txt` to declare it:

```cmake
target_link_libraries(${CLASSIC_TARGET} Eigen3::Eigen ground_seg_cores)
```

(yes, the classic target gains a transitive header dep on the cores target. This is the smallest-scope way to share the POD.)

### Task C1: Port full PatchworkParams + private state members

**Files:**

- Modify: `cpp/patchwork/include/patchwork/patchwork.h`

- \[ \] **Step 1: Replace the minimal PatchworkParams with the full struct**

Source: `up:119-148` (declare_parameter calls show every field with type and default). Translate to:

```cpp
struct PatchworkParams {
  // Sensor / range
  double sensor_height = 1.723;
  double max_range     = 80.0;
  double min_range     = 2.7;

  // Concentric Zone Model (parametric, mirrors patchwork++ style)
  int                  num_zones             = 4;
  std::vector<int>     num_sectors_each_zone = {16, 32, 54, 32};
  std::vector<int>     num_rings_each_zone   = {2, 4, 4, 4};
  std::vector<double>  min_ranges            = {2.7, 12.3625, 22.025, 41.35};

  // Plane fit
  int    num_iter    = 3;
  int    num_lpr     = 20;
  int    num_min_pts = 10;
  double th_seeds    = 0.5;
  double th_dist     = 0.125;

  // Ground likelihood thresholds (fixed)
  double              uprightness_thr = 0.5;
  std::vector<double> elevation_thr   = {0.523, 0.746, 0.879, 1.125};
  std::vector<double> flatness_thr    = {0.0005, 0.000725, 0.001, 0.001};

  // Adaptive seed selection margin for highly tilted ground
  double adaptive_seed_selection_margin = -1.1;

  // Global elevation guard
  bool   using_global_thr      = true;
  double global_elevation_thr  = 0.0;

  // ATAT (default ON per spec)
  bool   ATAT_ON              = true;
  double max_h_for_ATAT       = 0.3;
  int    num_sectors_for_ATAT = 20;
  double noise_bound          = 0.2;

  bool verbose = false;
};
```

- \[ \] **Step 2: Add private state members to PatchWork**

Mirror upstream `up:196-248` selectively. Drop ROS-only fields. Add:

```cpp
 private:
  PatchworkParams params_;

  // Materialized outputs
  std::vector<PointXYZ> ground_pts_;
  std::vector<PointXYZ> nonground_pts_;

  // Cached results
  Eigen::MatrixX3f ground_mat_;
  Eigen::MatrixX3f nonground_mat_;
  std::vector<int> ground_idx_;
  std::vector<int> nonground_idx_;
  bool             outputs_dirty_ = true;

  double time_taken_ = 0.0;
  double sensor_height_ = 1.723;  // updated by ATAT

  // Per-iteration scratch (cleared each estimateGround call)
  using Patch = std::vector<PointXYZ>;
  using Ring  = std::vector<Patch>;
  using RegionwisePatches = std::vector<Ring>;
  RegionwisePatches regionwise_patches_;

  // Helper functions (impl in .cpp)
  void   initialize();
  void   flush();
  double xy2theta(double x, double y) const;
  double xy2radius(double x, double y) const;
  void   pc2regionwise_patches(const std::vector<PointXYZ>& src);
  // ... more added in subsequent tasks
```

- \[ \] **Step 3: Build, run smoke**

```bash
cmake --build cpp/build -j && ./cpp/build/patchwork/patchwork_smoke
```

Expected: PASS.

- \[ \] **Step 4: Commit**

```bash
git add cpp/patchwork/include/patchwork/patchwork.h
git commit -m "feat(patchwork): port full PatchworkParams and private state"
```

### Task C2: Port `initialize()`, `flush()`, polar coord helpers

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`

Source: `up:282-323` for initialize/flush; `up:683-700` for xy2theta/xy2radius.

- \[ \] **Step 1: Implement helpers**

```cpp
double PatchWork::xy2theta(double x, double y) const {
  double a = std::atan2(y, x);
  return (a >= 0) ? a : (a + 2 * M_PI);
}

double PatchWork::xy2radius(double x, double y) const {
  return std::hypot(x, y);
}
```

- \[ \] **Step 2: Implement `initialize()`**

```cpp
void PatchWork::initialize() {
  regionwise_patches_.clear();
  regionwise_patches_.resize(params_.num_zones);
  for (int z = 0; z < params_.num_zones; ++z) {
    regionwise_patches_[z].resize(params_.num_rings_each_zone[z]);
    for (int r = 0; r < params_.num_rings_each_zone[z]; ++r) {
      regionwise_patches_[z][r].resize(params_.num_sectors_each_zone[z]);
    }
  }
}
```

Wait — original CZM uses `Ring = vector<Patch>` per zone (rings × sectors flat). Patchwork++ flattens by `num_rings_each_zone[z] × num_sectors_each_zone[z]` per zone. We follow patchwork++'s shape for consistency. Patches stored as `regionwise_patches_[zone][ring*num_sectors + sector]` is one option; another is `[zone][ring][sector]`. Pick the 3D form for readability:

```cpp
using Sector = std::vector<PointXYZ>;
using Ring   = std::vector<Sector>;     // sectors within one ring
using Zone   = std::vector<Ring>;        // rings within one zone
using RegionwisePatches = std::vector<Zone>;
```

(Update Task C1's typedef block accordingly.)

- \[ \] **Step 3: Implement `flush()`**

```cpp
void PatchWork::flush() {
  for (auto& zone : regionwise_patches_)
    for (auto& ring : zone)
      for (auto& sector : ring)
        sector.clear();
}
```

- \[ \] **Step 4: Build**

```bash
cmake --build cpp/build -j
```

Expected: success.

- \[ \] **Step 5: Commit**

```bash
git add cpp/patchwork/src/patchwork.cpp cpp/patchwork/include/patchwork/patchwork.h
git commit -m "feat(patchwork): port CZM init, flush, and polar coord helpers"
```

### Task C3: Port `pc2regionwise_patches`

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`

Source: `up:702-712`. The upstream uses sensor-config-derived ring assignment (`zone_model_.ring_idx(...)`). We rewrite to use parametric ranges.

- \[ \] **Step 1: Implement**

```cpp
void PatchWork::pc2regionwise_patches(const std::vector<PointXYZ>& src) {
  for (int idx = 0; idx < static_cast<int>(src.size()); ++idx) {
    const auto& p = src[idx];
    double r     = xy2radius(p.x, p.y);
    double theta = xy2theta(p.x, p.y);

    if (r < params_.min_range || r > params_.max_range) continue;

    // Determine zone by min_ranges
    int zone = 0;
    for (int z = params_.num_zones - 1; z >= 0; --z) {
      if (r >= params_.min_ranges[z]) { zone = z; break; }
    }
    if (zone < 0 || zone >= params_.num_zones) continue;

    // Within zone, ring index proportional to (r - min_ranges[z]) / ring_width
    double ring_width = (zone + 1 < params_.num_zones)
                        ? (params_.min_ranges[zone + 1] - params_.min_ranges[zone])
                              / params_.num_rings_each_zone[zone]
                        : (params_.max_range - params_.min_ranges[zone])
                              / params_.num_rings_each_zone[zone];
    int ring = std::min<int>(
        static_cast<int>((r - params_.min_ranges[zone]) / ring_width),
        params_.num_rings_each_zone[zone] - 1);

    int sector = std::min<int>(
        static_cast<int>(theta / (2 * M_PI / params_.num_sectors_each_zone[zone])),
        params_.num_sectors_each_zone[zone] - 1);

    regionwise_patches_[zone][ring][sector].push_back(p);
  }
}
```

- \[ \] **Step 2: Build**

```bash
cmake --build cpp/build -j
```

- \[ \] **Step 3: Commit**

```bash
git add cpp/patchwork/src/patchwork.cpp
git commit -m "feat(patchwork): port pc2regionwise_patches with parametric CZM"
```

### Task C4: Port `estimate_plane_` (PCA fit)

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`, `cpp/patchwork/include/patchwork/patchwork.h`

Source: `up:325-351`. PCA on a patch's seed points. Return PCAFeature struct.

- \[ \] **Step 1: Add PCAFeature struct to header**

In `cpp/patchwork/include/patchwork/patchwork.h`, before `class PatchWork`, add:

```cpp
struct PCAFeature {
  Eigen::Vector3f normal_;
  Eigen::Vector3f mean_;
  Eigen::Vector3f singular_values_;
  float d_;
  float th_dist_d_;
  float linearity_;
  float planarity_;
};
```

And declare `void estimate_plane(const std::vector<PointXYZ>& seeds, PCAFeature& out);` as a private method.

- \[ \] **Step 2: Implement**

Translate `up:325-351`. Pseudocode:

```cpp
void PatchWork::estimate_plane(const std::vector<PointXYZ>& seeds, PCAFeature& out) {
  Eigen::MatrixXf pts(seeds.size(), 3);
  for (size_t i = 0; i < seeds.size(); ++i) {
    pts(i, 0) = seeds[i].x;
    pts(i, 1) = seeds[i].y;
    pts(i, 2) = seeds[i].z;
  }
  Eigen::Vector3f mean = pts.colwise().mean();
  Eigen::MatrixXf centered = pts.rowwise() - mean.transpose();
  Eigen::Matrix3f cov = (centered.adjoint() * centered) / float(pts.rows() - 1);
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU);
  out.normal_ = svd.matrixU().col(2);
  if (out.normal_(2) < 0) out.normal_ = -out.normal_;
  out.singular_values_ = svd.singularValues();
  out.mean_  = mean;
  out.d_     = -out.normal_.dot(mean);
  out.th_dist_d_  = params_.th_dist - out.d_;
  out.linearity_  = (out.singular_values_(0) - out.singular_values_(1)) / out.singular_values_(0);
  out.planarity_  = (out.singular_values_(1) - out.singular_values_(2)) / out.singular_values_(0);
}
```

- \[ \] **Step 3: Build, commit**

```bash
cmake --build cpp/build -j
git add cpp/patchwork/
git commit -m "feat(patchwork): port estimate_plane PCA fit"
```

### Task C5: Port `extract_initial_seeds_`

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`, `cpp/patchwork/include/patchwork/patchwork.h`

Source: `up:353-393`. LPR-based seed selection.

- \[ \] **Step 1: Declare and implement**

Header (private):

```cpp
void extract_initial_seeds(int zone_idx,
                           const std::vector<PointXYZ>& sorted,
                           std::vector<PointXYZ>& seeds);
```

Body (translate `up:353-393`):

```cpp
void PatchWork::extract_initial_seeds(int zone_idx,
                                      const std::vector<PointXYZ>& sorted,
                                      std::vector<PointXYZ>& seeds) {
  seeds.clear();
  double sum = 0.0;
  int    cnt = 0;

  // Patchwork uses adaptive_seed_selection_margin in the first zone (innermost)
  // to skip points that are likely from the sensor body / car roof.
  int init_idx = 0;
  if (zone_idx == 0) {
    for (int i = 0; i < static_cast<int>(sorted.size()); ++i) {
      if (sorted[i].z < params_.adaptive_seed_selection_margin * params_.sensor_height) {
        ++init_idx;
      } else {
        break;
      }
    }
  }

  for (int i = init_idx; i < static_cast<int>(sorted.size()) && cnt < params_.num_lpr; ++i) {
    sum += sorted[i].z;
    ++cnt;
  }
  double lpr_height = (cnt != 0) ? (sum / cnt) : 0.0;

  for (const auto& p : sorted) {
    if (p.z < lpr_height + params_.th_seeds) seeds.push_back(p);
  }
}
```

- \[ \] **Step 2: Build, commit**

```bash
cmake --build cpp/build -j
git add cpp/patchwork/
git commit -m "feat(patchwork): port extract_initial_seeds with adaptive margin"
```

### Task C6: Port `determine_ground_likelihood_estimation_status` (GLE check)

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`, `cpp/patchwork/include/patchwork/patchwork.h`

Source: `up:809-836`. Classifies a patch as ground/nonground using uprightness + elevation + flatness.

- \[ \] **Step 1: Add status enum to header**

```cpp
enum class PatchStatus {
  NotAssigned = -2,
  FewPoints   = -1,
  UprightEnough = 0,
  FlatEnough = 1,
  TooHighElevation = 2,
  TooTilted = 3,
  GloballyTooHighElevation = 4,
};
```

- \[ \] **Step 2: Declare and implement**

Header (private):

```cpp
PatchStatus determine_gle_status(int zone_idx, int ring_idx,
                                 const PCAFeature& feature) const;
```

Body (translate `up:810-836`, replacing magic-number returns with the enum):

```cpp
PatchStatus PatchWork::determine_gle_status(int zone_idx, int ring_idx,
                                            const PCAFeature& feature) const {
  // Uprightness check (normal_ z-component)
  if (std::abs(feature.normal_(2)) < params_.uprightness_thr) return PatchStatus::TooTilted;

  // The first num_rings_of_interest rings get tier-specific elevation/flatness thresholds.
  int tier = (zone_idx == 0) ? ring_idx : zone_idx;
  if (tier < static_cast<int>(params_.elevation_thr.size())) {
    double mean_z = feature.mean_(2);
    if (mean_z > params_.elevation_thr[tier]) {
      // Recoverable if patch is very flat
      if (feature.singular_values_(2) < params_.flatness_thr[tier])
        return PatchStatus::FlatEnough;
      return PatchStatus::TooHighElevation;
    }
    return PatchStatus::UprightEnough;
  }

  // Beyond tier coverage: optional global elevation guard
  if (params_.using_global_thr && feature.mean_(2) > params_.global_elevation_thr) {
    return PatchStatus::GloballyTooHighElevation;
  }
  return PatchStatus::UprightEnough;
}
```

- \[ \] **Step 3: Build, commit**

```bash
cmake --build cpp/build -j
git add cpp/patchwork/
git commit -m "feat(patchwork): port GLE status classifier"
```

### Task C7: Port `perform_regionwise_ground_segmentation` (per-patch loop)

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`, `cpp/patchwork/include/patchwork/patchwork.h`

Source: `up:715-806`. Per-patch: sort by z, extract seeds, fit plane, refit num_iter times, classify.

- \[ \] **Step 1: Declare and implement**

Header (private):

```cpp
void perform_regionwise_segmentation(int zone_idx, int ring_idx,
                                     const std::vector<PointXYZ>& patch,
                                     std::vector<PointXYZ>& patch_ground,
                                     std::vector<PointXYZ>& patch_nonground,
                                     PatchStatus& status_out);
```

Body — translate the upstream function structure:

1. Skip if `patch.size() < num_min_pts`. Set `status_out = FewPoints` and put the entire patch into `patch_nonground`.
1. Sort patch by z.
1. Call `extract_initial_seeds`.
1. Loop `num_iter` times: estimate plane, partition points by `(p - mean) · normal < th_dist_d_` into ground/nonground.
1. After final iteration, call `determine_gle_status`.

Skip the upstream's `pcl::PointCloud` membership games; we're operating on `std::vector<PointXYZ>`.

- \[ \] **Step 2: Build, commit**

```bash
cmake --build cpp/build -j
git add cpp/patchwork/
git commit -m "feat(patchwork): port per-patch ground segmentation loop"
```

### Task C8: Port `estimate_ground` (main entry)

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`, `cpp/patchwork/include/patchwork/patchwork.h`

Source: `up:528-680`. Main entry that ties everything together.

- \[ \] **Step 1: Implement `estimateGround`**

The upstream signature is `estimate_ground(in, ground, nonground)`. Ours is `estimateGround(const Eigen::MatrixXf&)` and getters. Rewrite so that `estimateGround` populates `ground_pts_` / `nonground_pts_` / index vectors and starts a timer.

```cpp
void PatchWork::estimateGround(const Eigen::MatrixXf& cloud) {
  using clock = std::chrono::high_resolution_clock;
  auto t_start = clock::now();

  // Initialize CZM on first call
  if (regionwise_patches_.empty()) initialize();

  // 1) Convert input
  std::vector<PointXYZ> all_points;
  all_points.reserve(cloud.rows());
  for (int i = 0; i < cloud.rows(); ++i) {
    all_points.emplace_back(cloud(i, 0), cloud(i, 1), cloud(i, 2), i);
  }

  // 2) Quick pre-filter: drop points more than 2.0 m below sensor height
  //    (this is the noise filter the maintainer was interested in)
  std::vector<PointXYZ> kept;
  kept.reserve(all_points.size());
  for (const auto& p : all_points) {
    if (p.z >= -sensor_height_ - 2.0) kept.push_back(p);
  }

  // 3) ATAT (optional)
  if (params_.ATAT_ON) estimate_sensor_height(kept);

  // 4) Reset patch buckets, redistribute
  flush();
  pc2regionwise_patches(kept);

  // 5) Per-patch segmentation (sequential — was tbb::parallel_for upstream)
  ground_pts_.clear();
  nonground_pts_.clear();
  for (int z = 0; z < params_.num_zones; ++z) {
    for (int r = 0; r < params_.num_rings_each_zone[z]; ++r) {
      for (int s = 0; s < params_.num_sectors_each_zone[z]; ++s) {
        const auto& patch = regionwise_patches_[z][r][s];
        std::vector<PointXYZ> pg, png;
        PatchStatus status;
        perform_regionwise_segmentation(z, r, patch, pg, png, status);
        // Aggregate per-patch outputs
        switch (status) {
          case PatchStatus::UprightEnough:
          case PatchStatus::FlatEnough:
            ground_pts_.insert(ground_pts_.end(), pg.begin(), pg.end());
            nonground_pts_.insert(nonground_pts_.end(), png.begin(), png.end());
            break;
          default:
            // Reject the whole patch as nonground
            nonground_pts_.insert(nonground_pts_.end(), patch.begin(), patch.end());
        }
      }
    }
  }

  // 6) Materialize outputs (lazy via outputs_dirty_; here we just mark dirty)
  outputs_dirty_ = true;

  auto t_end = clock::now();
  time_taken_ = std::chrono::duration<double, std::micro>(t_end - t_start).count();
}
```

- \[ \] **Step 2: Implement getters with lazy materialization**

In the header, mark the cached output members `mutable`, plus `mutable bool outputs_dirty_`. Then the materialize helper can be `const`. Updated header excerpt:

```cpp
 private:
  // ... params_, ground_pts_, nonground_pts_, time_taken_, sensor_height_, regionwise_patches_ ...

  mutable Eigen::MatrixX3f ground_mat_;
  mutable Eigen::MatrixX3f nonground_mat_;
  mutable std::vector<int> ground_idx_;
  mutable std::vector<int> nonground_idx_;
  mutable bool             outputs_dirty_ = true;

  void materialize() const;
```

Implementation in `.cpp`:

```cpp
namespace {
inline Eigen::MatrixX3f to_matrix(const std::vector<patchwork::PointXYZ>& pts) {
  Eigen::MatrixX3f m(pts.size(), 3);
  for (size_t i = 0; i < pts.size(); ++i) {
    m(i, 0) = pts[i].x;
    m(i, 1) = pts[i].y;
    m(i, 2) = pts[i].z;
  }
  return m;
}
}  // namespace

void PatchWork::materialize() const {
  if (!outputs_dirty_) return;
  ground_mat_    = to_matrix(ground_pts_);
  nonground_mat_ = to_matrix(nonground_pts_);
  ground_idx_.clear();
  nonground_idx_.clear();
  for (const auto& p : ground_pts_)    ground_idx_.push_back(p.idx);
  for (const auto& p : nonground_pts_) nonground_idx_.push_back(p.idx);
  outputs_dirty_ = false;
}

Eigen::MatrixX3f PatchWork::getGround()    const { materialize(); return ground_mat_; }
Eigen::MatrixX3f PatchWork::getNonground() const { materialize(); return nonground_mat_; }
std::vector<int> PatchWork::getGroundIndices()    const { materialize(); return ground_idx_; }
std::vector<int> PatchWork::getNongroundIndices() const { materialize(); return nonground_idx_; }
double PatchWork::getTimeTaken() const { return time_taken_; }
double PatchWork::getHeight()    const { return sensor_height_; }
```

- \[ \] **Step 3: Build, run smoke test (still empty input — should succeed)**

```bash
cmake --build cpp/build -j && ./cpp/build/patchwork/patchwork_smoke
```

- \[ \] **Step 4: Commit**

```bash
git add cpp/patchwork/
git commit -m "feat(patchwork): port estimate_ground main pipeline"
```

### Task C9: Port `estimate_sensor_height` (ATAT)

**Files:**

- Modify: `cpp/patchwork/src/patchwork.cpp`, `cpp/patchwork/include/patchwork/patchwork.h`

Source: `up:453-525` for `estimate_sensor_height` and `up:394-451` for `consensus_set_based_height_estimation`.

- \[ \] **Step 1: Declare both methods (private)**

```cpp
void   estimate_sensor_height(std::vector<PointXYZ>& cloud);
double consensus_set_based_height_estimation(const std::vector<double>& candidate_heights);
```

- \[ \] **Step 2: Implement**

The consensus algorithm picks heights within `noise_bound` of each other and averages the largest cluster. Translate `up:394-451` literally — it's pure math, no PCL.

`estimate_sensor_height` (`up:453-525`) sorts points by xy distance, picks ones within `max_h_for_ATAT` of expected ground (using current `sensor_height_`), buckets them by sector, computes per-sector lowest-z, runs consensus on those candidates, and updates `sensor_height_`.

- \[ \] **Step 3: Build, commit**

```bash
cmake --build cpp/build -j
git add cpp/patchwork/
git commit -m "feat(patchwork): port ATAT (auto-tuning sensor height)"
```

______________________________________________________________________

## Phase D — Python bindings

### Task D1: Wire ground_seg_classic into the python module

**Files:**

- Modify: `python/CMakeLists.txt`

- \[ \] **Step 1: Add classic to add_subdirectory and link**

`python/CMakeLists.txt` already includes `cpp/` via `add_subdirectory(.../../cpp ...)`. That subdir now adds both targets. Update the `target_link_libraries` line:

```cmake
target_link_libraries(pypatchworkpp PUBLIC
    ${PARENT_PROJECT_NAME}::ground_seg_cores
    ${PARENT_PROJECT_NAME}::ground_seg_classic)
```

- \[ \] **Step 2: Verify pip install still works**

```bash
cd /tmp && rm -rf pkgtest_d1 && python3 -m venv pkgtest_d1
source pkgtest_d1/bin/activate
pip install --upgrade pip --quiet
pip install /Users/fudxo/git/patchwork-plusplus/python/
python -c "import pypatchworkpp; print(dir(pypatchworkpp))"
```

Expected: prints module symbols (still only the patchworkpp class until Task D2).

- \[ \] **Step 3: Commit**

```bash
git add python/CMakeLists.txt
git commit -m "build(python): link classic ground_seg target into pypatchworkpp module"
```

### Task D2: Add pybind11 bindings for PatchworkParams + patchwork

**Files:**

- Modify: `python/patchworkpp/pybinding.cpp`

- \[ \] **Step 1: Add include and bindings**

Edit `python/patchworkpp/pybinding.cpp`. Add at the top:

```cpp
#include "patchwork/patchwork.h"
```

Inside `PYBIND11_MODULE(pypatchworkpp, m)`, after the existing patchworkpp bindings, add:

```cpp
py::class_<patchwork::PatchworkParams>(m, "PatchworkParams")
    .def(py::init<>())
    .def_readwrite("sensor_height",         &patchwork::PatchworkParams::sensor_height)
    .def_readwrite("max_range",             &patchwork::PatchworkParams::max_range)
    .def_readwrite("min_range",             &patchwork::PatchworkParams::min_range)
    .def_readwrite("num_zones",             &patchwork::PatchworkParams::num_zones)
    .def_readwrite("num_sectors_each_zone", &patchwork::PatchworkParams::num_sectors_each_zone)
    .def_readwrite("num_rings_each_zone",   &patchwork::PatchworkParams::num_rings_each_zone)
    .def_readwrite("min_ranges",            &patchwork::PatchworkParams::min_ranges)
    .def_readwrite("num_iter",              &patchwork::PatchworkParams::num_iter)
    .def_readwrite("num_lpr",               &patchwork::PatchworkParams::num_lpr)
    .def_readwrite("num_min_pts",           &patchwork::PatchworkParams::num_min_pts)
    .def_readwrite("th_seeds",              &patchwork::PatchworkParams::th_seeds)
    .def_readwrite("th_dist",               &patchwork::PatchworkParams::th_dist)
    .def_readwrite("uprightness_thr",       &patchwork::PatchworkParams::uprightness_thr)
    .def_readwrite("elevation_thr",         &patchwork::PatchworkParams::elevation_thr)
    .def_readwrite("flatness_thr",          &patchwork::PatchworkParams::flatness_thr)
    .def_readwrite("adaptive_seed_selection_margin",
                   &patchwork::PatchworkParams::adaptive_seed_selection_margin)
    .def_readwrite("using_global_thr",      &patchwork::PatchworkParams::using_global_thr)
    .def_readwrite("global_elevation_thr",  &patchwork::PatchworkParams::global_elevation_thr)
    .def_readwrite("ATAT_ON",               &patchwork::PatchworkParams::ATAT_ON)
    .def_readwrite("max_h_for_ATAT",        &patchwork::PatchworkParams::max_h_for_ATAT)
    .def_readwrite("num_sectors_for_ATAT",  &patchwork::PatchworkParams::num_sectors_for_ATAT)
    .def_readwrite("noise_bound",           &patchwork::PatchworkParams::noise_bound)
    .def_readwrite("verbose",               &patchwork::PatchworkParams::verbose);

py::class_<patchwork::PatchWork>(m, "patchwork")
    .def(py::init<patchwork::PatchworkParams>())
    .def("estimateGround",     &patchwork::PatchWork::estimateGround)
    .def("getGround",          &patchwork::PatchWork::getGround)
    .def("getNonground",       &patchwork::PatchWork::getNonground)
    .def("getGroundIndices",   &patchwork::PatchWork::getGroundIndices)
    .def("getNongroundIndices",&patchwork::PatchWork::getNongroundIndices)
    .def("getTimeTaken",       &patchwork::PatchWork::getTimeTaken)
    .def("getHeight",          &patchwork::PatchWork::getHeight);
```

- \[ \] **Step 2: Reinstall and probe**

```bash
pip install --force-reinstall --no-deps /Users/fudxo/git/patchwork-plusplus/python/
python -c "import pypatchworkpp; pp=pypatchworkpp.patchwork(pypatchworkpp.PatchworkParams()); print('ok')"
```

Expected: `ok`.

- \[ \] **Step 3: Commit**

```bash
git add python/patchworkpp/pybinding.cpp
git commit -m "feat(python): expose PatchworkParams and patchwork class via pybind11"
```

### Task D3: Add Python smoke test for patchwork class

**Files:**

- Create: `python/tests/test_patchwork_smoke.py`

- \[ \] **Step 1: Write the test**

```python
# python/tests/test_patchwork_smoke.py
import os

import numpy as np
import pypatchworkpp


DATA_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data")


def _read_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def test_classic_module_exposes_api():
    assert hasattr(pypatchworkpp, "PatchworkParams")
    assert hasattr(pypatchworkpp, "patchwork")


def test_classic_estimate_ground_partitions_all_points():
    params = pypatchworkpp.PatchworkParams()
    pw = pypatchworkpp.patchwork(params)

    scan = _read_bin(os.path.join(DATA_DIR, "000000.bin"))
    pw.estimateGround(scan)

    ground = pw.getGround()
    nonground = pw.getNonground()

    assert ground.shape[0] > 0
    assert nonground.shape[0] > 0
    assert ground.shape[0] + nonground.shape[0] <= scan.shape[0]
```

- \[ \] **Step 2: Run all python tests**

```bash
pip install --force-reinstall --no-deps '/Users/fudxo/git/patchwork-plusplus/python[test]'
pytest -rA --verbose /Users/fudxo/git/patchwork-plusplus/python/
```

Expected: all 4 tests pass (2 patchworkpp + 2 patchwork).

- \[ \] **Step 3: Commit**

```bash
git add python/tests/test_patchwork_smoke.py
git commit -m "test(python): smoke test for patchwork classic against data/000000.bin"
```

______________________________________________________________________

## Phase E — ROS2 wiring

### Task E1: Add algorithm dispatch to GroundSegmentationServer

**Files:**

- Modify: `ros/src/GroundSegmentationServer.hpp`
- Modify: `ros/src/GroundSegmentationServer.cpp`

Read `ros/src/GroundSegmentationServer.hpp` and `.cpp` first. The current code holds a `std::unique_ptr<patchwork::PatchWorkpp>`. We change it to a `std::variant`.

- \[ \] **Step 1: In .hpp, replace the impl member**

Old:

```cpp
std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
```

New:

```cpp
#include <variant>
#include "patchwork/patchwork.h"
// existing includes for patchworkpp.h stay

using ImplVariant = std::variant<
    std::unique_ptr<patchwork::PatchWorkpp>,
    std::unique_ptr<patchwork::PatchWork>>;
ImplVariant impl_;
```

- \[ \] **Step 2: In .cpp constructor, branch on `algorithm` parameter**

Add at the top of the constructor (after node init, before any param reads that go into the patchworkpp Params):

```cpp
const std::string algorithm =
    declare_parameter<std::string>("algorithm", "patchworkpp");
```

Then load both Params structs from declared parameters (patchworkpp.h existing parameters keep working; add a parallel block for `PatchworkParams`). Branch:

```cpp
if (algorithm == "patchwork") {
  patchwork::PatchworkParams classic_params = loadClassicParamsFromROS();
  impl_ = std::make_unique<patchwork::PatchWork>(classic_params);
  RCLCPP_INFO(get_logger(), "Algorithm: patchwork (classic)");
} else {
  patchwork::Params plusplus_params = loadPlusplusParamsFromROS();
  impl_ = std::make_unique<patchwork::PatchWorkpp>(plusplus_params);
  RCLCPP_INFO(get_logger(), "Algorithm: patchworkpp (default)");
}
```

Extract `loadPlusplusParamsFromROS()` and `loadClassicParamsFromROS()` as private helpers.

- \[ \] **Step 3: Replace direct calls with std::visit**

Anywhere the code calls `patchwork_->estimateGround(...)` or `patchwork_->getGround()`, rewrite as:

```cpp
std::visit([&](auto& impl) { impl->estimateGround(cloud); }, impl_);
```

For the getters, the variant alternatives have identical method names so `std::visit` lambdas work uniformly:

```cpp
auto ground = std::visit([](auto& impl) { return impl->getGround(); }, impl_);
```

- \[ \] **Step 4: Build the ROS package**

```bash
cd /Users/fudxo/git/patchwork-plusplus
docker run --rm -v $(pwd):/src -w /src osrf/ros:humble-desktop \
  bash -lc 'source /opt/ros/humble/setup.bash && colcon build --packages-select patchworkpp --event-handlers console_direct+'
```

Expected: build succeeds. (If Docker is unavailable, just build via colcon on a Linux host with ROS humble.)

- \[ \] **Step 5: Commit**

```bash
git add ros/src/GroundSegmentationServer.hpp ros/src/GroundSegmentationServer.cpp
git commit -m "feat(ros): dispatch on algorithm parameter (patchwork vs patchworkpp)"
```

### Task E2: Expose `algorithm` launch arg

**Files:**

- Modify: `ros/launch/patchworkpp.launch.py`

- \[ \] **Step 1: Read the launch file**

```bash
cat ros/launch/patchworkpp.launch.py
```

- \[ \] **Step 2: Add a DeclareLaunchArgument and pass it through**

Add to the `LaunchDescription` items, before the `Node(...)`:

```python
DeclareLaunchArgument(
    "algorithm",
    default_value="patchworkpp",
    description="Ground segmentation algorithm: 'patchwork' or 'patchworkpp'",
),
```

In the `Node(...)` parameters list, append:

```python
{"algorithm": LaunchConfiguration("algorithm")},
```

(Adjust imports: add `from launch.actions import DeclareLaunchArgument` and `from launch.substitutions import LaunchConfiguration` if not already present.)

- \[ \] **Step 3: Commit**

```bash
git add ros/launch/patchworkpp.launch.py
git commit -m "feat(ros): expose 'algorithm' launch argument"
```

______________________________________________________________________

## Phase F — Documentation and version bump

### Task F1: Add "Choosing an algorithm" subsection to README

**Files:**

- Modify: `README.md`

- \[ \] **Step 1: Add the subsection**

Find the `## :gear: How to build & Run` section. After the existing Python/C++/ROS2 subsections, before `## :pencil: Citation`, insert:

```markdown
## :compass: Choosing an algorithm

This repository ships two ground segmentation algorithms with the same input/output API. Pick the one that fits your data:

- **Patchwork++** (default): adaptive elevation/flatness thresholds, RNR (intensity-based reflected noise removal), RVPF (vertical structure suppression), and TGR (probability-based ground revert). Best when the LiDAR has reflection artefacts or you want self-tuning thresholds.
- **Patchwork** (classic, since 1.1.0): fixed elevation/flatness thresholds with explicit `z < -sensor_height - 2.0m` cutoff and few-points reject, plus optional ATAT for unknown sensor heights. Often more aggressive on ground-plane noise in heavily cluttered scenes.

Python:

\`\`\`python
import pypatchworkpp as p

pp_default = p.patchworkpp(p.Parameters())          # Patchwork++
pp_classic = p.patchwork(p.PatchworkParams())       # Patchwork (classic)
\`\`\`

ROS2:

\`\`\`bash
ros2 launch patchworkpp patchworkpp.launch.py algorithm:=patchwork
\`\`\`
```

(Use real backticks for the code fences; the example here escapes them so the markdown plan parses cleanly.)

- \[ \] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs(readme): add 'Choosing an algorithm' subsection for patchwork vs patchwork++"
```

### Task F2: Bump version 1.0.4 → 1.1.0

**Files:**

- Modify: `python/pyproject.toml`

- Modify: `cpp/CMakeLists.txt`

- \[ \] **Step 1: Bump in pyproject.toml**

Change `version = "1.0.4"` to `version = "1.1.0"` (line 7 of `python/pyproject.toml`).

- \[ \] **Step 2: Bump in cpp/CMakeLists.txt**

Change `project(patchworkpp VERSION 1.0.4)` to `project(patchworkpp VERSION 1.1.0)` (line 2).

- \[ \] **Step 3: Reinstall and verify version**

```bash
pip install --force-reinstall --no-deps /Users/fudxo/git/patchwork-plusplus/python/
python -c "import pypatchworkpp; print(pypatchworkpp.patchwork.__init__.__doc__)"
```

(Sanity, not a hard assertion.)

- \[ \] **Step 4: Commit**

```bash
git add python/pyproject.toml cpp/CMakeLists.txt
git commit -m "chore: bump version to 1.1.0 (adds Patchwork classic algorithm)"
```

______________________________________________________________________

## Phase G — PR

### Task G1: Push branch and create PR

- \[ \] **Step 1: Final local CI run**

Build cpp + run python tests once more:

```bash
rm -rf cpp/build
cmake -Bcpp/build cpp/ -DBUILD_PATCHWORK_TESTS=ON
cmake --build cpp/build -j
./cpp/build/patchwork/patchwork_smoke

cd /tmp && python3 -m venv finalpkg && source finalpkg/bin/activate
pip install --upgrade pip --quiet
pip install '/Users/fudxo/git/patchwork-plusplus/python[test]'
pytest -rA --verbose /Users/fudxo/git/patchwork-plusplus/python/
```

All green.

- \[ \] **Step 2: Push and open PR**

```bash
git push -u origin feat/patchwork-classic
gh pr create --base master --head feat/patchwork-classic \
  --title "feat: add Patchwork (classic) algorithm alongside Patchwork++" \
  --body "$(cat docs/superpowers/specs/2026-05-09-add-patchwork-classic-algorithm-design.md | sed -n '/^## Background/,/^## Out of scope/p')"
```

- \[ \] **Step 3: Wait for CI, address feedback, merge**

Use `gh pr checks <PR#> --watch` or the same background-watcher pattern from PR #78.
