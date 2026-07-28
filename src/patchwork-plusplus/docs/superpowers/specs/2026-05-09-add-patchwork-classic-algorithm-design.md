# Add Patchwork (Classic) Algorithm Alongside Patchwork++

Date: 2026-05-09
Status: Draft, awaiting user approval

## Background

The maintainer suspects that the original Patchwork algorithm (predecessor of Patchwork++) handles ground-plane noise more aggressively in some scenes, particularly via the simple `z < -sensor_height - 2.0m` cutoff and per-patch reject-if-too-few-points policy. Rather than try to merge two genuinely different algorithms (Patchwork uses fixed elevation/flatness thresholds, Patchwork++ uses adaptive ones plus RNR/RVPF/TGR), we expose **both** in this repository so users can A/B compare on their own data and pick what fits.

## Goals

- Add the original Patchwork algorithm as a self-contained library target inside this repo, with the **identical I/O signature** as Patchwork++ (Eigen::MatrixXf in, Eigen::MatrixX3f out).
- Expose it through the same Python module as a parallel class.
- Wire a runtime `algorithm` switch into the ROS2 node so a single launch can run either.
- Preserve the existing Patchwork++ public API and behavior bit-for-bit. Adding the classic algorithm is purely additive.

## Non-goals

- No port-and-merge: we are not selectively grafting Patchwork's noise removal onto Patchwork++. Both implementations stay independent.
- No PCL or ROS dependency in the algorithm core. ROS support stays in `ros/`.
- No PyPI release in this PR. Version bump and release are a follow-up.

## Architecture

### Source layout

```
cpp/
├── patchworkpp/                       # unchanged
│   ├── include/patchwork/patchworkpp.h
│   └── src/patchworkpp.cpp
├── patchwork/                         # NEW
│   ├── include/patchwork/patchwork.h
│   └── src/patchwork.cpp
└── CMakeLists.txt                     # adds add_subdirectory(patchwork)

python/
├── patchworkpp/pybinding.cpp          # extended to bind both classes
└── tests/
    ├── test_smoke.py                  # unchanged (Patchwork++)
    └── test_patchwork_smoke.py        # NEW

ros/
├── src/GroundSegmentationServer.cpp   # adds algorithm switch
├── src/GroundSegmentationServer.hpp   # adds variant member
└── launch/patchworkpp.launch.py       # exposes algorithm parameter
```

### CMake targets

- `patchworkpp::ground_seg_cores` — unchanged. Builds the Patchwork++ algorithm.
- `patchworkpp::ground_seg_classic` — new. Builds the Patchwork (classic) algorithm. Static archive, Eigen-only dependency.

The two targets share the `patchworkpp::` namespace prefix so an installed `find_package(patchworkpp)` pulls both. The CMake target identifier (`ground_seg_classic`) is independent of the C++ class names; the C++ namespace and class names below are not affected by the target name choice.

## C++ API

Both algorithms live under `namespace patchwork`. Two parameter structs, two classes, no virtual base (kept simple and zero-cost; users pick concretely).

```cpp
namespace patchwork {

// Existing — unchanged
struct Params { /* Patchwork++ parameters */ };
class  PatchWorkpp { /* Patchwork++ implementation */ };

// New
struct PatchworkParams {
  // Sensor / range
  double sensor_height = 1.723;
  double max_range     = 80.0;
  double min_range     = 2.7;

  // Concentric Zone Model
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

  // Ground likelihood thresholds (fixed, the Patchwork classic style)
  double              uprightness_thr = 0.5;
  std::vector<double> elevation_thr   = {0.523, 0.746, 0.879, 1.125};
  std::vector<double> flatness_thr    = {0.0005, 0.000725, 0.001, 0.001};

  // ATAT (All-Terrain Automatic sensor-height Estimator)
  bool   ATAT_ON              = true;   // default ON per maintainer preference
  double max_h_for_ATAT       = 0.3;
  int    num_sectors_for_ATAT = 20;

  bool verbose = false;
};

class PatchWork {
 public:
  PatchWork() = default;
  explicit PatchWork(const PatchworkParams& params);

  // Identical signature to PatchWorkpp::estimateGround
  void estimateGround(const Eigen::MatrixXf& cloud);  // N x 4 (x, y, z, intensity)

  Eigen::MatrixX3f getGround()           const;
  Eigen::MatrixX3f getNonground()        const;
  std::vector<int> getGroundIndices()    const;
  std::vector<int> getNongroundIndices() const;
  double           getTimeTaken()        const;
  double           getHeight()           const;  // Updated by ATAT if enabled

 private:
  // Implementation (PCL/ROS-free port from /Users/fudxo/git/patchwork)
};

}  // namespace patchwork
```

The class deliberately has no virtual base. Users that want polymorphism can wrap with `std::variant<PatchWork, PatchWorkpp>` or function pointers, but the default API is concrete. Both classes have identical method names so generic templates over the algorithm are trivial.

## Adapting the original Patchwork code

The upstream `/Users/fudxo/git/patchwork` algorithm (`include/patchwork/patchwork.hpp`) is ROS-coupled and depends on PCL, TBB, Boost.Format, and rclcpp. Verified by header inspection:

```
#include <pcl/common/centroid.h>           # centroid math
#include <pcl/io/pcd_io.h>                  # PCD I/O (test only)
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>                   # ROS 2 QoS
#include <rclcpp/rclcpp.hpp>
#include <tbb/parallel_for.h>               # patch parallelism
#include <boost/format.hpp>                 # verbose logging
```

Our adapter strips all of these:

1. **Input conversion** — a private helper `convertToPointVec(const Eigen::MatrixXf&)` produces `std::vector<patchwork::PointXYZ>` (the existing POD struct from `cpp/patchworkpp/include/patchwork/patchworkpp.h`, reused so both algorithms share point representation).
1. **Output conversion** — `Eigen::MatrixX3f` built from accumulated ground/nonground vectors, identical to Patchwork++'s helpers.
1. **PCL → Eigen/std** — `pcl::PointCloud<PointXYZI>` becomes `std::vector<PointXYZ>`. `pcl::compute3DCentroid` and `pcl::getMinMax3D` become Eigen mean / min/max reductions.
1. **TBB → sequential** — the `tbb::parallel_for` over patches (patchwork.hpp:578) is replaced with a plain `for` loop. Patchwork++'s implementation is also sequential and fast enough for typical 100k-point scans, so the perf delta is acceptable. We do not add an optional TBB toggle in this PR.
1. **Boost.Format → std::cout** — verbose logging only.
1. **rclcpp** — every reference is in the ROS publishing path, not the algorithm itself; it does not appear in our adapter.
1. **ATAT** — ported as-is (sort patches by mean z, average the lowest N, set sensor_height).

The semantics of the original algorithm (seed selection, plane fitting, GLE thresholds) stay byte-for-byte logically equivalent. We do not "improve" the upstream algorithm; we adapt only the I/O surface.

## Python API

`python/patchworkpp/pybinding.cpp` is extended to bind the new class and parameter struct. The existing bindings stay untouched.

```python
import pypatchworkpp as p

# Existing (unchanged)
pp = p.patchworkpp(p.Parameters())
pp.estimateGround(scan)
ground = pp.getGround()

# New
classic = p.patchwork(p.PatchworkParams())
classic.estimateGround(scan)
ground = classic.getGround()
```

The Python class is named `patchwork` (lowercase) to match the existing `patchworkpp` casing convention. Method names are identical so the same downstream code can swap between them.

## ROS2 API

The ROS node gains a string parameter `algorithm` (default `"patchworkpp"`). At construction:

```cpp
class GroundSegmentationServer : public rclcpp::Node {
  // ...
  std::variant<std::unique_ptr<patchwork::PatchWorkpp>,
               std::unique_ptr<patchwork::PatchWork>> impl_;
};
```

`EstimateGround` and the getters dispatch via `std::visit`. The ROS launch file exposes `algorithm` as a launch argument so users can flip without rebuilding:

```bash
ros2 launch patchworkpp patchworkpp.launch.py algorithm:=patchwork
```

ROS CI runs both `humble` and `jazzy` builds, which already exercise the C++ build. We do not add a runtime launch test in this PR — that scope belongs to a separate testing PR.

## Testing

- `python/tests/test_smoke.py` — unchanged. Asserts Patchwork++ still partitions all input points on `data/000000.bin`.
- `python/tests/test_patchwork_smoke.py` — new. Same assertions but instantiates `pypatchworkpp.patchwork(PatchworkParams())`. Confirms the new binding loads and produces non-empty ground/nonground arrays.

No correctness comparison test (e.g., asserting one algorithm matches the other) — they are deliberately different. The smoke tests only verify that both pipelines run end to end on real data.

## Build / CI impact

- `cpp/CMakeLists.txt` adds `add_subdirectory(patchwork)`. The existing `cpp_api` matrix job (Ubuntu 22/24, Windows 2022, macOS 14) builds it automatically.
- `python/CMakeLists.txt` links the pybind module against both `patchworkpp::ground_seg_cores` and `patchworkpp::ground_seg_classic`.
- No new third-party dependency. Only Eigen3 (already FetchContent'd) and pybind11.
- `cpp/example_of_find_package/` is **not** updated in this PR — that example demonstrates `find_package(patchworkpp)` and continues to work. A separate example for the classic algorithm is out of scope.

## Versioning

Bump `python/pyproject.toml` and `cpp/CMakeLists.txt` from `1.0.4` to `1.1.0` (minor — additive feature). Actual PyPI release is a follow-up: cut a GitHub Release after this PR merges and the `pypa/gh-action-pypi-publish` step uploads.

## README updates

Add a "Choosing an algorithm" subsection under "How to build & Run":

- When to prefer Patchwork++ (default, adaptive thresholds, RNR/RVPF/TGR for noisy LiDAR).
- When to prefer Patchwork (fixed thresholds, simpler behavior, ATAT for unknown sensor heights).
- Example snippets for both.

## Risks and open questions

1. **Code volume** — the upstream `patchwork.hpp` is roughly 1k lines including PCL plumbing. After stripping PCL/ROS we expect 600–800 lines of pure algorithm. Reviewable but not trivial.
1. **Numerical equivalence** — we do not have a regression dataset to assert that our PCL-free port yields identical ground/nonground partitions for the same input as upstream Patchwork. We accept this: the upstream is the reference, ours is a faithful adaptation, and behavior may drift in floating-point edge cases.
1. **Eigen vector resize semantics** — original Patchwork uses pcl point cloud `push_back`; we use `std::vector<PointXYZ>` then materialize to `Eigen::MatrixX3f` once. This shifts the cost profile slightly but keeps allocations predictable.
1. **Build-target naming** — `ground_seg_classic` is the chosen identifier, deliberately neutral and tied to the build system, not user-facing.

## Out of scope (explicit)

- Cross-algorithm benchmark suite or paper-style evaluation.
- Refactoring or "improving" the original Patchwork algorithm.
- Running Patchwork via Patchwork++'s adaptive threshold loop or vice versa.
- C# / Rust bindings, GPU offload, threading changes.
- Updating `cpp/example_of_find_package/` to demonstrate the classic algorithm.

## Implementation phases (preview, not part of this spec)

The detailed plan is the next step (writing-plans skill). Expected sequence:

1. Bring in `cpp/patchwork/` skeleton with placeholder algorithm; verify CMake hooks compile.
1. Port the upstream algorithm body, removing PCL types incrementally; cross-check with the smoke test.
1. Wire the Python binding, add the new smoke test.
1. Wire the ROS variant dispatch and the launch parameter.
1. README + version bump.
