// Two-layer terrain-aware obstacle detector — algorithm core (C++ port).
//
// Direct port of lidar3d_bringup/lidar3d_bringup/surface_detector.py.
// Function names mirror the Python originals so the two can be diffed.
//
// Layer 1 — Surface model: smoothed 2.5D polar grid from Patchwork++ ground points.
// Layer 2 — Residual analysis: points significantly above S(r,theta) are obstacles.

#ifndef LIDAR3D_PERCEPTION_CPP__SURFACE_DETECTOR_HPP_
#define LIDAR3D_PERCEPTION_CPP__SURFACE_DETECTOR_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace lidar3d
{

// --- type system (simplified to 4 types) ---
enum ObstacleType : int
{
  TYPE_OBSTACLE = 0,       // 不可通过障碍物
  TYPE_PASSABLE_LOW = 1,   // 可通过（坡、细杆、减速带、粗糙地形）
  TYPE_PASSABLE_HIGH = 2,  // 需减速通过（波浪路、坑洼）
  TYPE_UNKNOWN = 3,        // 未知/低置信度
};

inline const char * typeLabel(int t)
{
  switch (t) {
    case TYPE_OBSTACLE: return "obstacle";
    case TYPE_PASSABLE_LOW: return "passable_low";
    case TYPE_PASSABLE_HIGH: return "passable_high";
    default: return "unknown";
  }
}

// (r,g,b,a) — mirrors Python TYPE_COLORS
inline void typeColor(int t, float & r, float & g, float & b, float & a)
{
  switch (t) {
    case TYPE_OBSTACLE:      r = 1.0f; g = 0.0f; b = 0.0f; a = 0.7f; break;
    case TYPE_PASSABLE_LOW:  r = 0.0f; g = 0.8f; b = 0.0f; a = 0.5f; break;
    case TYPE_PASSABLE_HIGH: r = 1.0f; g = 0.9f; b = 0.0f; a = 0.6f; break;
    default:                 r = 0.7f; g = 0.7f; b = 0.7f; a = 0.4f; break;
  }
}

using Point3 = Eigen::Vector3d;
using Cloud = std::vector<Point3>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ======================================================================
// Layer 1 — Surface model
// ======================================================================

struct PolarGrid
{
  int n_r = 0;
  int n_th = 0;
  std::vector<double> r_edges;      // size n_r + 1
  std::vector<double> th_edges;     // size n_th + 1
  std::vector<double> z_median;     // n_r * n_th, row-major; NaN = empty bin
  std::vector<double> z_mad;        // n_r * n_th
  std::vector<int> count;           // n_r * n_th
  std::vector<int> outlier_indices; // indices into the input cloud

  inline int idx(int ri, int ti) const {return ri * n_th + ti;}
  bool valid() const {return n_r > 0 && n_th > 0;}
};

struct SlopePatch
{
  Point3 center = Point3::Zero();
  Point3 apex = Point3::Zero();
  Point3 dims = Point3::Zero();
  double max_grade_deg = 0.0;
  int cell_count = 0;
};

// Extract connected uphill/downhill patches from points Patchwork++ already
// accepted as ground.  Each Cartesian cell receives a local z=ax+by+c fit;
// only the longitudinal derivative a is used for the slope grade so ordinary
// road camber does not become a longitudinal terrain feature.  The result is
// independent of the residual obstacle path below.
inline std::vector<SlopePatch> detectSlopePatches(
  const Cloud & ground,
  double cell_size_m = 0.5,
  double fit_radius_m = 1.5,
  double min_grade_deg = 1.5,
  double max_grade_deg = 18.0,
  int min_support_cells = 8,
  double max_fit_rmse_m = 0.08,
  int min_component_cells = 6,
  double min_span_x_m = 2.0,
  double min_forward_m = 0.5,
  double max_forward_m = 30.0)
{
  struct CellSamples
  {
    std::vector<Point3> points;
  };
  struct GroundCell
  {
    Point3 center = Point3::Zero();
    double z = 0.0;
  };

  if (ground.empty() || cell_size_m <= 0.0 || fit_radius_m <= 0.0 ||
      min_grade_deg < 0.0 || max_grade_deg <= min_grade_deg ||
      min_support_cells < 3 || min_component_cells < 1 ||
      min_span_x_m <= 0.0 || max_forward_m <= min_forward_m)
  {
    return {};
  }

  auto keyOf = [](int ix, int iy) -> int64_t {
      return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
    };
  auto keyX = [](int64_t key) -> int {
      return static_cast<int>(key >> 32);
    };
  auto keyY = [](int64_t key) -> int {
      return static_cast<int>(static_cast<uint32_t>(key & 0xFFFFFFFF));
    };

  std::unordered_map<int64_t, CellSamples> sample_cells;
  sample_cells.reserve(ground.size());
  for (const auto & point : ground) {
    if (point.x() < min_forward_m || point.x() > max_forward_m) {
      continue;
    }
    const int ix = static_cast<int>(std::floor(point.x() / cell_size_m));
    const int iy = static_cast<int>(std::floor(point.y() / cell_size_m));
    sample_cells[keyOf(ix, iy)].points.push_back(point);
  }

  std::unordered_map<int64_t, GroundCell> cells;
  cells.reserve(sample_cells.size());
  for (auto & entry : sample_cells) {
    auto & points = entry.second.points;
    if (points.empty()) {
      continue;
    }
    Point3 center = Point3::Zero();
    std::vector<double> z_values;
    z_values.reserve(points.size());
    for (const auto & point : points) {
      center += point;
      z_values.push_back(point.z());
    }
    center /= static_cast<double>(points.size());
    std::nth_element(
      z_values.begin(), z_values.begin() + z_values.size() / 2, z_values.end());
    cells.emplace(entry.first, GroundCell{center, z_values[z_values.size() / 2]});
  }

  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(fit_radius_m / cell_size_m)));
  std::unordered_map<int64_t, double> candidate_grades;
  candidate_grades.reserve(cells.size());
  for (const auto & entry : cells) {
    const int ix = keyX(entry.first);
    const int iy = keyY(entry.first);
    std::vector<const GroundCell *> support;
    support.reserve((2 * radius_cells + 1) * (2 * radius_cells + 1));
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const auto neighbor = cells.find(keyOf(ix + dx, iy + dy));
        if (neighbor == cells.end()) {
          continue;
        }
        const Point3 delta = neighbor->second.center - entry.second.center;
        if (delta.head<2>().norm() <= fit_radius_m) {
          support.push_back(&neighbor->second);
        }
      }
    }
    if (static_cast<int>(support.size()) < min_support_cells) {
      continue;
    }

    Eigen::MatrixXd matrix(support.size(), 3);
    Eigen::VectorXd heights(support.size());
    for (size_t index = 0; index < support.size(); ++index) {
      matrix(index, 0) = support[index]->center.x();
      matrix(index, 1) = support[index]->center.y();
      matrix(index, 2) = 1.0;
      heights(index) = support[index]->z;
    }
    const Eigen::Vector3d plane = matrix.colPivHouseholderQr().solve(heights);
    const Eigen::VectorXd errors = matrix * plane - heights;
    const double rmse = std::sqrt(errors.squaredNorm() / static_cast<double>(support.size()));
    if (!std::isfinite(rmse) || rmse > max_fit_rmse_m) {
      continue;
    }

    const double grade_deg = std::atan(std::abs(plane.x())) * 180.0 / M_PI;
    if (grade_deg >= min_grade_deg && grade_deg <= max_grade_deg) {
      candidate_grades.emplace(entry.first, grade_deg);
    }
  }

  std::vector<SlopePatch> patches;
  std::unordered_map<int64_t, bool> visited;
  visited.reserve(candidate_grades.size());
  for (const auto & entry : candidate_grades) {
    if (visited[entry.first]) {
      continue;
    }
    visited[entry.first] = true;
    std::vector<int64_t> stack{entry.first};
    std::vector<int64_t> component{entry.first};
    while (!stack.empty()) {
      const int64_t current = stack.back();
      stack.pop_back();
      const int ix = keyX(current);
      const int iy = keyY(current);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int64_t neighbor = keyOf(ix + dx, iy + dy);
          if (candidate_grades.count(neighbor) && !visited[neighbor]) {
            visited[neighbor] = true;
            stack.push_back(neighbor);
            component.push_back(neighbor);
          }
        }
      }
    }

    if (static_cast<int>(component.size()) < min_component_cells) {
      continue;
    }
    Point3 sum = Point3::Zero();
    Point3 apex = Point3::Zero();
    bool have_apex = false;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double component_max_grade = 0.0;
    for (int64_t key : component) {
      const GroundCell & cell = cells.at(key);
      sum += cell.center;
      if (!have_apex || cell.z > apex.z()) {
        apex = cell.center;
        have_apex = true;
      }
      min_x = std::min(min_x, cell.center.x());
      max_x = std::max(max_x, cell.center.x());
      min_y = std::min(min_y, cell.center.y());
      max_y = std::max(max_y, cell.center.y());
      min_z = std::min(min_z, cell.z);
      max_z = std::max(max_z, cell.z);
      component_max_grade = std::max(component_max_grade, candidate_grades.at(key));
    }
    const double span_x = max_x - min_x + cell_size_m;
    if (span_x < min_span_x_m) {
      continue;
    }
    SlopePatch patch;
    patch.center = sum / static_cast<double>(component.size());
    patch.apex = apex;
    patch.dims = Point3(
      span_x,
      max_y - min_y + cell_size_m,
      std::max(0.05, max_z - min_z));
    patch.max_grade_deg = component_max_grade;
    patch.cell_count = static_cast<int>(component.size());
    patches.push_back(patch);
  }
  return patches;
}

// Port of _build_polar_grid(). Bins ground points into a polar grid.
//
// Grid spacing grows with range: dr = dr_base + dr_per_m * r
//
// Outlier handling (方案A): Patchwork++ may mis-assign obstacle points to
// /patchworkpp/ground on slopes. Per bin the surface height is estimated from
// the LOWEST third of the points (ground is the continuous lowest surface, so
// even if obstacle points dominate a bin the lowest third is still real
// ground), and points above ground_h + max(k*MAD, 0.15) are flagged as
// outliers for downstream residual detection.
inline PolarGrid buildPolarGrid(
  const Cloud & xyz,
  double r_min = 0.5, double r_max = 35.0,
  double dr_base = 0.10, double dr_per_m = 0.02,
  double dth_deg = 1.5, double outlier_factor = 2.0)
{
  PolarGrid grid;

  // range filter, keeping original indices
  std::vector<int> keep_idx;
  std::vector<double> r_f, th_f, z_f;
  keep_idx.reserve(xyz.size());
  r_f.reserve(xyz.size());
  th_f.reserve(xyz.size());
  z_f.reserve(xyz.size());
  for (size_t i = 0; i < xyz.size(); ++i) {
    const double rr = std::hypot(xyz[i].x(), xyz[i].y());
    if (rr >= r_min && rr <= r_max) {
      keep_idx.push_back(static_cast<int>(i));
      r_f.push_back(rr);
      th_f.push_back(std::atan2(xyz[i].y(), xyz[i].x()));
      z_f.push_back(xyz[i].z());
    }
  }
  if (r_f.size() < 10) {return grid;}  // invalid()

  // radial bins with growing spacing
  grid.r_edges.push_back(r_min);
  while (grid.r_edges.back() < r_max) {
    grid.r_edges.push_back(grid.r_edges.back() + dr_base + dr_per_m * grid.r_edges.back());
  }
  grid.n_r = static_cast<int>(grid.r_edges.size()) - 1;

  // angular bins
  const int n_th = static_cast<int>(360.0 / dth_deg);
  grid.n_th = n_th;
  grid.th_edges.resize(n_th + 1);
  for (int i = 0; i <= n_th; ++i) {
    grid.th_edges[i] = -M_PI + 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n_th);
  }

  const size_t cells = static_cast<size_t>(grid.n_r) * static_cast<size_t>(grid.n_th);
  grid.z_median.assign(cells, kNaN);
  grid.z_mad.assign(cells, kNaN);
  grid.count.assign(cells, 0);

  // bucket point indices by flattened bin (equivalent to np.digitize + groupby)
  std::unordered_map<int, std::vector<int>> bins;
  bins.reserve(r_f.size());
  for (size_t i = 0; i < r_f.size(); ++i) {
    // np.digitize(r, r_edges) - 1, clipped
    const int ri = std::clamp(
      static_cast<int>(std::upper_bound(grid.r_edges.begin(), grid.r_edges.end(), r_f[i]) -
      grid.r_edges.begin()) - 1, 0, grid.n_r - 1);
    const int ti = std::clamp(
      static_cast<int>(std::upper_bound(grid.th_edges.begin(), grid.th_edges.end(), th_f[i]) -
      grid.th_edges.begin()) - 1, 0, grid.n_th - 1);
    bins[ri * grid.n_th + ti].push_back(static_cast<int>(i));
  }

  std::vector<double> scratch;
  for (const auto & kv : bins) {
    const auto & members = kv.second;
    const int n_pts = static_cast<int>(members.size());
    if (n_pts < 3) {continue;}

    scratch.clear();
    scratch.reserve(members.size());
    for (int m : members) {scratch.push_back(z_f[m]);}

    // lowest third estimates the ground (np.argpartition equivalent)
    const int k_ground = std::max(3, n_pts / 3);
    std::nth_element(scratch.begin(), scratch.begin() + (k_ground - 1), scratch.end());
    std::vector<double> ground_cand(scratch.begin(), scratch.begin() + k_ground);

    std::nth_element(ground_cand.begin(), ground_cand.begin() + k_ground / 2, ground_cand.end());
    double ground_h = ground_cand[k_ground / 2];
    if (k_ground % 2 == 0) {
      // even count → mean of the two central values, matching np.median
      const double hi = ground_cand[k_ground / 2];
      std::nth_element(
        ground_cand.begin(), ground_cand.begin() + (k_ground / 2 - 1), ground_cand.end());
      ground_h = 0.5 * (ground_cand[k_ground / 2 - 1] + hi);
    }

    std::vector<double> devs(k_ground);
    for (int i = 0; i < k_ground; ++i) {devs[i] = std::abs(ground_cand[i] - ground_h);}
    std::nth_element(devs.begin(), devs.begin() + k_ground / 2, devs.end());
    double mad_raw = devs[k_ground / 2];
    if (k_ground % 2 == 0) {
      const double hi = devs[k_ground / 2];
      std::nth_element(devs.begin(), devs.begin() + (k_ground / 2 - 1), devs.end());
      mad_raw = 0.5 * (devs[k_ground / 2 - 1] + hi);
    }
    const double ground_mad = 1.4826 * mad_raw;

    // outlier threshold with a floor so small terrain ripples are not captured
    const double thresh = std::max(outlier_factor * ground_mad, 0.15);
    for (int m : members) {
      if (z_f[m] > ground_h + thresh) {
        grid.outlier_indices.push_back(keep_idx[m]);
      }
    }

    const int cell = kv.first;
    grid.z_median[cell] = ground_h;  // surface height = ground (not raised by obstacles)
    grid.z_mad[cell] = ground_mad;
    grid.count[cell] = n_pts;
  }

  std::sort(grid.outlier_indices.begin(), grid.outlier_indices.end());
  return grid;
}

// Port of _fill_and_smooth(). NaN propagation, radial interpolation, Gaussian blur.
//
// Unlike the Python version (which falls back to a box blur because system SciPy
// is incompatible with NumPy 2.x) this uses a proper separable Gaussian kernel.
inline std::vector<double> fillAndSmooth(const PolarGrid & grid, double sigma = 1.0)
{
  const int n_r = grid.n_r, n_th = grid.n_th;
  std::vector<double> z = grid.z_median;

  // iterative 4-neighbour mean fill (theta wraps around)
  for (int iter = 0; iter < 3; ++iter) {
    std::vector<double> src = z;
    for (int ri = 0; ri < n_r; ++ri) {
      for (int ti = 0; ti < n_th; ++ti) {
        const int cell = ri * n_th + ti;
        if (!std::isnan(src[cell])) {continue;}
        double sum = 0.0;
        int cnt = 0;
        const int nb_r[4] = {ri + 1, ri - 1, ri, ri};
        const int nb_t[4] = {ti, ti, (ti + 1) % n_th, (ti - 1 + n_th) % n_th};
        for (int k = 0; k < 4; ++k) {
          if (nb_r[k] < 0 || nb_r[k] >= n_r) {continue;}
          const double v = src[nb_r[k] * n_th + nb_t[k]];
          if (!std::isnan(v)) {sum += v; ++cnt;}
        }
        if (cnt > 0) {z[cell] = sum / cnt;}
      }
    }
  }

  // linear interpolation along r for whatever is still NaN (np.interp semantics:
  // values outside the valid span clamp to the nearest valid sample)
  for (int ti = 0; ti < n_th; ++ti) {
    std::vector<int> valid;
    for (int ri = 0; ri < n_r; ++ri) {
      if (!std::isnan(z[ri * n_th + ti])) {valid.push_back(ri);}
    }
    if (valid.size() < 2) {continue;}
    for (int ri = 0; ri < n_r; ++ri) {
      const int cell = ri * n_th + ti;
      if (!std::isnan(z[cell])) {continue;}
      if (ri < valid.front()) {
        z[cell] = z[valid.front() * n_th + ti];
      } else if (ri > valid.back()) {
        z[cell] = z[valid.back() * n_th + ti];
      } else {
        const auto up = std::upper_bound(valid.begin(), valid.end(), ri);
        const int hi = *up, lo = *(up - 1);
        const double t = static_cast<double>(ri - lo) / static_cast<double>(hi - lo);
        z[cell] = z[lo * n_th + ti] * (1.0 - t) + z[hi * n_th + ti] * t;
      }
    }
  }

  // separable Gaussian blur; r edges clamp ('nearest'), theta wraps
  if (sigma > 1e-6) {
    const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
    std::vector<double> kernel(2 * radius + 1);
    double ksum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
      kernel[i + radius] = std::exp(-0.5 * (i * i) / (sigma * sigma));
      ksum += kernel[i + radius];
    }
    for (double & k : kernel) {k /= ksum;}

    std::vector<double> tmp(z.size());
    for (int ri = 0; ri < n_r; ++ri) {      // along theta (wrap)
      for (int ti = 0; ti < n_th; ++ti) {
        double acc = 0.0;
        for (int k = -radius; k <= radius; ++k) {
          const int tt = ((ti + k) % n_th + n_th) % n_th;
          acc += kernel[k + radius] * z[ri * n_th + tt];
        }
        tmp[ri * n_th + ti] = acc;
      }
    }
    for (int ti = 0; ti < n_th; ++ti) {     // along r (clamp)
      for (int ri = 0; ri < n_r; ++ri) {
        double acc = 0.0;
        for (int k = -radius; k <= radius; ++k) {
          const int rr = std::clamp(ri + k, 0, n_r - 1);
          acc += kernel[k + radius] * tmp[rr * n_th + ti];
        }
        z[ri * n_th + ti] = acc;
      }
    }
  }

  return z;
}

// Port of _sample_surface(). Returns surface height and the bin's real ground
// point count (count < 3 ⇒ interpolation-filled, unreliable).
inline void sampleSurface(
  const std::vector<double> & S, const PolarGrid & grid,
  double x, double y, double & out_z, int & out_count)
{
  const double r = std::hypot(x, y);
  const double th = std::atan2(y, x);
  // np.searchsorted(edges, v) - 1, clipped
  const int ri = std::clamp(
    static_cast<int>(std::lower_bound(grid.r_edges.begin(), grid.r_edges.end(), r) -
    grid.r_edges.begin()) - 1, 0, grid.n_r - 1);
  const int ti = std::clamp(
    static_cast<int>(std::lower_bound(grid.th_edges.begin(), grid.th_edges.end(), th) -
    grid.th_edges.begin()) - 1, 0, grid.n_th - 1);
  const int cell = ri * grid.n_th + ti;
  out_z = S[cell];
  out_count = grid.count[cell];
}

// Port of _adaptive_threshold(). Obstacle height threshold grows with range.
inline double adaptiveThreshold(
  double r, double th_near = 0.15, double th_far = 0.40,
  double r_near = 5.0, double r_far = 30.0)
{
  const double t = std::clamp((r - r_near) / (r_far - r_near), 0.0, 1.0);
  return th_near + t * (th_far - th_near);
}

// ======================================================================
// Layer 2 — Residual analysis + classification
// ======================================================================

// Port of _cluster_residual_pts(). 2D grid connected components (8-neighbour).
inline std::vector<Cloud> clusterResidualPts(
  const Cloud & xyz, const std::vector<double> & residuals,
  const std::vector<double> & threshold, int min_pts = 5,
  double grid_res = 0.2)
{
  std::vector<Cloud> clusters;
  std::vector<int> flagged;
  flagged.reserve(xyz.size());
  for (size_t i = 0; i < xyz.size(); ++i) {
    if (residuals[i] > threshold[i]) {flagged.push_back(static_cast<int>(i));}
  }
  if (static_cast<int>(flagged.size()) < min_pts) {return clusters;}

  // pack (ix, iy) into one 64-bit key
  auto key_of = [](int ix, int iy) -> int64_t {
      return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
    };

  std::unordered_map<int64_t, std::vector<int>> cells;
  cells.reserve(flagged.size());
  for (int fi : flagged) {
    const int ix = static_cast<int>(std::floor(xyz[fi].x() / grid_res));
    const int iy = static_cast<int>(std::floor(xyz[fi].y() / grid_res));
    cells[key_of(ix, iy)].push_back(fi);
  }

  std::unordered_map<int64_t, bool> visited;
  visited.reserve(cells.size());
  const int offs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

  for (const auto & kv : cells) {
    if (visited[kv.first]) {continue;}
    visited[kv.first] = true;

    std::vector<int64_t> stack{kv.first}, comp{kv.first};
    while (!stack.empty()) {
      const int64_t cur = stack.back();
      stack.pop_back();
      const int cx = static_cast<int>(cur >> 32);
      const int cy = static_cast<int>(static_cast<uint32_t>(cur & 0xFFFFFFFF));
      for (const auto & o : offs) {
        const int64_t nb = key_of(cx + o[0], cy + o[1]);
        if (cells.count(nb) && !visited[nb]) {
          visited[nb] = true;
          stack.push_back(nb);
          comp.push_back(nb);
        }
      }
    }

    Cloud cluster;
    for (int64_t c : comp) {
      for (int pi : cells[c]) {cluster.push_back(xyz[pi]);}
    }
    if (static_cast<int>(cluster.size()) >= min_pts) {clusters.push_back(std::move(cluster));}
  }
  return clusters;
}

struct Classification
{
  int type_id = TYPE_UNKNOWN;
  std::string label;
  Point3 centroid = Point3::Zero();
  Point3 dims = Point3::Zero();
  double verticality = 0.0;
  double edge_ratio = 0.0;
};

// Port of _classify_surface(). Geometry-based classification of one cluster.
inline Classification classifySurface(const Cloud & pts, double ground_z_mean)
{
  Classification out;
  const int n = static_cast<int>(pts.size());
  if (n == 0) {return out;}

  Point3 mn = pts[0], mx = pts[0], sum = Point3::Zero();
  for (const auto & p : pts) {
    mn = mn.cwiseMin(p);
    mx = mx.cwiseMax(p);
    sum += p;
  }
  const Point3 dims = mx - mn;
  // 质心用点云均值: 16线只命中物体部分表面时，包围盒中心偏向命中点集的
  // 几何中心，均值更接近物体真实中心。
  const Point3 c = sum / static_cast<double>(n);
  const double H = dims.z();

  const double dist = std::hypot(c.x(), c.y());
  const double dist_factor = std::clamp((dist - 5.0) / (25.0 - 5.0), 0.0, 1.0);
  const double pole_width_max = 0.5 + dist_factor * 0.3;      // 0.5m@5m → 0.8m@25m
  const int min_pts_small = static_cast<int>(10 - dist_factor * 5);  // 10@5m → 5@25m

  // PCA via covariance eigendecomposition (Eigen) — equivalent to the SVD the
  // Python version runs on the centred points; eigenvalues are the squared
  // singular values up to a constant factor, which cancels in the ratios below.
  double verticality = 0.0, slope_deg = 0.0, l2 = 0.0;
  if (n >= 5) {
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto & p : pts) {
      const Point3 d = p - c;
      cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() == Eigen::Success) {
      // eigenvalues ascending → descending for lambda ordering
      Eigen::Vector3d ev = solver.eigenvalues();
      const double ls = ev.sum();
      if (ls > 1e-12) {
        l2 = ev(0) / ls;
        // smallest-eigenvalue eigenvector = surface normal; sign is arbitrary so
        // take |z| (the Python port does the same via abs()).
        const double nz = std::abs(solver.eigenvectors().col(0).z());
        verticality = 1.0 - nz;
        slope_deg = std::acos(std::clamp(nz, -1.0, 1.0)) * 180.0 / M_PI;
      }
    }
  }

  const double W = std::max(dims.x(), dims.y());
  const double W_min = std::min(dims.x(), dims.y());
  const double curvature = l2;
  const double rel_elev = mn.z() - ground_z_mean;

  // edge ratio: share of points near the cluster's 2D bounding box border
  constexpr double kEdgeBand = 0.1;
  int on_edge = 0;
  for (const auto & p : pts) {
    if (p.x() - mn.x() < kEdgeBand || mx.x() - p.x() < kEdgeBand ||
      p.y() - mn.y() < kEdgeBand || mx.y() - p.y() < kEdgeBand)
    {
      ++on_edge;
    }
  }
  const double edge_ratio = static_cast<double>(on_edge) / std::max(1, n);

  out.centroid = c;
  out.dims = dims;
  out.verticality = verticality;
  out.edge_ratio = edge_ratio;

  char buf[128];
  // 物体高度估计: LiDAR只命中顶面时包围盒高度H偏小(≈顶面厚度)，此时离地高度
  // rel_elev准确；命中完整侧面时H准确。取两者较大值兼顾两种情形。
  const double height_est = std::max(rel_elev, H);

  // 1. 宽大坡面 → 可通过（提前判定，避免高坡被误判为障碍物）
  if (W > 4.0) {
    snprintf(buf, sizeof(buf), "passable_slope_big_W%.1fm", W);
    out.type_id = TYPE_PASSABLE_LOW; out.label = buf; return out;
  }
  // 2. 不可通过障碍物（高 + 紧凑 + 悬空/垂直）。compactness约束(W<2.5)区分
  //    "有限尺寸凸起(箱体/电线杆)"和"连续地形(缓坡/起伏路)"。
  if (height_est > 0.5 && W < 2.5 && (rel_elev > 0.25 || verticality > 0.7)) {
    snprintf(buf, sizeof(buf), "obstacle_H%.1fm_d%.0fm", height_est, dist);
    out.type_id = TYPE_OBSTACLE; out.label = buf; return out;
  }
  // 3. 需减速通过（波浪路：点多、平缓）
  if (n > 30 && curvature < 0.01 && H < 1.0) {
    snprintf(buf, sizeof(buf), "wave_L%.1fm", W);
    out.type_id = TYPE_PASSABLE_HIGH; out.label = buf; return out;
  }
  // 4. 可通过：缓坡
  if (slope_deg < 20.0 && height_est < 2.0) {
    snprintf(buf, sizeof(buf), "passable_slope%.0fdeg", slope_deg);
    out.type_id = TYPE_PASSABLE_LOW; out.label = buf; return out;
  }
  // 5. 可通过：细杆、减速带、矮障碍物
  if (height_est < 0.5) {
    if (verticality > 0.7 && W_min < pole_width_max) {
      snprintf(buf, sizeof(buf), "passable_pole_H%.2fm", height_est);
    } else {
      snprintf(buf, sizeof(buf), "passable_bump_H%.2fm", height_est);
    }
    out.type_id = TYPE_PASSABLE_LOW; out.label = buf; return out;
  }
  // 6. 远处稀疏小簇 → 未知
  if (dist > 15.0 && n < min_pts_small) {
    snprintf(buf, sizeof(buf), "unknown_sparse_d%.0fm", dist);
    out.type_id = TYPE_UNKNOWN; out.label = buf; return out;
  }
  // 7. 默认：可通过的粗糙地形
  snprintf(buf, sizeof(buf), "passable_rough_H%.2fm", height_est);
  out.type_id = TYPE_PASSABLE_LOW; out.label = buf;
  return out;
}

// Port of _confidence_surface().
inline double confidenceSurface(
  int n_pts, double verticality, double edge_ratio,
  const std::vector<int> & track_hist)
{
  double s = 0.0;
  if (n_pts >= 30) {s += 0.3;} else if (n_pts >= 20) {s += 0.2;} else if (n_pts >= 10) {s += 0.1;}
  s += (verticality > 0.8 || verticality < 0.1) ? 0.2 : 0.1;
  s += (edge_ratio > 0.5 || edge_ratio < 0.1) ? 0.2 : 0.0;
  if (track_hist.size() >= 3) {
    std::map<int, int> counts;
    for (int t : track_hist) {++counts[t];}
    int best = 0;
    for (const auto & kv : counts) {best = std::max(best, kv.second);}
    s += 0.3 * static_cast<double>(best) / static_cast<double>(track_hist.size());
  }
  return std::min(1.0, s);
}

// ======================================================================
// Pothole detection
// ======================================================================

struct Pothole
{
  double x, y, depth;
};

// Port of _detect_potholes(). Local Z anomaly in the ground cloud.
inline std::vector<Pothole> detectPotholes(
  const Cloud & ground, double grid_res = 0.2,
  double depth_thr = 0.08, int min_pts = 3)
{
  std::vector<Pothole> out;
  if (ground.size() < 20) {return out;}

  auto key_of = [](int ix, int iy) -> int64_t {
      return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
    };

  std::unordered_map<int64_t, std::pair<double, int>> acc;  // key → (sum z, count)
  for (const auto & p : ground) {
    const int ix = static_cast<int>(std::floor(p.x() / grid_res));
    const int iy = static_cast<int>(std::floor(p.y() / grid_res));
    auto & a = acc[key_of(ix, iy)];
    a.first += p.z();
    a.second += 1;
  }

  std::unordered_map<int64_t, double> mean_z;
  mean_z.reserve(acc.size());
  for (const auto & kv : acc) {
    if (kv.second.second >= min_pts) {
      mean_z[kv.first] = kv.second.first / kv.second.second;
    }
  }

  const int offs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (const auto & kv : mean_z) {
    const int cx = static_cast<int>(kv.first >> 32);
    const int cy = static_cast<int>(static_cast<uint32_t>(kv.first & 0xFFFFFFFF));
    double deepest = 0.0;
    for (const auto & o : offs) {
      const auto it = mean_z.find(key_of(cx + o[0], cy + o[1]));
      const double nz = (it != mean_z.end()) ? it->second : kv.second;
      deepest = std::max(deepest, nz - kv.second);
    }
    if (deepest > depth_thr) {
      out.push_back({cx * grid_res + grid_res / 2, cy * grid_res + grid_res / 2, deepest});
    }
  }
  return out;
}

// ======================================================================
// Temporal tracking
// ======================================================================

struct Track
{
  Point3 centroid = Point3::Zero();
  std::vector<int> hist;   // recent type ids (bounded by track_hist)
  int type_id = TYPE_UNKNOWN;
  std::string label;
  int lost = 0;
  bool matched = false;
};

// Port of _match_tracks(). Greedy nearest-neighbour matching + majority vote.
class Tracker
{
public:
  // Returns, per input cluster, the voted (type_id, label) and its track history.
  struct Result
  {
    int type_id;
    std::string label;
    std::vector<int> hist;
  };

  std::vector<Result> match(
    const std::vector<Point3> & cents, const std::vector<int> & tids,
    const std::vector<std::string> & labels,
    double dist_thr, int hist_size, int max_lost)
  {
    for (auto & kv : tracks_) {kv.second.matched = false;}

    std::map<int, int> assign;              // cluster index → track id
    std::vector<bool> claimed(cents.size(), false);

    if (!tracks_.empty()) {
      for (size_t ci = 0; ci < cents.size(); ++ci) {
        int best_id = -1;
        double best_d = std::numeric_limits<double>::max();
        for (const auto & kv : tracks_) {
          const double d = (kv.second.centroid - cents[ci]).norm();
          if (d < best_d) {best_d = d; best_id = kv.first;}
        }
        if (best_id >= 0 && best_d < dist_thr && !tracks_[best_id].matched) {
          assign[static_cast<int>(ci)] = best_id;
          tracks_[best_id].matched = true;
          claimed[ci] = true;
        }
      }
    }

    // update matched tracks
    for (const auto & kv : assign) {
      Track & t = tracks_[kv.second];
      t.centroid = cents[kv.first];
      t.hist.push_back(tids[kv.first]);
      if (static_cast<int>(t.hist.size()) > hist_size) {t.hist.erase(t.hist.begin());}
      t.label = labels[kv.first];
      std::map<int, int> counts;
      for (int h : t.hist) {++counts[h];}
      int best_cnt = -1;
      for (const auto & c : counts) {
        if (c.second > best_cnt) {best_cnt = c.second; t.type_id = c.first;}
      }
      t.lost = 0;
    }

    // spawn tracks for unmatched clusters
    for (size_t ci = 0; ci < cents.size(); ++ci) {
      if (claimed[ci]) {continue;}
      const int id = next_id_++;
      Track t;
      t.centroid = cents[ci];
      t.hist = {tids[ci]};
      t.type_id = tids[ci];
      t.label = labels[ci];
      t.matched = true;
      tracks_[id] = t;
      assign[static_cast<int>(ci)] = id;
    }

    // age out unmatched tracks
    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
      if (!it->second.matched) {
        if (++it->second.lost > max_lost) {it = tracks_.erase(it); continue;}
      }
      ++it;
    }

    std::vector<Result> out(cents.size());
    for (size_t ci = 0; ci < cents.size(); ++ci) {
      const auto it = assign.find(static_cast<int>(ci));
      if (it != assign.end() && tracks_.count(it->second)) {
        const Track & t = tracks_[it->second];
        out[ci] = {t.type_id, t.label, t.hist};
      } else {
        out[ci] = {tids[ci], labels[ci], {}};
      }
    }
    return out;
  }

private:
  std::map<int, Track> tracks_;
  int next_id_ = 1;
};

}  // namespace lidar3d

#endif  // LIDAR3D_PERCEPTION_CPP__SURFACE_DETECTOR_HPP_
