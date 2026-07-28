#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "patchwork/patchwork.h"
#include "patchwork/patchworkpp.h"

namespace py = pybind11;

PYBIND11_MODULE(pypatchworkpp, m) {
  m.doc()               = "Python Patchwork++";
  m.attr("__version__") = "0.0.1";

  py::class_<patchwork::Params>(m, "Parameters")
      .def(py::init<>())
      .def_readwrite("sensor_height", &patchwork::Params::sensor_height)
      .def_readwrite("verbose", &patchwork::Params::verbose)
      .def_readwrite("enable_RNR", &patchwork::Params::enable_RNR)
      .def_readwrite("enable_RVPF", &patchwork::Params::enable_RVPF)
      .def_readwrite("enable_TGR", &patchwork::Params::enable_TGR)
      .def_readwrite("num_iter", &patchwork::Params::num_iter)
      .def_readwrite("num_lpr", &patchwork::Params::num_lpr)
      .def_readwrite("num_min_pts", &patchwork::Params::num_min_pts)
      .def_readwrite("num_zones", &patchwork::Params::num_zones)
      .def_readwrite("num_rings_of_interest", &patchwork::Params::num_rings_of_interest)
      .def_readwrite("RNR_ver_angle_thr", &patchwork::Params::RNR_ver_angle_thr)
      .def_readwrite("RNR_intensity_thr", &patchwork::Params::RNR_intensity_thr)
      .def_readwrite("sensor_height", &patchwork::Params::sensor_height)
      .def_readwrite("th_seeds", &patchwork::Params::th_seeds)
      .def_readwrite("th_dist", &patchwork::Params::th_dist)
      .def_readwrite("th_seeds_v", &patchwork::Params::th_seeds_v)
      .def_readwrite("th_dist_v", &patchwork::Params::th_dist_v)
      .def_readwrite("max_range", &patchwork::Params::max_range)
      .def_readwrite("min_range", &patchwork::Params::min_range)
      .def_readwrite("uprightness_thr", &patchwork::Params::uprightness_thr)
      .def_readwrite("adaptive_seed_selection_margin",
                     &patchwork::Params::adaptive_seed_selection_margin)
      .def_readwrite("intensity_thr", &patchwork::Params::intensity_thr)
      .def_readwrite("num_sectors_each_zone", &patchwork::Params::num_sectors_each_zone)
      .def_readwrite("num_rings_each_zone", &patchwork::Params::num_rings_each_zone)
      .def_readwrite("max_flatness_storage", &patchwork::Params::max_flatness_storage)
      .def_readwrite("max_elevation_storage", &patchwork::Params::max_elevation_storage)
      .def_readwrite("elevation_thr", &patchwork::Params::elevation_thr)
      .def_readwrite("flatness_thr", &patchwork::Params::flatness_thr);

  py::class_<patchwork::PatchWorkpp>(m, "patchworkpp")
      .def(py::init<patchwork::Params>())
      .def("getHeight", &patchwork::PatchWorkpp::getHeight)
      .def("getTimeTaken", &patchwork::PatchWorkpp::getTimeTaken)
      .def("getGround", &patchwork::PatchWorkpp::getGround)
      .def("getNonground", &patchwork::PatchWorkpp::getNonground)
      .def("getCenters", &patchwork::PatchWorkpp::getCenters)
      .def("getGroundIndices", &patchwork::PatchWorkpp::getGroundIndices)
      .def("getNongroundIndices", &patchwork::PatchWorkpp::getNongroundIndices)
      .def("getNormals", &patchwork::PatchWorkpp::getNormals)
      .def("estimateGround", &patchwork::PatchWorkpp::estimateGround);
  // .def_readwrite("sensor_height_", &PatchWorkpp::sensor_height_);

  py::class_<patchwork::PatchworkParams>(m, "PatchworkParams")
      .def(py::init<>())
      .def_readwrite("sensor_height", &patchwork::PatchworkParams::sensor_height)
      .def_readwrite("max_range", &patchwork::PatchworkParams::max_range)
      .def_readwrite("min_range", &patchwork::PatchworkParams::min_range)
      .def_readwrite("num_zones", &patchwork::PatchworkParams::num_zones)
      .def_readwrite("num_sectors_each_zone", &patchwork::PatchworkParams::num_sectors_each_zone)
      .def_readwrite("num_rings_each_zone", &patchwork::PatchworkParams::num_rings_each_zone)
      .def_readwrite("min_ranges", &patchwork::PatchworkParams::min_ranges)
      .def_readwrite("num_iter", &patchwork::PatchworkParams::num_iter)
      .def_readwrite("num_lpr", &patchwork::PatchworkParams::num_lpr)
      .def_readwrite("num_min_pts", &patchwork::PatchworkParams::num_min_pts)
      .def_readwrite("th_seeds", &patchwork::PatchworkParams::th_seeds)
      .def_readwrite("th_dist", &patchwork::PatchworkParams::th_dist)
      .def_readwrite("uprightness_thr", &patchwork::PatchworkParams::uprightness_thr)
      .def_readwrite("elevation_thr", &patchwork::PatchworkParams::elevation_thr)
      .def_readwrite("flatness_thr", &patchwork::PatchworkParams::flatness_thr)
      .def_readwrite("adaptive_seed_selection_margin",
                     &patchwork::PatchworkParams::adaptive_seed_selection_margin)
      .def_readwrite("using_global_thr", &patchwork::PatchworkParams::using_global_thr)
      .def_readwrite("global_elevation_thr", &patchwork::PatchworkParams::global_elevation_thr)
      .def_readwrite("ATAT_ON", &patchwork::PatchworkParams::ATAT_ON)
      .def_readwrite("max_h_for_ATAT", &patchwork::PatchworkParams::max_h_for_ATAT)
      .def_readwrite("num_sectors_for_ATAT", &patchwork::PatchworkParams::num_sectors_for_ATAT)
      .def_readwrite("noise_bound", &patchwork::PatchworkParams::noise_bound)
      .def_readwrite("verbose", &patchwork::PatchworkParams::verbose);

  py::class_<patchwork::PatchWork>(m, "patchwork")
      .def(py::init<patchwork::PatchworkParams>())
      .def("estimateGround", &patchwork::PatchWork::estimateGround)
      .def("getGround", &patchwork::PatchWork::getGround)
      .def("getNonground", &patchwork::PatchWork::getNonground)
      .def("getGroundIndices", &patchwork::PatchWork::getGroundIndices)
      .def("getNongroundIndices", &patchwork::PatchWork::getNongroundIndices)
      .def("getTimeTaken", &patchwork::PatchWork::getTimeTaken)
      .def("getHeight", &patchwork::PatchWork::getHeight);
}
