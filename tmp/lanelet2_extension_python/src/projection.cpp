#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/projection/transverse_mercator_projector.hpp>

#include <boost/python.hpp>

#include <lanelet2_io/Projection.h>

#include <memory>

using namespace lanelet;

BOOST_PYTHON_MODULE(_lanelet2_extension_python_boost_python_projection)
{
  namespace bp = boost::python;

  bp::class_<Projector, boost::noncopyable, std::shared_ptr<Projector>>(
    "Projector", "Projects point from lat/lon to x/y and back", bp::no_init)
    .def("forward", &Projector::forward, "Convert lat/lon into x/y")
    .def("reverse", &Projector::reverse, "Convert x/y into lat/lon")
    .def(
      "origin", &Projector::origin, "Global origin of the converter",
      bp::return_internal_reference<>());
  bp::class_<
    lanelet::projection::MGRSProjector, std::shared_ptr<lanelet::projection::MGRSProjector>,
    bp::bases<Projector>>("MGRSProjector", bp::init<Origin>("origin"));
  bp::class_<
    lanelet::projection::TransverseMercatorProjector,
    std::shared_ptr<lanelet::projection::TransverseMercatorProjector>, bp::bases<Projector>>(
    "TransverseMercatorProjector", bp::init<Origin>("origin"));
}
