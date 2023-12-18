#include <boost/geometry/algorithms/area.hpp>
#include <boost/python.hpp>

#include <lanelet2_core/geometry/Polygon.h>

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME)
{
  namespace bp = boost::python;
  // NOTE: should be added to Lanelet2 mainstream
  // missing method of lanelet2_python/python_api/geometry.cpp
  // def("area", boost::geometry::area<BasicPolygon2d>);
  bp::def("area", boost::geometry::area<lanelet::ConstPolygon2d>);
  // def("area", boost::geometry::area<ConstHybridPolygon2d>);
  bp::def("area", boost::geometry::area<lanelet::CompoundPolygon2d>);
}
