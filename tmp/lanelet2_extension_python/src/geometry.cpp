#include <boost/geometry/algorithms/area.hpp>
#include <boost/python.hpp>

#include <lanelet2_core/geometry/Polygon.h>

using namespace lanelet;

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME)
{
  using namespace boost::python;
  // NOTE: should be added to Lanelet2 mainstream
  // missing method of lanelet2_python/python_api/geometry.cpp
  // def("area", boost::geometry::area<BasicPolygon2d>);
  def("area", boost::geometry::area<ConstPolygon2d>);
  // def("area", boost::geometry::area<ConstHybridPolygon2d>);
  def("area", boost::geometry::area<CompoundPolygon2d>);
}
