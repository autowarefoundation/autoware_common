// Copyright 2023 Autoware Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Mamoru Sobue

// NOLINTBEGIN(readability-identifier-naming)

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

// NOLINTEND(readability-identifier-naming)
