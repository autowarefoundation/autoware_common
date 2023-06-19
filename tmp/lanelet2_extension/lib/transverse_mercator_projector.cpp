// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
// Authors: Simon Thompson, Ryohsuke Mitsudome

// NOLINTBEGIN(readability-identifier-naming)

#include "lanelet2_extension/projection/transverse_mercator_projector.hpp"

#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace lanelet::projection
{
TransverseMercatorProjector::TransverseMercatorProjector(Origin origin) : Projector(origin)
{
  central_meridian_ = origin.position.lon;

  // Calculate origin in Transverse Mercator coordinate
  const GeographicLib::TransverseMercatorExact & proj =
    GeographicLib::TransverseMercatorExact::UTM();
  proj.Forward(central_meridian_, origin.position.lat, origin.position.lon, origin_x_, origin_y_);
}

BasicPoint3d TransverseMercatorProjector::forward(const GPSPoint & gps) const
{
  BasicPoint3d tm_point{0., 0., gps.ele};
  const GeographicLib::TransverseMercatorExact & proj =
    GeographicLib::TransverseMercatorExact::UTM();
  proj.Forward(central_meridian_, gps.lat, gps.lon, tm_point.x(), tm_point.y());
  tm_point.x() = tm_point.x() - origin_x_;
  tm_point.y() = tm_point.y() - origin_y_;
  std::cout << "KOJI632458!!!!!!!!!!!!!!!!!! " << tm_point.x() << ", " << tm_point.y() << std::endl;
  std::cout << "KOJI6534242534!!!!!!!!!!!!!!!!!! " << origin_x_ << ", " << origin_y_ << std::endl;

  return tm_point;
}

GPSPoint TransverseMercatorProjector::reverse(const BasicPoint3d & local_point) const
{
  GPSPoint gps{0.0, 0.0, local_point.z()};
  const GeographicLib::TransverseMercatorExact & proj =
    GeographicLib::TransverseMercatorExact::UTM();
  proj.Reverse(central_meridian_, local_point.x(), local_point.y(), gps.lat, gps.lon);
  return gps;
}

}  // namespace lanelet::projection

// NOLINTEND(readability-identifier-naming)
