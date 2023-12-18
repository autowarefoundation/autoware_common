#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_extension/regulatory_elements/no_parking_area.hpp>
#include <lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>

#include <boost/python.hpp>

using namespace boost::python;

// from lanelet2_python/src/core.cpp

void formatHelper([[maybe_unused]] std::ostream & os)
{
}

template <typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void formatHelper(std::ostream & os, const std::string & s, const Args &... Args_)
{
  if (!s.empty()) {
    os << ", ";
  }
  os << s;
  formatHelper(os, Args_...);
}

template <typename T, typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void formatHelper(std::ostream & os, const T & next, const Args &... Args_)
{
  os << ", ";
  os << next;
  formatHelper(os, Args_...);
}

template <typename T, typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
void format(std::ostream & os, const T & first, const Args &... Args_)
{
  os << first;
  formatHelper(os, Args_...);
}

template <typename... Args>
// NOLINTNEXTLINE(readability-identifier-naming)
std::string makeRepr(const char * name, const Args &... Args_)
{
  std::ostringstream os;
  os << name << '(';
  format(os, Args_...);
  os << ')';
  return os.str();
}

std::string repr(const object & o)
{
  object repr = import("builtins").attr("repr");
  return call<std::string>(repr.ptr(), o);
}

std::string repr(const lanelet::AttributeMap & a)
{
  if (a.empty()) {
    return {};
  }
  return repr(object(a));
}

std::string repr(const lanelet::RegulatoryElementConstPtrs & regelems)
{
  if (regelems.empty()) {
    return {};
  }
  return repr(list(regelems));
}

BOOST_PYTHON_MODULE(_lanelet2_extension_python_boost_python_regulatory_elements)
{
  // autoware_traffic_light
  class_<
    lanelet::autoware::AutowareTrafficLight, boost::noncopyable,
    std::shared_ptr<lanelet::autoware::AutowareTrafficLight>, bases<lanelet::TrafficLight>>(
    "AutowareTrafficLight", "autoware traffic light regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::AutowareTrafficLight::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("trafficLights"),
                     arg("stopline") = lanelet::Optional<lanelet::LineString3d>(),
                     arg("lightBulbs") = lanelet::LineString3d({}))))
    .def(
      "lightBulbs",
      +[](lanelet::autoware::AutowareTrafficLight & self) { return self.lightBulbs(); })
    .def("addLightBulbs", &lanelet::autoware::AutowareTrafficLight::addLightBulbs)
    .def("removeLightBulbs", &lanelet::autoware::AutowareTrafficLight::removeLightBulbs)
    .def(
      "__repr__", +[](lanelet::autoware::AutowareTrafficLight & r) {
        return makeRepr(
          "AutowareTrafficLight", r.id(), repr(dict(r.constData()->parameters)),
          repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::AutowareTrafficLight>, lanelet::RegulatoryElementPtr>();

  // crosswalk
  class_<
    lanelet::autoware::Crosswalk, boost::noncopyable, std::shared_ptr<lanelet::autoware::Crosswalk>,
    bases<lanelet::RegulatoryElement>>(
    "Crosswalk", "autoware crosswalk regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::Crosswalk::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("crosswalk_lanelet"), arg("crosswalk_area"),
                     arg("stop_line"))))
    .def(
      "crosswalkAreas", +[](lanelet::autoware::Crosswalk & self) { return self.crosswalkAreas(); })
    .def(
      "stopLines", +[](lanelet::autoware::Crosswalk & self) { return self.stopLines(); })
    .def(
      "crosswalkLanelet",
      +[](lanelet::autoware::Crosswalk & self) { return self.crosswalkLanelet(); })
    .def("addCrosswalkArea", &lanelet::autoware::Crosswalk::addCrosswalkArea)
    .def("removeCrosswalkArea", &lanelet::autoware::Crosswalk::removeCrosswalkArea)
    .def(
      "__repr__", +[](lanelet::autoware::Crosswalk & r) {
        return makeRepr(
          "Crosswalk", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::Crosswalk>, lanelet::RegulatoryElementPtr>();

  // detection_area
  class_<
    lanelet::autoware::DetectionArea, boost::noncopyable,
    std::shared_ptr<lanelet::autoware::DetectionArea>, bases<lanelet::RegulatoryElement>>(
    "DetectionArea", "detection_area regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::DetectionArea::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("detectionAreas"), arg("stopLine"))))
    .def(
      "detectionAreas",
      +[](lanelet::autoware::DetectionArea & self) { return self.detectionAreas(); })
    .def("addDetectionArea", &lanelet::autoware::DetectionArea::addDetectionArea)
    .def("removeDetectionArea", &lanelet::autoware::DetectionArea::removeDetectionArea)
    .def(
      "stopLine", +[](lanelet::autoware::DetectionArea & self) { return self.stopLine(); })
    .def("setStopLine", &lanelet::autoware::DetectionArea::setStopLine)
    .def("removeStopLine", &lanelet::autoware::DetectionArea::removeStopLine)
    .def(
      "__repr__", +[](lanelet::autoware::DetectionArea & r) {
        return makeRepr(
          "DetectionArea", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::DetectionArea>, lanelet::RegulatoryElementPtr>();

  // no_parking_area
  class_<
    lanelet::autoware::NoParkingArea, boost::noncopyable,
    std::shared_ptr<lanelet::autoware::NoParkingArea>, bases<lanelet::RegulatoryElement>>(
    "NoParkingArea", "no_parking_area regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::NoParkingArea::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("no_parking_areas"))))
    .def(
      "noParkingAreas",
      +[](lanelet::autoware::NoParkingArea & self) { return self.noParkingAreas(); })
    .def("addNoParkingArea", &lanelet::autoware::NoParkingArea::addNoParkingArea)
    .def("removeNoParkingArea", &lanelet::autoware::NoParkingArea::removeNoParkingArea)
    .def(
      "__repr__", +[](lanelet::autoware::NoParkingArea & r) {
        return makeRepr(
          "NoParkingArea", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::NoParkingArea>, lanelet::RegulatoryElementPtr>();

  // no_stopping_area
  class_<
    lanelet::autoware::NoStoppingArea, boost::noncopyable,
    std::shared_ptr<lanelet::autoware::NoStoppingArea>, bases<lanelet::RegulatoryElement>>(
    "NoStoppingArea", "no_stopping_area regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::NoStoppingArea::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("no_stopping_areas"),
                     arg("stopLine") = lanelet::Optional<lanelet::LineString3d>())))
    .def(
      "noStoppingAreas",
      +[](lanelet::autoware::NoStoppingArea & self) { return self.noStoppingAreas(); })
    .def("addNoStoppingArea", &lanelet::autoware::NoStoppingArea::addNoStoppingArea)
    .def("removeNoStoppingArea", &lanelet::autoware::NoStoppingArea::removeNoStoppingArea)
    .def(
      "stopLine", +[](lanelet::autoware::NoStoppingArea & self) { return self.stopLine(); })
    .def("setStopLine", &lanelet::autoware::NoStoppingArea::setStopLine)
    .def("removeStopLine", &lanelet::autoware::NoStoppingArea::removeStopLine)
    .def(
      "__repr__", +[](lanelet::autoware::NoParkingArea & r) {
        return makeRepr(
          "NoParkingArea", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::NoStoppingArea>, lanelet::RegulatoryElementPtr>();

  // road_marking
  class_<
    lanelet::autoware::RoadMarking, boost::noncopyable,
    std::shared_ptr<lanelet::autoware::RoadMarking>, bases<lanelet::RegulatoryElement>>(
    "RoadMarking", "road_marking regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::RoadMarking::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("road_maring"))))
    .def(
      "roadMarking", +[](lanelet::autoware::RoadMarking & self) { return self.roadMarking(); })
    .def("setRoadMarking", &lanelet::autoware::RoadMarking::setRoadMarking)
    .def("removeRoadMarking", &lanelet::autoware::RoadMarking::removeRoadMarking)
    .def(
      "__repr__", +[](lanelet::autoware::RoadMarking & r) {
        return makeRepr(
          "RoadMarking", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::RoadMarking>, lanelet::RegulatoryElementPtr>();

  // speed_bump
  class_<
    lanelet::autoware::SpeedBump, boost::noncopyable, std::shared_ptr<lanelet::autoware::SpeedBump>,
    bases<lanelet::RegulatoryElement>>("SpeedBump", "speed_bump regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::SpeedBump::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("speed_bump"))))
    .def(
      "speedBump", +[](lanelet::autoware::SpeedBump & self) { return self.speedBump(); })
    .def("addSpeedBump", &lanelet::autoware::SpeedBump::addSpeedBump)
    .def("removeSpeedBump", &lanelet::autoware::SpeedBump::removeSpeedBump)
    .def(
      "__repr__", +[](lanelet::autoware::SpeedBump & r) {
        return makeRepr(
          "SpeedBump", r.id(), repr(dict(r.constData()->parameters)), repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::SpeedBump>, lanelet::RegulatoryElementPtr>();

  // virtual_traffic_light
  class_<
    lanelet::autoware::VirtualTrafficLight, boost::noncopyable,
    std::shared_ptr<lanelet::autoware::VirtualTrafficLight>, bases<lanelet::RegulatoryElement>>(
    "VirtualTrafficLight", "virtual_traffic_light regulatory element", no_init)
    .def(
      "__init__", make_constructor(
                    &lanelet::autoware::VirtualTrafficLight::make, default_call_policies(),
                    (arg("Id"), arg("attributes"), arg("virtual_traffic_light"))))
    .def(
      "getVirtualTrafficLight",
      +[](lanelet::autoware::VirtualTrafficLight & self) { return self.getVirtualTrafficLight(); })
    .def(
      "getStopLine",
      +[](lanelet::autoware::VirtualTrafficLight & self) { return self.getStopLine(); })
    .def(
      "getStartLine",
      +[](lanelet::autoware::VirtualTrafficLight & self) { return self.getStartLine(); })
    .def(
      "getEndLine",
      +[](lanelet::autoware::VirtualTrafficLight & self) { return self.getEndLine(); })
    .def(
      "__repr__", +[](lanelet::autoware::VirtualTrafficLight & r) {
        return makeRepr(
          "VirtualTrafficLight", r.id(), repr(dict(r.constData()->parameters)),
          repr(r.attributes()));
      });
  implicitly_convertible<
    std::shared_ptr<lanelet::autoware::VirtualTrafficLight>, lanelet::RegulatoryElementPtr>();
}
