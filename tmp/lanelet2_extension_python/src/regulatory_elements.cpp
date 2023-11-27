#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_extension/regulatory_elements/no_parking_area.hpp>
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
}
