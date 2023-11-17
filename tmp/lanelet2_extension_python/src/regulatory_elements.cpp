#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_extension/regulatory_elements/no_parking_area.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>

#include <boost/python.hpp>

BOOST_PYTHON_MODULE(_lanelet2_extension_python_boost_python_regulatory_elements)
{
  using namespace boost::python;
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
    .def("removeCrosswalkArea", &lanelet::autoware::Crosswalk::removeCrosswalkArea);
}
