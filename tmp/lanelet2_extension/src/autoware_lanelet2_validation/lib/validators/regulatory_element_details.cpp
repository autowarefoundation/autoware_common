#include "lanelet2_extension/autoware_lanelet2_validation/validators/regulatory_element_details.hpp"

namespace lanelet
{
namespace validation
{
namespace
{
lanelet::validation::RegisterMapValidator<RegulatoryElementDetailsChecker> reg;
}  // namespace

lanelet::validation::Issues RegulatoryElementDetailsChecker::operator()(
  const lanelet::LaneletMap & map)
{
  // All issues found by all validators
  lanelet::validation::Issues issues;

  // Append issues found by each validator
  appendIssues(issues, checkRegulatoryElementsOfTrafficLight(map));
  appendIssues(issues, checkRegulatoryElementsOfCrosswalk(map));
  return issues;
}

template <typename T>
void RegulatoryElementDetailsChecker::checkPrimitiveType(
  std::vector<T> & in_vec, const std::string & expected_type, const std::string & message,
  lanelet::validation::Issues & issues)
{
  for (auto iter = in_vec.begin(); iter != in_vec.end(); ++iter) {
    const auto & item = *iter;
    const auto & attrs = item.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Type);
    if (it == attrs.end() || it->second != expected_type) {
      // Report warning if crosswalk polygon does not have type of crosswalk_polygon
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        item.id(), message);
      const auto new_it = in_vec.erase(iter);
      if (new_it != in_vec.end()) {
        iter = new_it;
      } else {
        break;
      }
    }
  }
}

template <typename T>
void RegulatoryElementDetailsChecker::checkPrimitiveType(
  std::vector<T> & in_vec, const std::string & expected_type, const std::string & expected_subtype,
  const std::string & message, lanelet::validation::Issues & issues)
{
  for (auto iter = in_vec.begin(); iter != in_vec.end(); ++iter) {
    const auto & item = *iter;
    const auto & attrs = item.attributes();
    const auto & it = attrs.find(lanelet::AttributeName::Type);
    const auto & it_sub = attrs.find(lanelet::AttributeName::Subtype);
    if (
      it == attrs.end() || it->second != expected_type || it_sub == attrs.end() ||
      it_sub->second != expected_subtype) {
      // Report warning if crosswalk polygon does not have type of crosswalk_polygon
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        item.id(), message);
      const auto new_it = in_vec.erase(iter);
      if (new_it != in_vec.end()) {
        iter = new_it;
      } else {
        break;
      }
    }
  }
}

lanelet::validation::Issues RegulatoryElementDetailsChecker::checkRegulatoryElementsOfTrafficLight(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;
  // filter regulatory element whose Subtype is traffic light
  auto elems =
    map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
      const auto & attrs = elem->attributes();
      const auto & it = attrs.find(lanelet::AttributeName::Subtype);
      return it != attrs.end() && it->second == lanelet::AttributeValueString::TrafficLight;
    });

  for (const auto & elem : elems) {
    // Get line strings of traffic light referred by regulatory element
    auto refers = elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::Refers);
    // Get stop line referred by regulatory element
    auto ref_lines = elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);

    checkPrimitiveType(
      refers, lanelet::AttributeValueString::TrafficLight,
      "Refers of traffic light regulatory element must have type of traffic_light", issues);
    checkPrimitiveType(
      ref_lines, lanelet::AttributeValueString::StopLine,
      "Refline of traffic light regulatory element must have type of stop_line", issues);

    if (refers.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of traffic light must have a traffic light(refers).");
    }
    // TODO: Check correct behavior if regulatory element has two or more traffic light
    //  else if (refers.size() != 1) {
    //   issues.emplace_back(
    //     lanelet::validation::Severity::Error,
    //     lanelet::validation::Primitive::RegulatoryElement, elem->id(), "Regulatory element of
    //     traffic light must have only one traffic light(refers).");
    // }

    // Report error if regulatory element does not have stop line and crosswalk
    if (ref_lines.empty() && tl_elem_with_cw_.find(elem->id()) == tl_elem_with_cw_.end()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of traffic light must have a stop line(ref_line).");
    }
  }
  return issues;
}

lanelet::validation::Issues RegulatoryElementDetailsChecker::checkRegulatoryElementsOfCrosswalk(
  const lanelet::LaneletMap & map)
{
  lanelet::validation::Issues issues;
  // filter elem whose Subtype is crosswalk and has crosswalk polygon
  auto elems = map.regulatoryElementLayer | ranges::views::filter([](auto && elem) {
                 const auto & attrs = elem->attributes();
                 const auto & it = attrs.find(lanelet::AttributeName::Subtype);
                 return it != attrs.end() && it->second == lanelet::AttributeValueString::Crosswalk;
               });

  for (const auto & elem : elems) {
    // Get lanelet of crosswalk referred by regulatory element
    auto refers = elem->getParameters<lanelet::ConstLanelet>(lanelet::RoleName::Refers);
    // Get stop line referred by regulatory element
    auto ref_lines = elem->getParameters<lanelet::ConstLineString3d>(lanelet::RoleName::RefLine);
    // Get crosswalk polygon referred by regulatory element
    auto crosswalk_polygons = elem->getParameters<lanelet::ConstPolygon3d>(
      lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon);

    checkPrimitiveType(
      refers, lanelet::AttributeValueString::Lanelet, lanelet::AttributeValueString::Crosswalk,
      "Refers of crosswalk regulatory element must have type of crosswalk", issues);
    checkPrimitiveType(
      ref_lines, lanelet::AttributeValueString::StopLine,
      "Refline of crosswalk regulatory element must have type of stopline", issues);
    checkPrimitiveType(
      crosswalk_polygons, lanelet::autoware::Crosswalk::AutowareRoleNameString::CrosswalkPolygon,
      "Crosswalk polygon of crosswalk regulatory element must have type of Crosswalk_polygon",
      issues);

    // Report warning if regulatory element does not have crosswalk polygon
    if (crosswalk_polygons.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Warning, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk is nice to have crosswalk_polygon.");
    }
    // Report warning if regulatory element has two or more crosswalk polygon
    else if (crosswalk_polygons.size() != 1) {
      issues.emplace_back(
        lanelet::validation::Severity::Warning, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk must have only one crosswalk_polygon.");
    }
    // Report warning if regulatory element does not have stop line
    if (ref_lines.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Warning, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk is nice to have stop line(ref_line).");
    }
    // Report error if regulatory element does not have lanelet of crosswalk
    if (refers.empty()) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(), "Regulatory element of cross walk must have lanelet of crosswalk(refers).");
    }
    // Report error if regulatory element has two or more lanelet of crosswalk
    else if (refers.size() != 1) {
      issues.emplace_back(
        lanelet::validation::Severity::Error, lanelet::validation::Primitive::RegulatoryElement,
        elem->id(),
        "Regulatory element of cross walk must have only one lanelet of crosswalk(refers).");
    }
  }
  return issues;
}

}  // namespace validation
}  // namespace lanelet
