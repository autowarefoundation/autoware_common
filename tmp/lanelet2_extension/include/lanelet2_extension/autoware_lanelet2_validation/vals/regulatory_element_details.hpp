#include "lanelet2_extension/autoware_lanelet2_validation/utils.hpp"
#include "lanelet2_extension/regulatory_elements/crosswalk.hpp"

#include <range/v3/view/filter.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/Validation.h>

#include <set>

namespace lanelet
{
namespace validation
{

#ifndef LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__VALS__REGULATORY_ELEMENT_DETAILS_HPP_
#define LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__VALS__REGULATORY_ELEMENT_DETAILS_HPP_

// TODO: update description
//! This check looks for points within linestrings or polygons that appear two times in succession.
//! These are not allowed because they often confuse geometry algorithms.
class RegulatoryElementDetailsChecker : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.regulatory_elements_details"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  lanelet::validation::Issues checkRegulatoryElementOfTrafficLight(const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkRegulatoryElementOfCrosswalk(const lanelet::LaneletMap & map);

  std::set<lanelet::Id> tl_elem_with_cw_;
};
}  // namespace validation
}  // namespace lanelet

#endif  // LANELET2_EXTENSION__AUTOWARE_LANELET2_VALIDATION__VALS__REGULATORY_ELEMENT_DETAILS_HPP_
