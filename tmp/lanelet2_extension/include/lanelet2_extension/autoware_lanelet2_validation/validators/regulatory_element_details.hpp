#include <lanelet2_extension/autoware_lanelet2_validation/utils.hpp>
#include <lanelet2_extension/regulatory_elements/crosswalk.hpp>
#include <range/v3/view/filter.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_validation/Validation.h>

#include <set>

namespace lanelet
{
namespace validation
{

// TODO: update description
//! This check looks for points within linestrings or polygons that appear two times in succession.
//! These are not allowed because they often confuse geometry algorithms.
class RegulatoryElementDetailsChecker : public lanelet::validation::MapValidator
{
public:
  constexpr static const char * name() { return "mapping.regulatory_elements_details"; }

  lanelet::validation::Issues operator()(const lanelet::LaneletMap & map) override;

private:
  lanelet::validation::Issues checkRegulatoryElementsOfTrafficLight(
    const lanelet::LaneletMap & map);
  lanelet::validation::Issues checkRegulatoryElementsOfCrosswalk(const lanelet::LaneletMap & map);
  template <typename T>
  void checkPrimitiveType(
    std::vector<T> & in_vec, const std::string & expected_type, const std::string & message,
    lanelet::validation::Issues & issues);
  template <typename T>
  void checkPrimitiveType(
    std::vector<T> & in_vec, const std::string & expected_type,
    const std::string & expected_subtype, const std::string & message,
    lanelet::validation::Issues & issues);

  std::set<lanelet::Id> tl_elem_with_cw_;
};
}  // namespace validation
}  // namespace lanelet
