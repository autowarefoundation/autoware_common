#include <lanelet2_extension/autoware_lanelet2_validation/utils.hpp>

namespace lanelet
{
namespace validation
{

void appendIssues(
  std::vector<lanelet::validation::DetectedIssues> & to,
  std::vector<lanelet::validation::DetectedIssues> && from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}
void appendIssues(lanelet::validation::Issues & to, lanelet::validation::Issues && from)
{
  to.insert(to.end(), from.begin(), from.end());
}

}  // namespace validation
}  // namespace lanelet
