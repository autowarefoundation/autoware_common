#include <lanelet2_validation/Validation.h>
#include <lanelet2_validation/ValidatorFactory.h>

namespace lanelet
{
namespace validation
{
std::vector<std::regex> parseFilterString(const std::string & str);

void appendIssues(
  std::vector<lanelet::validation::DetectedIssues> & to,
  std::vector<lanelet::validation::DetectedIssues> && from);

void appendIssues(lanelet::validation::Issues & to, lanelet::validation::Issues && from);

}  // namespace validation
}  // namespace lanelet
