#include "lanelet2_extension/autoware_lanelet2_validation/cli.hpp"

#include <lanelet2_extension/autoware_lanelet2_validation/utils.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/projection/transverse_mercator_projector.hpp>

#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_validation/Cli.h>
#include <lanelet2_validation/Validation.h>

#include <regex>

namespace
{
namespace projector_names
{
constexpr const char * transverse_mercator = "tm";
constexpr const char * mgrs = "mgrs";
constexpr const char * utm = "utm";
constexpr const char * local = "local";
}  // namespace projector_names
}  // namespace

namespace lanelet
{
namespace autoware
{
namespace validation
{
std::unique_ptr<lanelet::Projector> getProjector(const MetaConfig & config);
std::vector<lanelet::validation::DetectedIssues> validateMap(const MetaConfig & config);
}  // namespace validation
}  // namespace autoware
}  // namespace lanelet
