#include <boost/program_options.hpp>

#include <lanelet2_validation/Cli.h>

#include <iostream>

namespace lanelet
{
namespace autoware
{
namespace validation
{
struct MetaConfig
{
  lanelet::validation::CommandLineConfig command_line_config;
  std::string projector_type;
};

MetaConfig parseCommandLine(int argc, const char * argv[]);

}  // namespace validation
}  // namespace autoware
}  // namespace lanelet
