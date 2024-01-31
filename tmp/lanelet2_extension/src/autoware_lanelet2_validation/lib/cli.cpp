#include "lanelet2_extension/autoware_lanelet2_validation/cli.hpp"

namespace po = boost::program_options;

namespace lanelet
{
namespace autoware
{
namespace validation
{

MetaConfig parseCommandLine(int argc, const char * argv[])
{
  MetaConfig config;
  auto & validation_config = config.command_line_config.validationConfig;
  po::options_description desc(
    "Runs a set of validators on a map. Think of it like a linter. The following checks are "
    "available:");
  desc.add_options()("help,h", "this help message")

    ("map_file", po::value<std::string>(), "Path to the map to be validated")

      ("filter,f", po::value(&validation_config.checksFilter),
       "Comma separated list of regexes to filter the applicable tests. Will run all tests by "
       "default. Example: "
       "routing_graph.* to run all checks for the routing graph")

        ("projector,p", po::value(&config.projector_type)->composing(),
         "Participants for which the routing graph will be instanciated (default: vehicle)")

          ("location,l",
           po::value(&validation_config.location)->default_value(validation_config.location),
           "Location of the map (for instanciating the traffic rules), e.g. de for Germany")

            ("participants", po::value(&validation_config.participants)->composing(),
             "Participants for which the routing graph will be instanciated (default: vehicle)")

              ("lat",
               po::value(&validation_config.origin.lat)
                 ->default_value(validation_config.origin.lat),
               "latitude coordinate of map origin")

                ("lon",
                 po::value(&validation_config.origin.lon)
                   ->default_value(validation_config.origin.lon),
                 "longitude coofdinate of map origin")

                  ("print", "Only print the checks that will be run, but dont run them");
  po::variables_map vm;
  po::positional_options_description pos;
  pos.add("map_file", 1);
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  po::notify(vm);
  config.command_line_config.help = vm.count("help") != 0;
  config.command_line_config.print = vm.count("print") != 0;
  if (vm.count("map_file") != 0) {
    config.command_line_config.mapFile =
      vm["map_file"].as<decltype(config.command_line_config.mapFile)>();
  }
  if (config.command_line_config.help) {
    std::cout << '\n' << desc;
  } else if (config.command_line_config.mapFile.empty() && !config.command_line_config.print) {
    std::cout << "Please pass either a valid file or '--print' or '--help'!\n";
  }
  return config;
}

}  // namespace validation
}  // namespace autoware
}  // namespace lanelet
