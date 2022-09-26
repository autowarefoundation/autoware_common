#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <utility>
#include <vector>

using namespace std;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    cout << "usage: ros2 run lanelet2_extension check_row <map_path>";
    return 0;
  }

  const string map_path = string(argv[1]);

  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  lanelet::LaneletMapPtr map = lanelet::load(map_path, projector, &errors);
  for (auto && error : errors) cout << error << endl;

  lanelet::traffic_rules::TrafficRulesPtr trafficRules =
    lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphPtr routing_graph_ptr =
    lanelet::routing::RoutingGraph::build(*map, *trafficRules);

  vector<lanelet::RegulatoryElementPtr> row_elems;
  for (auto regelem : map->regulatoryElementLayer) {
    const auto & attrs = regelem->attributes();
    if (auto it = attrs.find(lanelet::AttributeName::Subtype);
        it != attrs.end() && it->second == lanelet::AttributeValueString::RightOfWay)
      row_elems.push_back(regelem);
  }
  const vector<shared_ptr<lanelet::RightOfWay>> rows =
    lanelet::utils::transformSharedPtr<lanelet::RightOfWay>(row_elems);
  for (auto && row : rows) {
    const vector<lanelet::Lanelet> & right_of_ways = row->rightOfWayLanelets();
    const vector<lanelet::Lanelet> & yields = row->yieldLanelets();
    set<int> yield_ids;
    for (auto && yield : yields) yield_ids.insert(yield.id());

    set<int> conflicting_ids;
    for (auto && right_of_way : right_of_ways) {
      const vector<lanelet::ConstLanelet> & conflicting_lanelets =
        lanelet::utils::getConflictingLanelets(routing_graph_ptr, right_of_way);
      for (auto && conflict : conflicting_lanelets) conflicting_ids.insert(conflict.id());
    }

    vector<int> unnecessary_yields;
    set_difference(
      yield_ids.begin(), yield_ids.end(), conflicting_ids.begin(), conflicting_ids.end(),
      std::inserter(unnecessary_yields, unnecessary_yields.end()));
    if (unnecessary_yields.size() == 0) continue;
    cout << "RightOfWay " << row->id() << ": [";
    const char * delim = "";
    for (auto && unnecessary_yield : unnecessary_yields)
      cout << std::exchange(delim, ",") << unnecessary_yield;
    cout << "] are unnecessary yield" << endl;
  }
}
