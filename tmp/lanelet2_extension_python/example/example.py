import lanelet2
import lanelet2.geometry
from lanelet2_extension_python.projection import MGRSProjector
import lanelet2_extension_python.utility.query as query

# import lanelet2_extension_python.utility.utilities as utility


def test_projection():
    return MGRSProjector(lanelet2.io.Origin(0.0, 0.0))


def test_io(map_path, projection):
    return lanelet2.io.load(map_path, projection)


def test_utility_query(lanelet_map, routing_graph):
    lanelets = query.laneletLayer(lanelet_map)
    print(f"{len(lanelets)=}")
    print(f"""{len(query.subtypeLanelets(lanelets, "road"))=}""")
    print(f"""{len(query.subtypeLanelets(lanelets, "road"))=}""")
    print(f"""{len(query.crosswalkLanelets(lanelets))=}""")
    print(f"""{len(query.walkwayLanelets(lanelets))=}""")
    print(f"""{len(query.roadLanelets(lanelets))=}""")
    print(f"""{len(query.shoulderLanelets(lanelets))=}""")
    print(f"""{len(query.trafficLights(lanelets))=}""")
    print(f"""{len(query.autowareTrafficLights(lanelets))=}""")
    print(f"""{len(query.detectionAreas(lanelets))=}""")
    print(f"""{len(query.noStoppingAreas(lanelets))=}""")
    print(f"""{len(query.noParkingAreas(lanelets))=}""")
    print(f"""{len(query.speedBumps(lanelets))=}""")
    print(f"""{len(query.crosswalks(lanelets))=}""")


def test_utility_utilities():
    pass


if __name__ == "__main__":
    proj = test_projection()
    lanelet_map = test_io(
        "/home/mamorusobue/workspace/pilot-auto.xx1/src/autoware/common/tmp/lanelet2_extension_python/example/sample-map-planning/lanelet2_map.osm",
        proj,
    )

    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)

    test_utility_utilities()
    test_utility_query(lanelet_map, routing_graph)
