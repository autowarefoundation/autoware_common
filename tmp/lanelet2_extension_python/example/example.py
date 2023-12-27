import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import lanelet2
import lanelet2.geometry
from lanelet2_extension_python.projection import MGRSProjector
import lanelet2_extension_python.utility.query as query
import numpy as np


def test_projection():
    return MGRSProjector(lanelet2.io.Origin(0.0, 0.0))


def test_io(map_path, projection):
    return lanelet2.io.load(map_path, projection)


def test_utility_query(lanelet_map, routing_graph):
    lanelets = query.laneletLayer(lanelet_map)
    lanelet56 = lanelet_map.laneletLayer.get(56)
    lanelet108 = lanelet_map.laneletLayer.get(108)
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
    print(f"""{len(query.curbstones(lanelet_map))=}""")
    print(f"""{len(query.getAllPolygonsByType(lanelet_map, "parking_lot"))=}""")
    print(f"""{len(query.getAllParkingLots(lanelet_map))=}""")
    print(f"""{len(query.getAllPartitions(lanelet_map))=}""")
    print(f"""{len(query.getAllParkingSpaces(lanelet_map))=}""")
    print(f"""{len(query.getAllFences(lanelet_map))=}""")
    print(f"""{len(query.getAllPedestrianMarkings(lanelet_map))=}""")
    print(f"""{len(query.getLinkedParkingSpaces(lanelet56, lanelet_map))=}""")
    print(
        f"""{len(query.getLinkedParkingSpaces(lanelet56, query.getAllParkingSpaces(lanelet_map), query.getAllParkingLots(lanelet_map)))=}"""
    )
    print(f"""{len(query.stopLinesLanelets(lanelets))=}""")
    print(f"""{len(query.stopLinesLanelet(lanelet108))=}""")
    print(f"""{len(query.stopSignStopLines(lanelets))=}""")
    lanelet56_centerline_basic_points = [p.basicPoint() for p in lanelet56.centerline]
    lanelet56_centerline_center = np.sum(lanelet56_centerline_basic_points) * (
        1.0 / len(lanelet56_centerline_basic_points)
    )
    search_point = Point()
    search_point.x = lanelet56_centerline_center.x
    search_point.y = lanelet56_centerline_center.y
    search_point.z = lanelet56_centerline_center.z
    print(f"""{[ll2.id for ll2 in query.getLaneletsWithinRange(lanelets, search_point, 15.0)]=}""")
    search_point_2d = lanelet2.core.BasicPoint2d()
    search_point_2d.x = lanelet56_centerline_center.x
    search_point_2d.y = lanelet56_centerline_center.y
    print(
        f"""{[ll2.id for ll2 in query.getLaneletsWithinRange(lanelets, search_point_2d, 15.0)]=}"""
    )
    print(f"""{[ll2.id for ll2 in query.getLaneChangeableNeighbors(routing_graph, lanelet108)]=}""")
    print(f"""{[ll2.id for ll2 in query.getAllNeighbors(routing_graph, lanelet56)]=}""")
    print(f"""{[ll2.id for ll2 in query.getAllNeighborsLeft(routing_graph, lanelet56)]=}""")
    print(f"""{[ll2.id for ll2 in query.getAllNeighborsRight(routing_graph, lanelet56)]=}""")
    search_pose = Pose()
    search_pose.position = search_point
    temp_lanelet = lanelet2.core.Lanelet(0, lanelet56.leftBound, lanelet56.rightBound)
    if (
        query.getClosestLaneletWithConstrains(lanelets, search_pose, temp_lanelet, 10.0, math.pi)
        and temp_lanelet is not None
    ):
        print(f"""{temp_lanelet.id}""")
    # lanelet::ConstLaneletsへのpointerだからこの関数は無理
    temp_lanelets = []
    if query.getCurrentLanelets(lanelets, search_point, temp_lanelets):
        print(f"""{[ll2.id for ll2 in temp_lanelets]}""")


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
