from geometry_msgs.msg import Pose
import lanelet2
import lanelet2.geometry
from lanelet2_extension_python.projection import MGRSProjector
from lanelet2_extension_python.utility.utilities import getArcCoordinates
from lanelet2_extension_python.utility.utilities import getLaneletAngle
from lanelet2_extension_python.utility.utilities import getLateralDistanceToClosestLanelet
from lanelet2_extension_python.utility.utilities import isInLanelet
import numpy as np


def print_layer(layer, layerName):
    print("IDs in " + layerName)
    print(sorted([elem.id for elem in layer]))


# Download the file from
# https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/#preparation
if __name__ == "__main__":
    proj = MGRSProjector(lanelet2.io.Origin(0.0, 0.0))
    ll2_map = lanelet2.io.load(
        "/home/mamorusobue/workspace/pilot-auto.xx1/src/autoware/common/tmp/lanelet2_extension_python/example/sample-map-planning/lanelet2_map.osm",
        proj,
    )

    layers = {
        #        "Points": ll2_map.pointLayer,
        #        "Line Strings": ll2_map.lineStringLayer,
        "Polygons": ll2_map.polygonLayer,
        "Lanelets": ll2_map.laneletLayer,
        "Areas": ll2_map.areaLayer,
        #        "Regulatory Elements": ll2_map.regulatoryElementLayer,
    }

    for layer_name, layer in layers.items():
        print_layer(layer, layer_name)

    lanelet56 = ll2_map.laneletLayer.get(56)

    # regulatory element
    # get associated traffic lights
    lights = lanelet56.trafficLights()
    for light in lights:
        print(light.stopLine)
    # get turn_direction
    if "turn_direction" in lanelet56.attributes:
        turn_direction = lanelet56.attributes["turn_direction"]
        print(f"lanelet56 has {turn_direction} turn_direction value")

    # routing
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    graph = lanelet2.routing.RoutingGraph(ll2_map, traffic_rules)
    # get conflicting lanelets
    conflictings = graph.conflicting(lanelet56)
    print(f"lanelet56 is conflicting with {[conflicting.id for conflicting in conflictings]}")
    # get following lanelets
    followings = graph.following(lanelet56)
    print(f"lanelet56 is connected to {[following.id for following in followings]}")
    previouses = graph.previous(lanelet56)
    print(f"lanelet56 is followed by {[previous.id for previous in previouses]}")

    centerline = lanelet56.centerline
    print(
        f"The centerline of lanelet56 is of id {centerline.id} and contains {len(centerline)} points."
    )
    # access to points
    pts = []
    for p in centerline:
        pts.append(p)
    # length, area
    print(f"The length centerline is {lanelet2.geometry.length(centerline)}.")

    basic_pts = [p.basicPoint() for p in pts]
    center = np.sum(basic_pts) * (1.0 / len(basic_pts))

    center_sd_frame = lanelet2.geometry.toArcCoordinates(
        lanelet2.geometry.to2D(centerline), lanelet2.geometry.to2D(center)
    )
    print(
        f"""The center of centerline56 is ({center_sd_frame.distance}, {center_sd_frame.length}) in arc coordinates"""
    )

    # test utility
    pose = Pose()
    pose.position.x = center.x
    pose.position.y = center.y
    pose.position.z = center.z
    print(f"pose = [{pose.position.x}, {pose.position.y}, {pose.position.z}]")

    arc_coord = getArcCoordinates([lanelet56], pose)
    print(f"getArcCoordinates(lanelet56, pose) = {arc_coord.distance}, {arc_coord.length}")

    lanelet_angle = getLaneletAngle(lanelet56, pose.position)
    print(lanelet_angle)

    print(f"isInLanelet(pose, lanelet56) = {isInLanelet(pose, lanelet56)}")

    """
    closest_center_pose = getClosestCenterPose(lanelet56, pose.position)
    print(
        f"getClosestCenterPose(lanelet56, pose.position) = ({closest_center_pose})"
    )
    """

    print(
        f"getLateralDistanceToClosestLanelet(lanelet56, pose) = {getLateralDistanceToClosestLanelet([lanelet56], pose)}"
    )
