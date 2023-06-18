from typing import overload

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from lanelet2.core import BasicPoint2d
import lanelet2_extension_python._lanelet2_extension_python_boost_python_utility as _utility_cpp

# from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

combineLaneletsShape = _utility_cpp.combineLaneletsShape
generateFineCenterline = _utility_cpp.generateFineCenterline
getCenterlineWithOffset = _utility_cpp.getCenterlineWithOffset
getRightBoundWithOffset = _utility_cpp.getRightBoundWithOffset
getLeftBoundWithOffset = _utility_cpp.getLeftBoundWithOffset
getExpandedLanelet = _utility_cpp.getExpandedLanelet
getExpandedLanelets = _utility_cpp.getExpandedLanelets
overwriteLaneletsCenterline = _utility_cpp.overwriteLaneletsCenterline
getLaneletLength2d = _utility_cpp.getLaneletLength2d
getLaneletLength3d = _utility_cpp.getLaneletLength3d


def getArcCoordinates(lanelet_sequence, pose: Pose):
    pose_byte = serialize_message(pose)
    return _utility_cpp.getArcCoordinates(lanelet_sequence, pose_byte)


getClosestSegment = _utility_cpp.getClosestSegment
getPolygonFromArcLength = _utility_cpp.getPolygonFromArcLength


def getLaneletAngle(lanelet, point: Point):
    point_byte = serialize_message(point)
    return _utility_cpp.getLaneletAngle(lanelet, point_byte)


def isInLanelet(pose: Pose, lanelet, radius=0.0):
    pose_byte = serialize_message(pose)
    return _utility_cpp.isInLanelet(pose_byte, lanelet, radius)


"""
TODO
def getClosestCenterPose(lanelet, point: Point):
    point_byte = serialize_message(point)
    pose_byte = _utility_cpp.getClosestCenterPose(lanelet, point_byte)
    bytes = []
    for i in range(len(pose_byte)):
        bytes.append(pose_byte[i].encode("latin1"))
    print(bytes)
    return deserialize_message(bytes, Pose)
"""


def getLateralDistanceToCenterline(lanelet, pose: Pose):
    pose_byte = serialize_message(pose)
    return _utility_cpp.getLateralDistanceToCenterline(lanelet, pose_byte)


def getLateralDistanceToClosestLanelet(lanelet_sequence, pose: Pose):
    pose_byte = serialize_message(pose)
    return _utility_cpp.getLateralDistanceToClosestLanelet(lanelet_sequence, pose_byte)


laneletLayer = _utility_cpp.laneletLayer
subtypeLanelets = _utility_cpp.subtypeLanelets
crosswalkLanelets = _utility_cpp.crosswalkLanelets
walkwayLanelets = _utility_cpp.walkwayLanelets
roadLanelets = _utility_cpp.roadLanelets
shoulderLanelets = _utility_cpp.shoulderLanelets
trafficLights = _utility_cpp.trafficLights
autowareTrafficLights = _utility_cpp.autowareTrafficLights
noStoppingAreas = _utility_cpp.noStoppingAreas
noParkingAreas = _utility_cpp.noParkingAreas
speedBumps = _utility_cpp.speedBumps
crosswalks = _utility_cpp.crosswalks
curbstones = _utility_cpp.curbstones
getAllPolygonsByType = _utility_cpp.getAllPolygonsByType
getAllObstaclePolygons = _utility_cpp.getAllObstaclePolygons
getAllParkingLots = _utility_cpp.getAllParkingLots
getAllPartitions = _utility_cpp.getAllPartitions
getAllFences = _utility_cpp.getAllFences
getAllPedestrianMarkings = _utility_cpp.getAllPedestrianMarkings
getAllParkingSpaces = _utility_cpp.getAllParkingSpaces

# TODO(Mamoru Sobue): how to dispatch overloads
getLinkedParkingSpaces = _utility_cpp.getLinkedParkingSpaces

# TODO(Mamoru Sobue): how to dispatch overloads
getLinkedLanelet = _utility_cpp.getLinkedLanelet

# TODO(Mamoru Sobue): how to dispatch overloads
getLinkedLanelets = _utility_cpp.getLinkedLanelets
help(getLinkedLanelets)

# TODO(Mamoru Sobue): how to dispatch overloads
getLinkedParkingLot = _utility_cpp.getLinkedParkingLot

stopLinesLanelets = _utility_cpp.stopLinesLanelets
stopLinesLanelet = _utility_cpp.stopLinesLanelet
stopSignStopLines = _utility_cpp.stopSignStopLines


@overload
def getLaneletsWithinRange(lanelets, point: BasicPoint2d, rng: float):
    return _utility_cpp.getLaneletsWithinRange(lanelets, point, rng)


@overload
def getLaneletsWithinRange(lanelets, point: Point, rng: float):
    point_byte = serialize_message(point)
    return _utility_cpp.getLaneletsWithinRange_point(lanelets, point_byte, rng)


@overload
def getLaneChangeableNeighbors(routing_graph, lanelet):
    return _utility_cpp.getLaneChangeableNeighbor(routing_graph, lanelet)


@overload
def getLaneChangeableNeighbors(routing_graph, lanelets, point: Point):
    point_byte = serialize_message(point)
    return _utility_cpp.getLaneChangeableNeighbor_point(routing_graph, lanelets, point_byte)


@overload
def getAllNeighbors(routing_graph, lanelet):
    return _utility_cpp.getAllNeighbors(routing_graph, lanelet)


@overload
def getAllNeighbors(routing_graph, lanelets, point: Point):
    point_byte = serialize_message(point)
    return _utility_cpp.getAllNeighbors(routing_graph, lanelets, point_byte)


getAllNeighborsLeft = _utility_cpp.getAllNeighborsLeft
getAllNeighborsRight = _utility_cpp.getAllNeighborsRight


def getClosestLaneletWithConstrains(
    lanelets, pose: Pose, closest_lanelet, dist_threshold, yaw_threshold
):
    pose_byte = serialize_message(pose)
    return _utility_cpp.getClosestLaneletWithConstrains(
        lanelets, pose_byte, closest_lanelet, dist_threshold, yaw_threshold
    )


@overload
def getCurrentLanelets(point: Point):
    point_byte = serialize_message(point)
    return _utility_cpp.getCurrentLanelets_point(point_byte)


@overload
def getCurrentLanelets(pose: Pose):
    pose_byte = serialize_message(pose)
    return _utility_cpp.getCurrentLanelets_pose(pose_byte)


getSucceedingLaneletSequences = _utility_cpp.getSucceedingLaneletSequences
getPrecedingLaneletSequences = _utility_cpp.getPrecedingLaneletSequences
