from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import lanelet2.core
import lanelet2_extension_python._lanelet2_extension_python_boost_python_utility as _utility_cpp

# from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message

laneletLayer = _utility_cpp.laneletLayer
subtypeLanelets = _utility_cpp.subtypeLanelets
crosswalkLanelets = _utility_cpp.crosswalkLanelets
walkwayLanelets = _utility_cpp.walkwayLanelets
roadLanelets = _utility_cpp.roadLanelets
shoulderLanelets = _utility_cpp.shoulderLanelets
trafficLights = _utility_cpp.trafficLights
autowareTrafficLights = _utility_cpp.autowareTrafficLights
detectionAreas = _utility_cpp.detectionAreas
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

# TODO(Mamoru Sobue): how to dispatch overloads
getLinkedParkingLot = _utility_cpp.getLinkedParkingLot

stopLinesLanelets = _utility_cpp.stopLinesLanelets
stopLinesLanelet = _utility_cpp.stopLinesLanelet
stopSignStopLines = _utility_cpp.stopSignStopLines


def getLaneletsWithinRange(lanelets, point, rng):
    if isinstance(point, Point):
        point_byte = serialize_message(point)
        return _utility_cpp.getLaneletsWithinRange_point(lanelets, point_byte, rng)
    if isinstance(point, lanelet2.core.BasicPoint2d):
        return _utility_cpp.getLaneletsWithinRange(lanelets, point, rng)
    raise TypeError("argument point is neither BasicPoint2d nor Point")


def getLaneChangeableNeighbors(*args):
    if len(args) == 2 and isinstance(args[1], lanelet2.core.Lanelet):
        return _utility_cpp.getLaneChangeableNeighbors(args[0], args[1])
    if len(args) == 3 and isinstance(args[2], Point):
        point_byte = serialize_message(args[2])
        return _utility_cpp.getLaneChangeableNeighbor_point(args[0], args[1], point_byte)
    raise TypeError("argument number does not match or 3rd argument is not Point type")


def getAllNeighbors(*args):
    if len(args) == 2 and isinstance(args[1], lanelet2.core.Lanelet):
        return _utility_cpp.getAllNeighbors(args[0], args[1])
    if len(args) == 3 and isinstance(args[2], Point):
        point_byte = serialize_message(args[2])
        return _utility_cpp.getAllNeighbors(args[0], args[1], point_byte)


getAllNeighborsLeft = _utility_cpp.getAllNeighborsLeft
getAllNeighborsRight = _utility_cpp.getAllNeighborsRight


def getClosestLaneletWithConstrains(
    lanelets, pose: Pose, closest_lanelet, dist_threshold, yaw_threshold
):
    pose_byte = serialize_message(pose)
    return _utility_cpp.getClosestLaneletWithConstrains(
        lanelets, pose_byte, closest_lanelet, dist_threshold, yaw_threshold
    )


def getCurrentLanelets(lanelets, point: Point, current_lanelets):
    if isinstance(point, Point):
        point_byte = serialize_message(point)
        return _utility_cpp.getCurrentLanelets_point(lanelets, point_byte, current_lanelets)
    if isinstance(point, Pose):
        pose_byte = serialize_message(point)
        return _utility_cpp.getCurrentLanelets_pose(lanelets, pose_byte, current_lanelets)
    raise TypeError("argument number does not match or 3rd argument is not Pose type")


getSucceedingLaneletSequences = _utility_cpp.getSucceedingLaneletSequences
getPrecedingLaneletSequences = _utility_cpp.getPrecedingLaneletSequences
