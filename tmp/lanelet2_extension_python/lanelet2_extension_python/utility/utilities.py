from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import lanelet2_extension_python._lanelet2_extension_python_boost_python_utility as _utility_cpp
from rclpy.serialization import deserialize_message
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
