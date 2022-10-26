from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from typing import List
from typing import Optional

from launch.some_substitutions_type import SomeSubstitutionsType

class RespawnableNodeContainer(ComposableNodeContainer):
    """
    Action that executes a container node for composable ROS nodes with fixed
    respawn flag and time.
    """

    def __init__(
        self,
        *,
        name: SomeSubstitutionsType,
        namespace: SomeSubstitutionsType,
        composable_node_descriptions: Optional[List[ComposableNode]] = None,
        respawn = True,
        respawn_delay = 0.5,
        **kwargs
    ) -> None:
        super().__init__(
            name=name,
            namespace=namespace,
            composable_node_descriptions=composable_node_descriptions,
            respawn=respawn,
            respawn_delay=respawn_delay,
            **kwargs)

