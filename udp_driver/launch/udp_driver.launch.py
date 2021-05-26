import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml

import lifecycle_msgs.msg

def generate_launch_description():
    ld = launch.LaunchDescription()

    parameters_file_path = Path(get_package_share_directory('udp_driver'), 'params', 'example_udp_params.yaml')

    with open(parameters_file_path, 'r') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    print(params)

    udp_driver_receiver = LifecycleNode(
        namespace = '',
        name = 'udp_driver_receiver',
        package = 'udp_driver',
        executable = 'udp_receiver_node_exe',
        parameters = [params],
        output = 'screen')

    udp_driver_sender = LifecycleNode(
        namespace = '',
        name = 'udp_driver_sender',
        package = 'udp_driver',
        executable = 'udp_sender_node_exe',
        parameters = [params],
        output = 'screen')

    # Make the udp_driver node take the 'configure' transition
    udp_receiver_configure_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(udp_driver_receiver),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Make the ZED node take the 'activate' transition
    udp_receiver_activate_event = EmitEvent(
        event = ChangeState(
            lifecycle_node_matcher = launch.events.matches_action(udp_driver_receiver),
            transition_id = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
         )
    )

    ld.add_action(udp_driver_receiver)
    ld.add_action(udp_receiver_configure_event)
    ld.add_action(udp_receiver_activate_event)

    return ld