import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    nodes = []

    params = [{"video_path": os.path.join(os.environ["HOME"], "Videos", "demo.mp4"),
               "image_out": "/image",
               "fps": 30}]
    node = ComposableNode(
                package="video_image_publisher",
                plugin="ros2_example::VideoImagePublisher",
                name=f"video_image_publisher",
                parameters=params,
                extra_arguments=[{"use_intra_process_comms": True}])
    nodes.append(node)


    container = ComposableNodeContainer(
            name="ros2_example_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=nodes,
            output="screen")


    return launch.LaunchDescription([container])
