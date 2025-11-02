import os
import launch
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    params = [{"video_path": os.path.join(os.environ["HOME"], "Videos", "demo.mp4"),
               "image_out": "/image",
               "fps": 30}]
#    nodes.append(node)


#    return launch.LaunchDescription([container])
