# ros2_video_streamer

A simple ROS 2 package for reading a video file and publishing its frames as ROS image messages.

---

## Overview

`ros2_video_streamer` is a lightweight ROS 2 node that reads video files (e.g., `.mp4`, `.avi`) and publishes each frame as a `sensor_msgs/msg/Image` message.
This package is useful for:
- Simulating camera inputs from prerecorded videos
- Testing perception nodes without real cameras

---

## Features

- Reads video files using OpenCV
- Publishes frames as ROS 2 image topics
- Adjustable frame rate (can match real-time or play faster/slower)
- Supports looping playback

---

## Dependencies

Make sure you have the following installed:

- ROS 2 (The package is developed and tested in Humble)
- OpenCV
