#!/usr/bin/env python3
"""起 slam_toolbox 的异步建图节点，配 slam_bridge.py 发布的 /scan + /odom + TF。

桥接节点自己另起（它必须用 python3.8 直接跑，见 slam_bridge.py 头注释），
所以这里只管 slam_toolbox 本身。use_sim_time 必须为 true —— 桥接发布 /clock，
时间戳都是仿真时间，不打开的话 TF 查找会全部超时。

用法:
    source /opt/ros/foxy/setup.bash
    ros2 launch ./slam/slam.launch.py            # 或 python3.8 slam/slam.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, "mapper_params_online_async.yaml")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="用仿真时间(/clock)，桥接节点必须配 true"),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[PARAMS, {"use_sim_time": use_sim_time}],
        ),
    ])


if __name__ == "__main__":
    # 允许直接 python3.8 slam/slam.launch.py 跑，省得记 ros2 launch 的路径规则
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    raise SystemExit(ls.run())
