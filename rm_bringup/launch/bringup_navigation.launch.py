import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    livox_bringup_dir = get_package_share_directory('livox_ros_driver2')
    segmentation_bringup_dir = get_package_share_directory('linefit_ground_segmentation_ros')
    fast_lio_bringup_dir = get_package_share_directory('fast_lio')
    laserscan_conversion_bringup_dir = get_package_share_directory('pointcloud_to_laserscan')
    icp_bringup_dir = get_package_share_directory('icp_registration')
    navigation_bringup_dir = get_package_share_directory('rm_navigation')
    rm_decision_bringup_dir = get_package_share_directory('rm_decision')

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(livox_bringup_dir, 'launch', 'msg_MID360_launch.py'))
        )

    segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(segmentation_bringup_dir, 'launch', 'segmentation.launch.py'))
        )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fast_lio_bringup_dir, 'launch', 'mapping.launch.py'))
        )

    laserscan_conversion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(laserscan_conversion_bringup_dir, 'launch', 'pointcloud_to_laserscan_launch.py'))
        )

    icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(icp_bringup_dir, 'launch', 'icp.launch.py'))
        )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_bringup_dir, 'launch', 'bringup_launch.py'))
        )

    decision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rm_decision_bringup_dir, 'launch', 'rm_decision_launch.py')))
    
    
    delay_segmentation_launch = TimerAction(
        period=3.5,
        actions=[segmentation_launch]
    )
    
    delay_fast_lio_launch = TimerAction(
        period=3.0,
        actions=[fast_lio_launch]
    )

    delay_laserscan_conversion_launch = TimerAction(
        period=4.0,
        actions=[laserscan_conversion_launch]
    )

    delay_icp_launch = TimerAction(
        period=4.5,
        actions=[icp_launch]
    )

    delay_bringup_launch = TimerAction(
        period=5.5,
        actions=[bringup_launch]
    )

    delay_decision_launch = TimerAction(
        period=7.0,
        actions=[decision_launch]
    )

    return LaunchDescription([
        livox_launch,
        delay_segmentation_launch,
        delay_fast_lio_launch,
        delay_laserscan_conversion_launch,
        delay_icp_launch,
        delay_bringup_launch,
        delay_decision_launch
    ])