from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    # tracker node *******************************************************************************
    node = Node(
        package='rosbot_follower',
        executable='tracker',
        name='tracker',
        output='screen',
    )
    ld.add_action(node)

    # main controller node **********************************************************************************
    node = Node(
        package='rosbot_follower',
        executable='main_controller',
        name='main_controller',
        output='screen',
    )
    ld.add_action(node)
    
    # follow node **********************************************************************************
    # node = Node(
    #     package='rosbot_follower',
    #     executable='follow',
    #     name='follow',
    #     output='screen',
    # )
    # ld.add_action(node)

    #navigator node ****************************************************************************************
    node = Node(
        package='rosbot_follower',
        executable='navigator',
        name='navigator',
        output='screen',
    )
    ld.add_action(node)

    #motion controller node *******************************************************************************
    node = Node(
        package='rosbot_follower',
        executable='motion_controller',
        name='motion_controller',
        output='screen',
    )
    ld.add_action(node)

    #node = Node(
    #    package='rosbot_follower',
    #    executable='lidar_logger',
    #    name='lidar_logger',
    #    output='screen',
    #)
    #ld.add_action(node)

    node = Node(
        package='rosbot_follower',
        executable='lidar_track',
        name='lidar_track',
        output='screen',
    )
    ld.add_action(node)

    return ld