from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    #find object 2d *********************************************************************************

    # image_topic = '/camera/color/image_raw'
    image_topic = '/camera/color/image_raw/compressed'
    image_topic_repeat = image_topic + '/repeat'
    # use_compressed = 'false'
    use_compressed = 'true'

    # Environment Variables
    ev_stdout = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')
    ev_logging_stream = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')

    ld.add_action(ev_stdout)
    ld.add_action(ev_logging_stream)


    # Launch arguments
    # Set to false when running after training is finished
    la_gui = DeclareLaunchArgument('gui', default_value='false', description='Launch GUI.')

    # Our default camera topic. If streaming images, consider using the compressed image instead
    la_image_topic = DeclareLaunchArgument('image_topic', default_value=image_topic, description='Image topic from the camera (best_effort).')
    la_image_repeat = DeclareLaunchArgument('image_topic_repeat', default_value=image_topic_repeat, description='Image to repeat to for find object (reliable).')
    la_image_compressed = DeclareLaunchArgument('use_compressed', default_value=use_compressed, description='Determine if compressed image is to be used')

    # Path where you have saved the existing trained images
    # Uses the path to the AIIL Workspace, but this could be set to anywhere
    # LogInfo(msg=('AIIL_CHECKOUT_DIR, ', EnvironmentVariable(name='AIIL_CHECKOUT_DIR'))),
    #TODO: this will currently break if this file is moved out of launch or launch is moved out of rosbot_follower!!
    la_obj_path = DeclareLaunchArgument('objects_path',
                           default_value=[EnvironmentVariable(name='AIIL_CHECKOUT_DIR'),'/humble_workspace/src/PAR_rosbot_follower/rosbot_follower/objects'],
                           description='Directory containing objects to load on initialization.')

    # Find Object 2D Setting. By default just use the standard settings, but you can copy and tweak this file if you wish
    la_setting_path = DeclareLaunchArgument('settings_path', default_value='~/.ros/find_object_2d.ini', description='Config file.')

    # Launch Configurations
    #lc_gui = LaunchConfiguration('gui')
    #lc_obj_path = LaunchConfiguration('objects_path')
    ld.add_action(la_gui)
    ld.add_action(la_image_topic)
    ld.add_action(la_image_repeat)
    ld.add_action(la_image_compressed)
    ld.add_action(la_obj_path)
    ld.add_action(la_setting_path)

    # Nodes to launch
    # Find Object 2D node
    node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        # output='screen',
        output='log',
        parameters=[{
          'gui': LaunchConfiguration('gui'),
          'objects_path': LaunchConfiguration('objects_path'),
          'settings_path': LaunchConfiguration('settings_path')
        }],
        remappings=[
            ('image', LaunchConfiguration('image_topic_repeat'))
    ])
    ld.add_action(node)

    # Best Effort repeater *****************************************************************
    # since find_object ONLY uses reliable QoS
    #TODO: for now using best effort repeater from aiil_rosbot_demo
    node = Node(
        package='aiil_rosbot_demo',
        executable='best_effort_repeater',
        name='best_effort_repeater',
        output='screen',
        parameters=[
            {'sub_topic_name': LaunchConfiguration('image_topic')},
            {'repeat_topic_name': LaunchConfiguration('image_topic_repeat')},
            {'use_compressed': LaunchConfiguration('use_compressed')},
        ]
    )
    ld.add_action(node)

    # tracker node *******************************************************************************
    node = Node(
        package='rosbot_follower',
        executable='tracker',
        name='tracker',
        output='screen',
    )
    ld.add_action(node)

    return ld