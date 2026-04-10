import launch
import launch_ros
from ament_index_python import get_package_share_directory
import os
import launch_ros.parameter_descriptions
import random
from launch.substitutions import TextSubstitution
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    # 获取URDF文件路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path  , 'urdf', 'fishbot/fishbot.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_package_path  , 'config', 'display_robot_model.rviz')
    default_gazebo_world_path = os.path.join(urdf_package_path  , 'world', 'custom_room.world')
    #声明一个urdf目录参数，方便修改,不写这个，不能在启动命令里传路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='加载的模型文件路径'
    )
    
    
    #通过文件路径，获取内容，并转换成参数值对象，以供传入robot_state_publisher
    substitutions_command_result =launch.substitutions.Command(
        ['xacro ',
        launch.substitutions.LaunchConfiguration('model')])
    #打包成参数值对象
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result,
                                                                               value_type=str)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )
    
    # entity_name = 'fishbot_' + str(random.randint(0, 1000))
    # action_spawn_entity = launch.actions.TimerAction(
    #     period=3.0,
    #     actions=[
    #         launch_ros.actions.Node(
    #             package='gazebo_ros',
    #             executable='spawn_entity.py',
    #             arguments=[
    #                 '-topic', 'robot_description',
    #                 '-entity', TextSubstitution(text=entity_name)
    #             ]
    #         )
    #     ]
    # )
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]
    )
    entity_name = 'fishbot_' + str(random.randint(0, 1000))
   
    action_spawn_entity =   launch_ros.actions.Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', entity_name],
            )
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_joint_state_broadcaster', '--set-state', 'active'],
        output='screen'
    )
    action_load_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_effort_controller', '--set-state', 'active'],
        output='screen'
    )
    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_diff_drive_controller', '--set-state', 'active'],
        output='screen'
    )

    return launch.LaunchDescription([
        
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_spawn_entity,
        action_launch_gazebo,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity,
                on_exit=[action_load_joint_state_controller]
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller]
            )
        )
    ])