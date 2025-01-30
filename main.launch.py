# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch.actions import ExecuteProcess

import os

def generate_launch_description():
    # Get the launch directory and other file paths
    param_file_path = '/home/user/amcl_test/config/params.yaml'
    my_map_dir = '/home/user/sim_ws/src/f1tenth_gym_ros/maps'
    my_param_dir = '/home/user/amcl_test/config'

    my_param_file = 'params.yaml'
    my_map_file = 'levine.yaml'
    map_yaml_file = '/home/user/sim_ws/src/f1tenth_gym_ros/maps/levine.yaml'
    
    tf_route = '/home/user/amcl_test/transform.py'
    laser_tf_route = '/home/user/amcl_test/laser_model_transform.py'
    
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')

    lifecycle_nodes = ['amcl' , 'controller_server' , 'planner_server', 'behavior', ]
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # URDF
    urdf = '/home/user/amcl_test/ego.urdf'

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(my_map_dir, my_map_file),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(my_param_dir, my_param_file),
        description='Full path to the ROS2 parameters file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='Name of the container that nodes will load in if composition is used')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    
    
    #########################
    # Robot state Publisher #
    #########################   
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf])

    
    
    ####################################
    # AMCL Robot TF Translation Launch #
    ####################################   
    tf_launch = ExecuteProcess(
        cmd=['python3', tf_route],
        output='screen',
    )


    ##########################
    # Laser Transform Launch #
    ##########################
    laser_model_tf_launch = ExecuteProcess(
        cmd=['python3', laser_tf_route],
        output='screen',
    )

    
    ##################
    # A M C L Launch #
    ##################   
    amcl_launch = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        remappings=remappings
    )


    ###################
    # Behavior Launch #
    ###################   
    behavior_launch = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='behavior',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )


    ##########################
    # Behavior Server Launch #
    ##########################
    behavior_server_launch = Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args',  ],
                remappings=remappings)
                
    ##################
    # Planner Launch #
    ##################          
      
    planner_launch = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args',  ],
                remappings=remappings)
    
    
    controller_server_launch = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args',  ],
            remappings = remappings
            )
    

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            robot_state_publisher_node,
            tf_launch,
            laser_model_tf_launch,
            amcl_launch,
            behavior_launch,
            planner_launch,
            controller_server_launch,
            
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                    {'managed_nodes': lifecycle_nodes}
                ]
            ),
                
                
        ]
    )

    
    load_composable_nodes = LoadComposableNodes(
    condition=IfCondition(use_composition),
    target_container=container_name,
    composable_node_descriptions=[
        ComposableNode(package='nav2_map_server', plugin='nav2_map_server::MapServer', name='map_server', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_amcl', plugin='nav2_amcl::AmclNode', name='amcl', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator', name='bt_navigator', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager', name='lifecycle_manager_localization', parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': lifecycle_nodes}]),
        ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer', name='planner_server', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_behaviors', plugin='behavior_server::BehaviorServer', name='behavior_server', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_costmap_2d', plugin='nav2_costmap_2d::Costmap2DROS', name='global_costmap', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_costmap_2d', plugin='nav2_costmap_2d::Costmap2DROS', name='local_costmap', parameters=[configured_params], remappings=remappings),
        ComposableNode(package='nav2_controller', plugin='nav2_controller::ControllerServer', name='controller_server', parameters=[configured_params], remappings=remappings)
    ]
    )
    

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)

    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
