#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from os import name
from launch import LaunchDescription
from launch_ros.actions import Node

map_frame_name = 'map'
'''
origin_lat = 33.0
origin_lon = 130.0
angle_offset = 0.0
scale_offset = 1.0
'''
#offset_param_file = '/home/common/ros2_ws/src/bot_navigation/param/gnss_offset.yaml'
offset_param_file = '/home/common/ros2_ws/src/latlng_tools/latlng_tools/param/gnss_offset.yaml'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lawnmower',
            executable='mow_planner',
            namespace='mow_area',
            output='screen',
            parameters=[{
                'sub_topic_name': 'area_polygon',
                'pub_topic_name': 'goal_list_polygon',

                #'route_mode': 1,
                'route_mode': 2,
                'mow_width': 0.5,
                'overlap_rate': 0.0
            }],
            remappings=[('/mow_area/tf', '/tf'), ('/mow_area/tf_static', '/tf_static')]
        ),
        Node(
            #package='bot_navigation',
            package='latlng_tools',
            executable='latlng2pos_polygon',
            namespace='mow_area',
            output='screen',
            parameters=[
                offset_param_file,
                {
                    'map_frame_name': map_frame_name,
                    #'origin_lat': origin_lat,
                    #'origin_lon': origin_lon,
                    #'angle_offset': angle_offset,
                    #'scale_offset': scale_offset,

                    'use_polygon64': True,
                    #'use_polygon64': False,

                    'sub_topic_name': 'area_latlon',
                    #'sub_topic_name': 'area_latlon_64',
                    'pub_topic_name': 'area_polygon'
                }
            ]
        ),
        Node(
            package='bot_navigation',
            executable='polygon2posearray',
            namespace='mow_area',
            output='screen',
            parameters=[{
                'sub_topic_name': 'goal_list_polygon',
                'pub_topic_name': '/nav_manager/goal_list',

                #'yaw_is_fixed': False,
                'yaw_is_fixed': True,
                'fixed_yaw': 1.57,

                'final_yaw_is_to_origin': False,
                #'final_yaw': 0.0
                'final_yaw': 1.57
            }]
        ),
        Node(
            #package='bot_navigation',
            package='latlng_tools',
            executable='pos2latlng_polygon',
            namespace='mow_area',
            output='screen',
            parameters=[
                offset_param_file,
                {
                    #'origin_lat': origin_lat,
                    #'origin_lon': origin_lon,
                    #'angle_offset': angle_offset,
                    #'scale_offset': scale_offset,

                    'sub_topic_name': 'goal_list_polygon',
                    #'pub_topic_name': 'goal_list_latlon_polygon'
                    'pub_topic_name': 'goal_list_latlon'
                }
            ]
        ),
        Node(
            #package='bot_navigation',
            package='latlng_tools',
            executable='polygon2markerline',
            namespace='mow_area',
            output='screen',
            parameters=[{
                #'sub_topic_name': 'goal_list_latlon_polygon',
                'sub_topic_name': 'goal_list_latlon',
                'pub_topic_name': 'goal_list_latlon_marker',

                'marker_color_rgb': [0.0, 1.0, 1.0],
                'marker_scale': [0.1, 0.1, 0.1],

                'log_print': False
            }]
        ),

        # 選択した作業範囲可視化
        Node(
            package='latlng_tools',
            executable='pos2latlng_polygon',
            namespace='mow_area/select',
            output='screen',
            parameters=[
                offset_param_file,
                {
                    'sub_topic_name': 'area_polygon',
                    'pub_topic_name': 'area_latlon',

                    'log_print': False
                }
            ]
        ),
        Node(
            package='latlng_tools',
            executable='polygon2markerline',
            namespace='mow_area/select',
            output='screen',
            parameters=[{
                'sub_topic_name': 'area_latlon',
                'pub_topic_name': 'area_latlon_marker',

                'marker_color_rgb': [1.0, 0.2, 0.2],
                'marker_scale': [0.2, 0.2, 0.2],

                'log_print': False
            }]
        ),
        
        Node(
            package='bot_navigation',
            executable='navigation_manager',
            namespace='nav_manager',
            name='nav_manager',
            output='screen',
        ),
        Node(
            #package='bot_navigation',
            package='latlng_tools',
            executable='latlng2pos_polygon',
            namespace='nav_manager',
            output='screen',
            parameters=[
                offset_param_file,
                {
                    'map_frame_name': map_frame_name,
                    #'origin_lat': origin_lat,
                    #'origin_lon': origin_lon,
                    #'angle_offset': angle_offset,
                    #'scale_offset': scale_offset,

                    'use_polygon64': True,

                    'sub_topic_name': 'goal_list_latlon',
                    'pub_topic_name': 'goal_list_polygon'
                }
            ]
        ),
        Node(
            package='bot_navigation',
            executable='polygon2posearray',
            namespace='nav_manager',
            output='screen',
            parameters=[{
                'sub_topic_name': 'goal_list_polygon',
                'pub_topic_name': 'goal_list',

                'final_yaw_is_to_origin': False,
                'final_yaw': 0.0
            }]
        )
    ])
