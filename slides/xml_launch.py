#!/usr/bin/env python3

import elsie
from elsie.ext import unordered_list
from my_layouts import *


slides = init_deck()


@slides.slide(debug_boxes=False)
def title(slide):
    content = logo_header_slide(slide, "")
    content.box(width="fill").text(
        "Retro ROS 2 Launch", elsie.TextStyle(size=160, bold=True)
    )
    content.box(width="fill").text(
        "Remember XML files?", elsie.TextStyle(size=36, bold=True, italic=True)
    )
    content.box(width="fill", p_top=10).text("October 19, 2023")
    content.box(width="fill", p_top=180).text("Tyler Weaver")
    content.box(width="fill").text("Staff Software Engineer\ntyler@picknik.ai")


@slides.slide(debug_boxes=False)
def author(slide):
    text_area = image_slide(slide, "Tyler Weaver", get_image_path("kart.jpg"))
    lst = unordered_list(text_area)
    lst.item().text("Racing Kart Driver")
    lst.item().text("MoveIt Maintainer")
    lst.item().text("Rust Evangelist")
    lst.item().text("Docker Skeptic")


@slides.slide(debug_boxes=False)
def comfort_zone(slide):
    code_slide(
        slide,
        "The launch file you remember",
        "xml",
        """
<launch>
  <arg name="pipeline" default="ompl" />
  <arg name="capabilities" default=""/>

  <node name="move_group" pkg="moveit_ros_move_group" type="move_group"
    output="screen">
    <param name="default_planning_pipeline" value="$(arg pipeline)" />
    <param name="capabilities" value="$(arg capabilities)" />
  </node>
</launch>
  """,
    )


@slides.slide(debug_boxes=False)
def trigger(slide):
    slide.update_style("code", elsie.TextStyle(size=16))
    content = logo_header_slide(slide, "The launch we have at home")
    content = slide.overlay(horizontal=True)
    content.box(width="50%", height="fill", p_left=20).code(
        "python",
        """
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("rviz_config",
            default_value="kinova_moveit_config_demo.rviz",
            description="RViz configuration file",
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "dof": "7",
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
  """,
    )
    content.box(width="50%").code(
        "python",
        """
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution([FindPackageShare("moveit2_tutorials"), "launch", rviz_base])

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    nodes_to_start = [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node
    ]

    return nodes_to_start
  """,
    )


@slides.slide(debug_boxes=False)
def initial_success(slide):
    slide.update_style("code", elsie.TextStyle(size=30))
    code_slide(
        slide,
        "The launch she never told you about",
        "xml",
        """
<launch>
  <arg name="robot_ip" default="xxx.yyy.zzz.www" />
  <arg name="use_fake_hardware" default="true" />
  <arg name="gripper" default="robotiq_2f_85" />
  <arg name="dof" default="7" />

  <let name="robot_description" value="$(command 'xacro $(find-pkg-share kortex_description)/robots/gen3.xacro robot_ip:=$(var robot_ip) use_fake_hardware:=$(var use_fake_hardware) gripper:=$(var gripper) dof:=$(var dof)')" />
  <let name="robot_description_semantic" value="$(command 'xacro $(find-pkg-share kinova_gen3_7dof_robotiq_2f_85_moveit_config)/config/gen3.srdf')" />

  <!-- MoveGroup -->
  <node pkg="moveit_ros_move_group" exec="move_group" output="screen">
    <param name="robot_description" value="$(var robot_description)" type="str" />
    <param name="robot_description_semantic" value="$(var robot_description_semantic)" type="str" />
    <param from="$(find-pkg-share easy_launch_demo)/config/moveit.yaml" />
  </node>

  <!-- RViz -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" output="log" args="-d $(find-pkg-share moveit2_tutorials)/launch/kinova_moveit_config_demo.rviz">
    <param name="robot_description" value="$(var robot_description)" type="str" />
    <param name="robot_description_semantic" value="$(var robot_description_semantic)" type="str" />
    <param from="$(find-pkg-share easy_launch_demo)/config/moveit.yaml" />
  </node>

  <!-- Static TF -->
  <node pkg="tf2_ros" exec="static_transform_publisher" name="static_transform_publisher" output="log"
    args="--frame-id world --child-frame-id base_link" />

  <!-- Publish TF -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher" output="both" >
    <param name="robot_description" value="$(var robot_description)" type="str" />
  </node>

  <!-- ros2_control -->
  <node pkg="controller_manager" exec="ros2_control_node" output="both" >
    <param name="robot_description" value="$(var robot_description)" type="str" />
    <param from="$(find-pkg-share kinova_gen3_7dof_robotiq_2f_85_moveit_config)/config/ros2_controllers.yaml" />
  </node>

  <!-- ros2_control spawners -->
  <node pkg="controller_manager" exec="spawner" args="joint_state_broadcaster -c /controller_manager" />
  <node pkg="controller_manager" exec="spawner" args="joint_trajectory_controller -c /controller_manager" />
  <node pkg="controller_manager" exec="spawner" args="robotiq_gripper_controller -c /controller_manager" />
</launch>
  """,
    )
    slide.overlay(show="2+").image("images/xml_launch_rviz.png", scale=1.4)


@slides.slide(debug_boxes=False)
def better_place(slide):
    content = logo_header_slide(slide, "Better Place: ROS 2 XML Launch")
    text_box = content.box(width="fill")
    text_box.set_style("bold", elsie.TextStyle(bold=True))
    lst = unordered_list(text_box)
    lst.item().text(
        "Launch MoveIt with !bold{43} lines of XML vs !bold{~1000} lines of Python",
        escape_char="!",
    )
    lst.item().text("Single ~link{moveit.yaml} config for MoveIt")
    lst.item().text("Try it yourself: ~link{tylerjw.dev/posts/xml-launch}")
    content.fbox().image("images/xml_launch_link_qr.svg")


render_deck(slides, "xml_launch.pdf")
