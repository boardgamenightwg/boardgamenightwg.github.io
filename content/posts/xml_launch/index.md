+++
title = "ROSCON 2023 - Retro ROS 2 Launch"
date = 2023-09-04
type = "post"
description = "XML vs Python"
in_search_index = true
[taxonomies]
tags = ["Talks"]
+++

- [PDF of Slides](/pdf/xml_launch.pdf)
- [Demo code for launching moveit](https://github.com/tylerjw/tylerjw.dev/tree/main/content/posts/xml_launch/easy_launch_demo)

# Launching MoveIt using XML

As you may know MoveIt's launch files and config are crazy complex.
For this talk I converted MoveIt's launch from hundreds of lines of Python to less than 50 lines of xml.
I also took the dozen or so config files and reduced them to just these:

- `moveit.yaml` <- all the moveit config parameters
- `ros2_control.yaml` <- config for ros2_control
- `urdf` <- robot description
- `srdf` <- semantic robot description

#### Use the types

Due to an issue with how parameter value strings are parsed in the xml launch system the SRDF didn't parse at first.
I had an ugly work-around that involved modifying the xml files, but [G.A. vd. Hoorn](https://github.com/gavanderhoorn) came to my rescue and [posted this in github](https://github.com/ros2/launch/issues/729#issuecomment-1743445571).

The trick to string parameters is to put a `type="str"` attribute in the string parameters and they work.

## References:

- [ROS 2 XML Launch Docs](https://docs.ros.org/en/rolling/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html)
- [Comparing Python/XML/YAML](https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html)

## The launch file

```xml
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
```

## The MoveIt Config

```yaml
/**:
  ros__parameters:
    default_planning_pipeline: ompl
    planning_pipelines:
      - ompl
      - stomp
      - pilz_industrial_motion_planner
    ompl:
      planning_plugin: ompl_interface/OMPLPlanner
      start_state_max_bounds_error: 0.1
      jiggle_fraction: 0.05
      request_adapters: >-
          default_planner_request_adapters/AddTimeOptimalParameterization
          default_planner_request_adapters/ResolveConstraintFrames
          default_planner_request_adapters/FixWorkspaceBounds
          default_planner_request_adapters/FixStartStateBounds
          default_planner_request_adapters/FixStartStateCollision
          default_planner_request_adapters/FixStartStatePathConstraints
    stomp:
      planning_plugin: stomp_moveit/StompPlanner
      request_adapters: >-
        default_planner_request_adapters/AddTimeOptimalParameterization
        default_planner_request_adapters/ResolveConstraintFrames
        default_planner_request_adapters/FixWorkspaceBounds
        default_planner_request_adapters/FixStartStateBounds
        default_planner_request_adapters/FixStartStateCollision
        default_planner_request_adapters/FixStartStatePathConstraints
      stomp_moveit:
        num_timesteps: 60
        num_iterations: 40
        num_iterations_after_valid: 0
        num_rollouts: 30
        max_rollouts: 30
        exponentiated_cost_sensitivity: 0.8
        control_cost_weight: 0.1
        delta_t: 0.1
    pilz_industrial_motion_planner:
      planning_plugin: pilz_industrial_motion_planner/CommandPlanner
      request_adapters: ""
      default_planner_config: PTP
      capabilities: >-
          pilz_industrial_motion_planner/MoveGroupSequenceAction
          pilz_industrial_motion_planner/MoveGroupSequenceService
    robot_description_kinematics:
      manipulator:
        kinematics_solver: "kdl_kinematics_plugin/KDLKinematicsPlugin"
        kinematics_solver_search_resolution: 0.005
        kinematics_solver_timeout: 0.005
    robot_description_planning:
      cartesian_limits:
        max_trans_vel: 1.0
        max_trans_acc: 2.25
        max_trans_dec: -5.0
        max_rot_vel: 1.57
      default_velocity_scaling_factor: 0.1
      default_acceleration_scaling_factor: 0.1
      joint_limits:
        joint_1:
          has_velocity_limits: true
          max_velocity: 1.3963000000000001
          has_acceleration_limits: true
          max_acceleration: 8.6
        joint_2:
          has_velocity_limits: true
          max_velocity: 1.3963000000000001
          has_acceleration_limits: true
          max_acceleration: 8.6
        joint_3:
          has_velocity_limits: true
          max_velocity: 1.3963000000000001
          has_acceleration_limits: true
          max_acceleration: 8.6
        joint_4:
          has_velocity_limits: true
          max_velocity: 1.3963000000000001
          has_acceleration_limits: true
          max_acceleration: 8.6
        joint_5:
          has_velocity_limits: true
          max_velocity: 1.2218
          has_acceleration_limits: true
          max_acceleration: 8.6
        joint_6:
          has_velocity_limits: true
          max_velocity: 1.2218
          has_acceleration_limits: true
          max_acceleration: 8.6
        joint_7:
          has_velocity_limits: true
          max_velocity: 1.2218
          has_acceleration_limits: true
          max_acceleration: 8.6
        robotiq_85_left_knuckle_joint:
          has_velocity_limits: true
          max_velocity: 0.5
          has_acceleration_limits: true
          max_acceleration: 1.0
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
    moveit_simple_controller_manager:
      controller_names:
        - joint_trajectory_controller
        - robotiq_gripper_controller
      joint_trajectory_controller:
        type: FollowJointTrajectory
        joints:
          - joint_1
          - joint_2
          - joint_3
          - joint_4
          - joint_5
          - joint_6
          - joint_7
        action_ns: follow_joint_trajectory
        default: true
      robotiq_gripper_controller:
        type: GripperCommand
        joints:
          - robotiq_85_left_knuckle_joint
        action_ns: gripper_cmd
        default: true
```
