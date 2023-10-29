+++
title = "ROSCON 2023 - Parameters Should be Boring"
date = 2023-10-20
type = "post"
description = "generate_parameter_library"
in_search_index = true
[taxonomies]
tags = ["Talks"]
+++

- [Slides PDF](/pdf/roscon23_parameters.pdf)
- [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)

## ROS Parameter References

- [Parameter Concept](https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html)
- [Understanding ROS 2 Parameters Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Using parameters in a class (C++)](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Using parameters in a class (Python)](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [Migrating Launch Files](https://docs.ros.org/en/rolling/How-To-Guides/Migrating-from-ROS1/Migrating-Launch-Files.html)
- [Migrating Parameters](https://docs.ros.org/en/rolling/How-To-Guides/Migrating-from-ROS1/Migrating-Parameters.html)
- [parameter_blackboard source](https://github.com/ros2/demos/blob/rolling/demo_nodes_cpp/src/parameters/parameter_blackboard.cpp)
- [Parameter API design in ROS](https://design.ros2.org/articles/ros_parameters.html)

## Elsie

The slides for this presentation were generated using a python library called [Elsie](https://spirali.github.io/elsie/).
This is my second presentation I've created using this library and I am a huge fan.
Here is a small snippet from the source of this presntation showing how I defined the header of each slide and the layout of the slides with code blocks.

```python
def logo_header_slide(parent: Box, title: str):
    parent.box(x=1570, y=40).image("picknik_logo.png")
    parent.sbox(name="header", x=0, height=140).fbox(p_left=20).text(
        title, elsie.TextStyle(bold=True)
    )
    return parent.fbox(name="content", p_left=20, p_right=20)


def code_slide(parent: Box, title: str, language: str, code: str):
    content = logo_header_slide(parent, title)
    code_bg = "#F6F8FA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    box.overlay()
       .box(x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20)
       .code(language, code)
```

You can find the [full source of the talk slides here](https://github.com/tylerjw/tylerjw.dev/blob/main/slides/roscon23_parameters.py).
