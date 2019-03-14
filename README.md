# rviz_robot_plugins

Plugin to control a robot using sliders in rviz. 



## Demo
``` 
roscore
rosrun rviz rviz (seperal terminal)
``` 
In Rviz add the panel via: Panels->Add New Panel -> rviz_plugin_tutorials -> RobotControl

## Errors

**First notice that when creating a custom plugin change name in**

* CMakeLists.txt 
* package.xml
* plugin_description.xml
* AND in .cpp and .h file (the namespaces and the line: PLUGINLIB_EXPORT_CLASS(rviz_robot_plugins::ImuDisplay,rviz::Display ))
* Otherwise when using the plugin: 

``` 
This error might occur:
[ERROR] [1552290294.550873921]: PluginlibFactory: The plugin for class 'rviz_robot_plugins/Imu' failed to load.  Error: MultiLibraryClassLoader: Could not create class of type
rviz_robot_plugins::ImuDisplay
``` 

**In case of not that the plugin is not visible under rviz->panels**

* catkin clean
* catkin build
* source ws

## Types:
in Rviz there are different types (and they are added differntly)
* Panel (new panel...)
* Tool  (flag etc.)
* Display with main add button bottom left)
* additionally [https://github.com/PickNikRobotics/rviz_visual_tools](rviz_visual_tools) describes a RvizVisualToolsGui Panel

Related project for a [drive plugin](https://answers.ros.org/question/209325/is-there-a-graphical-virtual-joypad/)

## License BSD
If you want to use this package please contact: [me](https://simact.de/about_me).

Respect the License of  Author: Dave Coleman who is the author of a similar package [rviz_visual_tools
](https://github.com/PickNikRobotics/rviz_visual_tools) that was used as a start point for this package.