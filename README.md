# distributed_teb_multi_robot
 A distributed TEB planner for multi-robot trajectory planning

Video on YouTube:

[![Video](https://img.youtube.com/vi/LnB0i0jpYLM/0.jpg)](https://www.youtube.com/watch?v=LnB0i0jpYLM "Video")



Launch:

1. For multi-robot systems, each robot should has its own namespace. 
2. In the move_base node, load the setting for teb_local_planner. A reference file is uploaded. It may need some tuning.
3. The robot_scan_matcher remove the robot footprint from LaserScan, and output a PointCloud2. (optional if there is no sensor)
4. The msg_merger receives other robots' trajectories.

Example
```xml
<?xml version="1.0"?>

<launch>
	
  <group ns="$(arg robot_name)">
     <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
      
      <!-- other settings such as costmap settings -->
      
      <rosparam file="$(find your_robot_package)/cfg/move_base_config/teb_local_planner_params.yaml" command="load" />
      <param name="TebLocalPlannerROS/map_frame" value="$(arg global_frame)"/>
      <param name="TebLocalPlannerROS/odom_topic" value="$(arg odom_topic)"/>
      
    </node>
 
  	 <!-- robot_scan_matcher -->
    <include file="$(find robot_scan_matcher)/launch/robot_scan_matcher.launch">
  		  <arg name="scan_topic" value="base_scan"/>
  		  <arg name="global_frame" value="$(arg robot_name)/odom"/>
  	 </include>

	   <!-- message merger -->
	   <include file="$(find msg_merger)/launch/msg_merger.launch">
    </include>
 
  </group>

</launch>
```
