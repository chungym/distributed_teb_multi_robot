/*
* BSD 3-Clause License
*
* Copyright (c) 2021, chungym
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <robot_scan_matcher/robot_scan_matcher_base.h>

#include <tf2_ros/transform_listener.h>

class MatcherStandalone
{
public:
  MatcherStandalone(ros::NodeHandle* nodehandle, tf2_ros::Buffer& tf) : 
  n_(*nodehandle), robot_scan_matcher_loader("robot_scan_matcher", "robot_scan_matcher_base::BaseMatcher"), tf_(tf)
  {

    std::string node_ns = ros::this_node::getNamespace(); //this is the robot name in most cases

    n_ = ros::NodeHandle("~");
    
    ros::NodeHandle teb_nh(node_ns+"/move_base/TebLocalPlannerROS");
    ros::NodeHandle costmap_nh(node_ns+"/move_base/local_costmap");
    
    // load converter plugin from parameter server, otherwise set default

    std::string matcher_plugin = "robot_scan_matcher_plugins::NavieMatcher";
    
    std::string global_frame = "robot_odom";
    costmap_nh.param("global_frame", global_frame, global_frame);
    std::string robot_base_frame = "robot_base_link";
    costmap_nh.param("robot_base_frame", robot_base_frame, robot_base_frame);
    double width = 15;
    costmap_nh.param("width", width, width);
    double height = 15;
    costmap_nh.param("height", height, height);

    std::string scan_topic = "front_laser/scan";
    n_.param("scan_topic", scan_topic, scan_topic);
    std::string obstacle_topic = "move_base/TebLocalPlannerROS/dynamic_obstacles";
    n_.param("obstacle_topic", obstacle_topic, obstacle_topic);

    scan_topic = node_ns + "/" + scan_topic;
    obstacle_topic = node_ns + "/" + obstacle_topic;

    int rate = 10;
    n_.param("rate", rate, rate);


    try
    {
      matcher_ = robot_scan_matcher_loader.createInstance(matcher_plugin);
    }
    catch(const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      ros::shutdown();
    }

    ROS_INFO_STREAM("Standalone robot_scan_matcher:" << matcher_plugin << " loaded.");

    if (matcher_)
    {
      matcher_->initialize(n_, &tf, global_frame, robot_base_frame,
                            scan_topic, width, height,
                            obstacle_topic);
      matcher_->startWorker(ros::Rate(double(rate)), true);
    }

  }

private:
  pluginlib::ClassLoader<robot_scan_matcher_base::BaseMatcher> robot_scan_matcher_loader;
  boost::shared_ptr<robot_scan_matcher_base::BaseMatcher> matcher_;

  ros::NodeHandle n_;
  tf2_ros::Buffer& tf_;


  std::string frame_id_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_scan_matcher");

  ros::NodeHandle nh;

  tf2_ros::Buffer buffer(ros::Duration(5));
  tf2_ros::TransformListener tf(buffer);

  MatcherStandalone match_process(&nh, buffer);

  ros::spin();

  return 0;
}