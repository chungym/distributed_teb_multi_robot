/*********************************************************************
*
* Author: Yiu-Ming Chung (yiu-ming.chung@tu-dortmund.de)
*
* 
* 
*********************************************************************/

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