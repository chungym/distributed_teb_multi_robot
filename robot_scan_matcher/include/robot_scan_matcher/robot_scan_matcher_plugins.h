/*********************************************************************
*
* License
*
* Author: Yiu Ming Chung
* 
*********************************************************************/
#ifndef ROBOT_SCAN_MATCHER_PLUGINS_H_
#define ROBOT_SCAN_MATCHER_PLUGINS_H_
#include <robot_scan_matcher/robot_scan_matcher_base.h>

#include <ros/ros.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

// message types
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/ObstacleWithTrajectory.h>
#include <teb_local_planner/ObstacleWithTrajectoryArray.h>

// boost classes
#include <boost/shared_ptr.hpp>

// dynamic reconfigure
#include <robot_scan_matcher/robot_scan_matcherConfig.h>
#include <dynamic_reconfigure/server.h>

namespace robot_scan_matcher_plugins 
{
  class NavieMatcher : public robot_scan_matcher_base::BaseMatcher
  {
    public:
      NavieMatcher()
      {
        initialized_ = false;
        tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
      }

    struct Parameters
    {
      Parameters() : enable_filtering_(true), rate_(10), removal_distance_(0.1), translation_threshold_(0.1) {}

      bool enable_filtering_; //!< enable the matcher to filter laser scan
      int rate_; //!< rate of processing
      double removal_distance_; //!< distance in meter between scan points and robot model in which removal of points is done
      double translation_threshold_; //!< allowed distance in meter for the matching
    };

    /**
    * @brief Initializes the matcher plugin
    * @param name The name of the instance
    * @param tf Pointer to a tf buffer
    * @param laserScan_topic laserScan topic (required since TEB planner is not directly exposed to laserscan)
    * @param costmap_SizeInMetersX size of the costmap in X, only scan points within this region is considered
    * @param costmap_SizeInMetersY size of the costmap in Y, only scan points within this region is considered
    * @param obstacles_raw un-altered obstacles array from TEB planner
    * @param parent_obst_mutex_ mutex from TEB planner
    */
      void initialize(ros::NodeHandle nh, tf2_ros::Buffer* tf, std::string global_frame, std::string local_frame,
                      std::string laserScan_topic, double costmap_SizeInMetersX, double costmap_SizeInMetersY,
                      std::string obst_topic);

      void laserScanCB(const sensor_msgs::LaserScan::ConstPtr& laserScan_msg);

      void obstSubCB(const teb_local_planner::ObstacleWithTrajectoryArray::ConstPtr& obst_msg);

      /**
       * @brief This method performs the actual work
       */
      void compute();

      teb_local_planner::ObstaclePtr createObstacleFromMsg(teb_local_planner::ObstacleWithTrajectory& curr_obstacle);

      Parameters parameter_;          //< active parameters throughout computation
      Parameters parameter_buffered_; //< the buffered parameters that are offered to dynamic reconfigure
      boost::mutex parameter_mutex_;  //!< Mutex that keeps track about the ownership of the shared polygon instance

    private:

      void reconfigureCB(robot_scan_matcher::robot_scan_matcherConfig& config, uint32_t level);

      bool initialized_;  //!< Keeps track about the correct initialization of this class

      tf2_ros::Buffer* tf_; //!< pointer to tf buffer
      std::string global_frame_;
      std::string local_frame_;
      tf2_ros::Buffer tfBuffer_;
      tf2_ros::TransformListener* tfListener_;

      message_filters::Subscriber<sensor_msgs::LaserScan> laserScan_sub_; //!< Subscriber for laserScan since TEB planner is not directly exposed to laserscan
      tf2_ros::MessageFilter<sensor_msgs::LaserScan>* laser_filter_;
      boost::mutex laserScan_mutex_;
      double costmap_SizeInMetersX_;
      double costmap_SizeInMetersY_;

      laser_geometry::LaserProjection projector_;
      sensor_msgs::PointCloud2 cloud_; 
      sensor_msgs::PointCloud2 cloud_filtered_; 
      ros::Publisher cloud_pub_;
      ros::Publisher scan_pub_;

      boost::mutex obst_mutex_; //!< mutex for copying obstacles from parent node
      ros::Subscriber obst_msg_sub_;
      //tf2_ros::MessageFilter<teb_local_planner::ObstacleWithTrajectoryArray>* obst_filter_;
      teb_local_planner::ObstacleWithTrajectoryArray obstacles_raw_;
      teb_local_planner::ObstacleWithTrajectoryArray obstacles_filtered_;

      dynamic_reconfigure::Server<robot_scan_matcher::robot_scan_matcherConfig>* dynamic_recfg_;

  };

};
#endif
