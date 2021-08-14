/*********************************************************************
*
* Author: Yiu-Ming Chung (yiu-ming.chung@tu-dortmund.de)
*
*
* 
*********************************************************************/

#include <robot_scan_matcher/robot_scan_matcher_plugins.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(robot_scan_matcher_plugins::NavieMatcher, robot_scan_matcher_base::BaseMatcher)

namespace robot_scan_matcher_plugins 
{

  void NavieMatcher::initialize(ros::NodeHandle nh, tf2_ros::Buffer* tf, std::string global_frame, std::string local_frame,
                                std::string laserScan_topic, double costmap_SizeInMetersX, double costmap_SizeInMetersY,
                                std::string obst_topic)
  {
    if(!initialized_)
    {

        tf_ = tf;
        global_frame_ = global_frame;
        local_frame_ = local_frame;
        
        ROS_INFO_STREAM("LaserScan is transformed to planner frame: "<< global_frame_);

        laserScan_sub_.subscribe(nh, laserScan_topic, 2);
        laser_filter_ = new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(laserScan_sub_, tfBuffer_, global_frame_, 5, 0);
        laser_filter_->registerCallback( boost::bind(&NavieMatcher::laserScanCB, this, _1) );

        obst_msg_sub_ = nh.subscribe(obst_topic, 1, &NavieMatcher::obstSubCB, this);


        costmap_SizeInMetersX_ = costmap_SizeInMetersX;
        costmap_SizeInMetersY_ = costmap_SizeInMetersY;

        std::string ns = "";
        {
          using namespace ros;
          ns = this_node::getNamespace();
        }
        
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ns+"/PointCloud_filtered",5);
        scan_pub_ = nh.advertise<sensor_msgs::LaserScan>(ns+"/base_scan_filtered",5);

        nh.param("enable_filtering", parameter_.enable_filtering_, true);
        nh.param("removal_distance", parameter_.removal_distance_, 0.1);
        nh.param("translation_threshold", parameter_.translation_threshold_, 0.1);
        
        parameter_buffered_ = parameter_;
        
        // setup dynamic reconfigure
        dynamic_recfg_ = new dynamic_reconfigure::Server<robot_scan_matcher::robot_scan_matcherConfig>(nh);
        dynamic_reconfigure::Server<robot_scan_matcher::robot_scan_matcherConfig>::CallbackType cb = boost::bind(&NavieMatcher::reconfigureCB, this, _1, _2);
        dynamic_recfg_->setCallback(cb);

        initialized_ = true;
        ROS_DEBUG("robot_scan_matcher plugin initialized.");
    }
    else
    {
        ROS_WARN("robot_scan_matcher plugin has already been initialized, doing nothing.");
    }
  }


  void NavieMatcher::laserScanCB(const sensor_msgs::LaserScan::ConstPtr& laserScan_msg)
  {   


        boost::mutex::scoped_lock l(laserScan_mutex_);

        try
        {
        
        sensor_msgs::LaserScan temp;
        temp = *laserScan_msg;

        // remove any leading slash
        if('/' == temp.header.frame_id.front())
        {
          temp.header.frame_id.erase(temp.header.frame_id.begin());
        }

        scan_pub_.publish(temp);

        projector_.transformLaserScanToPointCloud(global_frame_, temp, cloud_, tfBuffer_, INFINITY, laser_geometry::channel_option::Intensity);
        cloud_.header.stamp = temp.header.stamp;

        }
        catch (tf::TransformException& e)
        {
          std::cout << e.what();
          return;
        }
  }

  void NavieMatcher::obstSubCB(const teb_local_planner::ObstacleWithTrajectoryArray::ConstPtr& obst_msg)
  {   
        boost::mutex::scoped_lock l(obst_mutex_);
        obstacles_raw_ = *obst_msg;
  }


  void NavieMatcher::compute()
  {

    { //copy from dynamic reconfigure
      boost::mutex::scoped_lock lock(parameter_mutex_);
      parameter_ = parameter_buffered_;
    }

    pcl::PointCloud<pcl::PointXYZI> pclpc;

    boost::mutex::scoped_lock l(laserScan_mutex_);
    // Convert to PCL data type

    
    pcl::fromROSMsg(cloud_, pclpc);


    boost::mutex::scoped_lock l_obst(obst_mutex_);

    

    if (!obstacles_raw_.obstacles.empty())
    {
        for (size_t i=0; i<obstacles_raw_.obstacles.size(); ++i)
        {
            teb_local_planner::ObstacleWithTrajectory& curr_obstacle =  obstacles_raw_.obstacles.at(i); 

            teb_local_planner::ObstaclePtr obstPtr = createObstacleFromMsg(curr_obstacle);

            if (parameter_.enable_filtering_)
            {
              auto it = pclpc.begin();
              while (it != pclpc.end())
              {

                  if (obstPtr->getMinimumDistance(Eigen::Vector2d(it->x, it->y)) <= parameter_.removal_distance_)
                  {
                    it = pclpc.erase(it);
                  }
                  else
                  {
                    it++;
                  }
                  
              }
            }

        }

    }

    pcl::toROSMsg(pclpc, cloud_filtered_);
    cloud_filtered_.header.stamp = cloud_.header.stamp;
    
    cloud_pub_.publish(cloud_filtered_);
    
  }

  teb_local_planner::ObstaclePtr NavieMatcher::createObstacleFromMsg(teb_local_planner::ObstacleWithTrajectory& obstacleMsg)
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    geometry_msgs::TransformStamped obstacle_to_map;
    try 
    {
      obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                              obstacleMsg.header.frame_id, ros::Time(0),
                                              obstacleMsg.header.frame_id, ros::Duration(0.1));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    double theta_offset = tf::getYaw(obstacle_to_map.transform.rotation);

    //time passed since the creation of message
    ros::Duration time_offset = ros::Time::now() - obstacleMsg.header.stamp; 
    
    //make a copy from the original message, otherwise will be recursely offset in incorrect way
    std::vector<teb_local_planner::TrajectoryPointSE2, std::allocator<teb_local_planner::TrajectoryPointSE2>> trajectory_copy (obstacleMsg.trajectory);

    //compute relative time from now, and transform pose
    for (unsigned int j = 0; j < trajectory_copy.size(); j++)
    {
      trajectory_copy.at(j).time_from_start -= time_offset;

      // T = T2*T1 = (R2 t2) (R1 t1) = (R2*R1 R2*t1+t2)
      //             (0   1) (0   1)   (0            1)
      trajectory_copy.at(j).pose.x = obstacleMsg.trajectory.at(j).pose.x*cos(theta_offset) - obstacleMsg.trajectory.at(j).pose.y*sin(theta_offset) 
                                    + obstacle_to_map.transform.translation.x;
      trajectory_copy.at(j).pose.y = obstacleMsg.trajectory.at(j).pose.x*sin(theta_offset) + obstacleMsg.trajectory.at(j).pose.y*cos(theta_offset) 
                                    + obstacle_to_map.transform.translation.y;
      trajectory_copy.at(j).pose.theta = obstacleMsg.trajectory.at(j).pose.theta + theta_offset;
    } 

    teb_local_planner::ObstaclePtr obst_ptr(NULL);
    if (obstacleMsg.footprint.type == teb_local_planner::ObstacleFootprint::CircularObstacle) // circle
    {
      teb_local_planner::PoseSE2 pose_init;
      Eigen::Vector2d speed_init;
      teb_local_planner::CircularObstacle* obstacle = new teb_local_planner::CircularObstacle;
      obstacle->setTrajectory(trajectory_copy, obstacleMsg.footprint.holonomic);
      obstacle->predictPoseFromTrajectory(0, pose_init, speed_init);
      obstacle->x() = pose_init.x();
      obstacle->y() = pose_init.y();
      obstacle->radius() = obstacleMsg.footprint.radius;
      obstacle->setCentroidVelocity(speed_init);
      obst_ptr = teb_local_planner::ObstaclePtr(obstacle);
    }
    else if (obstacleMsg.footprint.type == teb_local_planner::ObstacleFootprint::PointObstacle ) // point
    {
      teb_local_planner::PoseSE2 pose_init;
      Eigen::Vector2d speed_init;
      teb_local_planner::PointObstacle* obstacle = new teb_local_planner::PointObstacle;
      obstacle->setTrajectory(trajectory_copy, obstacleMsg.footprint.holonomic);
      obstacle->predictPoseFromTrajectory(0, pose_init, speed_init);
      obstacle->x() = pose_init.x();
      obstacle->y() = pose_init.y();
      obstacle->setCentroidVelocity(speed_init);
      obst_ptr = teb_local_planner::ObstaclePtr(obstacle);
    }
    else if (obstacleMsg.footprint.type == teb_local_planner::ObstacleFootprint::LineObstacle ) // line
    {
      Eigen::Vector2d start_robot(  obstacleMsg.footprint.point1.x,
                                    obstacleMsg.footprint.point1.y);
      Eigen::Vector2d end_robot(  obstacleMsg.footprint.point2.x,
                                  obstacleMsg.footprint.point2.y);
      teb_local_planner::PoseSE2 pose_init;
      Eigen::Vector2d speed_init;
      teb_local_planner::LineObstacle* obstacle = new teb_local_planner::LineObstacle;
      Eigen::Vector2d start;
      Eigen::Vector2d end;
      obstacle->setTrajectory(trajectory_copy, obstacleMsg.footprint.holonomic);
      obstacle->predictPoseFromTrajectory(0, pose_init, speed_init);
      obstacle->transformToWorld(pose_init, start_robot, end_robot, start, end);
      obstacle->setStart(start);
      obstacle->setEnd(end);
      obstacle->setCentroidVelocity(speed_init);
      obst_ptr = teb_local_planner::ObstaclePtr(obstacle);
    }
    else if (obstacleMsg.footprint.type == teb_local_planner::ObstacleFootprint::PillObstacle ) // pill
    {
      Eigen::Vector2d start_robot(  obstacleMsg.footprint.point1.x,
                                    obstacleMsg.footprint.point1.y);
      Eigen::Vector2d end_robot(  obstacleMsg.footprint.point2.x,
                                  obstacleMsg.footprint.point2.y);
      teb_local_planner::PoseSE2 pose_init;
      Eigen::Vector2d speed_init;
      teb_local_planner::PillObstacle* obstacle = new teb_local_planner::PillObstacle;
      Eigen::Vector2d start;
      Eigen::Vector2d end;
      obstacle->setTrajectory(trajectory_copy, obstacleMsg.footprint.holonomic);
      obstacle->predictPoseFromTrajectory(0, pose_init, speed_init);
      obstacle->transformToWorld(pose_init, start_robot, end_robot, start, end);
      obstacle->setStart(start);
      obstacle->setEnd(end);
      obstacle->setRadius(obstacleMsg.footprint.radius);
      obstacle->setCentroidVelocity(speed_init);
      obst_ptr = teb_local_planner::ObstaclePtr(obstacle);
    }
    else if(obstacleMsg.footprint.type == teb_local_planner::ObstacleFootprint::PolygonObstacle ) // polygon
    {
      teb_local_planner::Point2dContainer vertices_robocentric;
      for (size_t j=0; j<obstacleMsg.footprint.polygon.points.size(); ++j)
      {
        Eigen::Vector2d pos( obstacleMsg.footprint.polygon.points[j].x,
                              obstacleMsg.footprint.polygon.points[j].y);
        vertices_robocentric.push_back( pos );
      }

      teb_local_planner::PoseSE2 pose_init;
      Eigen::Vector2d speed_init;
      teb_local_planner::Point2dContainer vertices;
      teb_local_planner::PolygonObstacle* polyobst = new teb_local_planner::PolygonObstacle;
      polyobst->setTrajectory(trajectory_copy, obstacleMsg.footprint.holonomic);
      polyobst->predictPoseFromTrajectory(0, pose_init, speed_init);
      polyobst->setCentroidVelocity(speed_init);
      polyobst->transformToWorld(pose_init, vertices_robocentric, vertices);

      for (size_t j=0; j<vertices.size(); ++j)
      {
        polyobst->pushBackVertex( vertices.at(j) );
      }
      polyobst->finalizePolygon();
      obst_ptr = teb_local_planner::ObstaclePtr(polyobst);
    }
    else
    {
      ROS_WARN("Invalid custom obstacle received. Invalid Type. Skipping...");
    }

    return obst_ptr;

  }


  void NavieMatcher::reconfigureCB(robot_scan_matcher::robot_scan_matcherConfig& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(parameter_mutex_);
    parameter_buffered_.enable_filtering_ = config.enable_filtering;
    parameter_buffered_.removal_distance_ = config.removal_distance;
    parameter_buffered_.translation_threshold_ = config.removal_distance;
    parameter_buffered_.rate_ = config.rate;

    if (parameter_buffered_.rate_ != parameter_.rate_){worker_timer_.setPeriod(ros::Duration(1.0/parameter_buffered_.rate_), false);}
  }

}