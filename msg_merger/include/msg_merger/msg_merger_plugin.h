/*********************************************************************
*
* License
*
* Author: Yiu Ming Chung
* 
*********************************************************************/
#ifndef MSG_MERGER_PLUGIN_H_
#define MSG_MERGER_PLUGIN_H_

#include <ros/ros.h>
#include <msg_merger/msg_merger_base.h>
#include <boost/shared_ptr.hpp>
#include <teb_local_planner/ObstacleWithTrajectoryArray.h>

namespace msg_merger_plugin
{
  class MergerPlugin  : public msg_merger_base::BaseMerger
  {
    public:

      MergerPlugin(): initialized_(false)
      {
      }

      void initialize(ros::NodeHandle nh);

      void compute();
      void compute2();

      void obstSubCB
      (
        const teb_local_planner::ObstacleWithTrajectory::ConstPtr& obst_msg,
        teb_local_planner::ObstacleWithTrajectory::Ptr& output
      );

    private:

      bool initialized_;  //!< Keeps track about the correct initialization of this class
      boost::mutex obst_subs_array_mutex_;
      ros::NodeHandle nh_;

      std::vector<std::pair< boost::shared_ptr<ros::Subscriber>, teb_local_planner::ObstacleWithTrajectory::Ptr>> obst_subs_array_;

      teb_local_planner::ObstacleWithTrajectoryArray obst_array_;

      ros::Publisher obst_array_pub_;
      
      ros::master::V_TopicInfo topic_infos_;

  };
}
#endif
