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
