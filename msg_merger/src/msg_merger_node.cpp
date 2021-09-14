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
#include <msg_merger/msg_merger_base.h>

class MergerStandalone
{
public:
  MergerStandalone(ros::NodeHandle* nodehandle): n_(*nodehandle), loader_("msg_merger", "msg_merger_base::BaseMerger")
  {

    n_ = ros::NodeHandle("~");

    int rate = 10;
    
    try
    {
      merger_ = loader_.createInstance("msg_merger_plugin::MergerPlugin");
    }
    catch(const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      ros::shutdown();
    }

    ROS_INFO_STREAM("Standalone msg_merger:" << "msg_merger::MergerPlugin" << " loaded.");

    
    if (merger_)
    {
      merger_->initialize(n_);
      merger_->startWorker(ros::Rate(double(rate)), true);
    }
   
  }
  

private:
  pluginlib::ClassLoader<msg_merger_base::BaseMerger> loader_;
  boost::shared_ptr<msg_merger_base::BaseMerger> merger_;

  ros::NodeHandle n_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_merger_node");

  ros::NodeHandle nh;

  MergerStandalone process(&nh);

  ros::spin();
  return 0;
}