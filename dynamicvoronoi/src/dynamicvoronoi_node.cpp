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
#include <dynamicvoronoi/voronoi_base.h>

class VoronoiProcess
{
public:
  VoronoiProcess(ros::NodeHandle* nodehandle): n_(*nodehandle), loader_("dynamicvoronoi", "dynamicvoronoi::BaseVoronoi")
  {

    //n_ = ros::NodeHandle("~");
    
    int rate = 20;
    
    try
    {
      generator_ = loader_.createInstance("dynamicvoronoi::BoostPlugin");
    }
    catch(const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
      ros::shutdown();
    }

    ROS_INFO_STREAM("Standalone voronoi generator:" << "dynamicvoronoi::BoostPlugin" << " loaded.");


    std::string node_ns = ros::this_node::getNamespace();
    ros::NodeHandle costmap_nh(node_ns+"/move_base/local_costmap");
    double width = 10;
    costmap_nh.param("width", width, width);
    double height = 10;
    costmap_nh.param("height", height, height);
    double resolution = 0.05;
    costmap_nh.param("resolution", resolution, resolution);
    unsigned int width_cell = (unsigned int) (width / resolution);
    unsigned int height_cell = (unsigned int)(height / resolution);

    
    if (generator_)
    {
      generator_->initialize(n_);
      generator_->setCostmapTopic(node_ns+"/move_base/local_costmap/costmap", width_cell, height_cell, resolution);
      generator_->startWorker(ros::Rate(double(rate)), true);
    }
   
  }
  

private:
  pluginlib::ClassLoader<dynamicvoronoi::BaseVoronoi> loader_;
  boost::shared_ptr<dynamicvoronoi::BaseVoronoi> generator_;

  ros::NodeHandle n_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "voronoi_node");

  ros::NodeHandle nh;

  VoronoiProcess process(&nh);

  ros::spin();
  return 0;
}