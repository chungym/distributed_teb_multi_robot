/*********************************************************************
*
* Author: Yiu-Ming Chung (yiu-ming.chung@tu-dortmund.de)
*
* 
* 
*********************************************************************/

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