/*********************************************************************
*
* Author: Yiu-Ming Chung (yiu-ming.chung@tu-dortmund.de)
*
* 
* 
*********************************************************************/

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