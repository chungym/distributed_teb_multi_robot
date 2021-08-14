/*********************************************************************
*
* License
*
* Author: Yiu Ming Chung
* 
*********************************************************************/
#ifndef VORONOI_GRID_H_
#define VORONOI_GRID_H_
#include <dynamicvoronoi/voronoi_base.h>

// dynamic reconfigure
//#include <dynamicvoronoi/dynamicvoronoiConfig.h>
//#include <dynamic_reconfigure/server.h>
#include <dynamicvoronoi/dynamicvoronoi.h>


namespace dynamicvoronoi 
{
  class GridPlugin : public dynamicvoronoi::BaseVoronoi
  {
    public:
      GridPlugin() : voronoi_()
      {
        initialized_ = false;
      }


      /**
      * @brief Initializes the matcher plugin
      * @param nh ROS node handle
      * @param costmap_topic topic of thte occupancy grid of the local costmap
      */
      void initialize(ros::NodeHandle nh);

      void setCostmapTopic(std::string costmap_topic, int sizeX, int sizeY, double resolution);

      void occupancyGridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
      
      void compute();


    private:

      ros::NodeHandle nh_;

      bool initialized_;  //!< Keeps track about the correct initialization of this class

      std::string costmap_topic_;

      ros::Subscriber occupancyGrid_sub_; //!< Subscriber

      ros::Publisher paths_pub_;

      DynamicVoronoi voronoi_;

  };


};
#endif
