#ifndef VORONOI_COSTMAP_H_
#define VORONOI_COSTMAP_H_

#include <boost/functional/hash.hpp>


#include <dynamicvoronoi/voronoi_base.h>
#include <dynamicvoronoi/boost_voronoi.h>


#include <time.h>

namespace dynamicvoronoi 
{

  // sys/time.h
  timespec timespec_diff(timespec *a, timespec *b) {
      timespec result; 
      result.tv_sec  = a->tv_sec  - b->tv_sec;
      result.tv_nsec = a->tv_nsec - b->tv_nsec;
      if (result.tv_nsec < 0) {
          --result.tv_sec;
          result.tv_nsec += 1000000000L;
      }
      return result;
  }


  class BoostPlugin : public dynamicvoronoi::BaseVoronoi
  {
    public:
      BoostPlugin()
      {
        initialized_ = false;
        voronoi_initialized_ = false;
      }

      ~BoostPlugin();


      void compute();

      /**
      * @brief Initializes the matcher plugin
      * @param nh ROS node handle
      * @param costmap_topic topic of thte occupancy grid of the local costmap
      */
      void initialize(ros::NodeHandle nh);

      void occupancyGridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);

      void setCostmap2D(costmap_2d::Costmap2D *costmap, std::string global_frame_id);

      void setCostmapTopic(std::string costmap_topic, int sizeX, int sizeY, double resolution);

      teb_local_planner::HcGraph getGraph();

      std::vector<std::vector<std::pair<double, double> >> getPaths_world();

    private:


      void DepthFirst(teb_local_planner::HcGraph& g, std::vector<teb_local_planner::HcGraphVertexType>& visited, bool*& visited_array, const teb_local_planner::HcGraphVertexType& goal, int depth);

      ros::NodeHandle nh_;

      bool initialized_;  //!< Keeps track about the correct initialization of this class

      ros::Subscriber occupancyGrid_sub_;
      bool useCostmap_;



      std::string global_frame_;

      BoostVoronoi voronoi_;

      std::vector<std::vector<std::pair<double, double> >> paths_world;
  };
};




#endif
