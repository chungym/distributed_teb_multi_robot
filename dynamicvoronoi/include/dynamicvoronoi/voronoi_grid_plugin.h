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
