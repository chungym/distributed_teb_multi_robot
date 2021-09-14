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

#include <dynamicvoronoi/voronoi_grid_plugin.h>

#include <visualization_msgs/Marker.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dynamicvoronoi::GridPlugin, dynamicvoronoi::BaseVoronoi)

namespace dynamicvoronoi 
{

  void GridPlugin::initialize(ros::NodeHandle nh)
  {
    if(!initialized_)
    {
      nh_ = nh;
      std::string ns = "";
      {
        using namespace ros;
        ns = this_node::getNamespace();
      }
      
      voronoi_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(ns+"/voronoi",1);
      paths_pub_ = nh_.advertise<visualization_msgs::Marker>(ns+"/voronoi_paths", 5);

      previous_grid_ = nav_msgs::OccupancyGrid();


      initialized_ = true;
      ROS_DEBUG("voronoi plugin initialized.");
    }
    else
    {
        ROS_WARN("voronoi local plugin has already been initialized, doing nothing.");
    }
  }

  void GridPlugin::setCostmapTopic(std::string costmap_topic, int sizeX, int sizeY, double resolution)
  {
    occupancyGrid_sub_ = nh_.subscribe(costmap_topic, 1, &GridPlugin::occupancyGridCB, this);

    voronoi_.initializeEmpty(sizeX, sizeY, true);
    voronoi_initialized_ = true;
  }

  void GridPlugin::occupancyGridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {   

    boost::mutex::scoped_lock l(og_mutex_);
    current_grid_ = * msg;

  }

  void GridPlugin::compute()
  {

    boost::mutex::scoped_lock l(og_mutex_);

    if (voronoi_initialized_ && current_grid_.header.stamp != ros::Time())
    {

      ROS_ASSERT_MSG(current_grid_.info.width == voronoi_.getSizeX() && current_grid_.info.height == voronoi_.getSizeY(), "Mismatched grid sizes.");

      if (current_grid_.info.width == voronoi_.getSizeX() && current_grid_.info.height == voronoi_.getSizeY())
      {
        
        double resolution = current_grid_.info.resolution;
        int dis_x_cell = int(round((current_grid_.info.origin.position.x - previous_grid_.info.origin.position.x) / resolution)); 
        int dis_y_cell = int(round((current_grid_.info.origin.position.y - previous_grid_.info.origin.position.y) / resolution)); 
        
        int source_x = 0;
        int source_y = 0; 
        int target_x = - dis_x_cell;
        int target_y = - dis_y_cell;
        int overlap_size_x = current_grid_.info.width + dis_x_cell;
        int overlap_size_y = current_grid_.info.height + dis_y_cell;
        if (dis_x_cell >= 0) 
        {
            source_x = dis_x_cell;
            target_x = 0;
            overlap_size_x = current_grid_.info.width - dis_x_cell;
        }
        if (dis_y_cell >= 0)
        {
            source_y = dis_y_cell;
            target_y = 0;
            overlap_size_y = current_grid_.info.height - dis_y_cell;
        }


        if ( (dis_x_cell != 0 || dis_y_cell != 0) && abs(dis_x_cell) < voronoi_.getSizeX() && abs(dis_y_cell) < voronoi_.getSizeY() )
        {
            voronoi_.move(source_x, source_y, target_x, target_y, overlap_size_x, overlap_size_y);
        }

        for (int x=0; x<current_grid_.info.width; x++)
        {
            for (int y=0; y<current_grid_.info.height; y++)
            {
                
                if (current_grid_.data[y*current_grid_.info.width+x] > 50)
                {
                    voronoi_.occupyCell(x,y);
                }
                else
                {
                    voronoi_.clearCell(x,y);
                }
            }
        }


        voronoi_.update();
        voronoi_.prune();

        previous_grid_ = current_grid_;
        

        nav_msgs::OccupancyGrid output = current_grid_;

        
        for(int y = voronoi_.getSizeY()-1; y >=0; y--)
        {      
            for(int x = 0; x<voronoi_.getSizeX(); x++)
            {	
                
                if (voronoi_.isVoronoi(x,y)) 
                {
                    output.data[y*output.info.width + x] = 100;
                } 
                else 
                {
                    output.data[y*output.info.width + x] = 0;
                }

            }
        }

        voronoi_pub_.publish(output);
        
        
        voronoi_.setMapInfo(current_grid_.info.resolution, current_grid_.info.origin.position.x, current_grid_.info.origin.position.y);

            
        voronoi_.createGraph();
        
        voronoi_.search(150,150, 200, 200);


        std::vector<std::vector<std::pair<double, double> >> paths_world = voronoi_.getPaths_world();

        
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = current_grid_.header.frame_id;
        line_strip.header.stamp = ros::Time::now();
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1;
        line_strip.color.a = 1.0;
        line_strip.color.b = 1.0;
        line_strip.lifetime = ros::Duration(0.15);

        int count = 0;

        for (auto it_path=paths_world.begin(); it_path!=paths_world.end(); it_path++)
        {
          
          count++;
          line_strip.ns = "path_"+ std::to_string(count);
          line_strip.points.clear();

          for (auto it_pt=it_path->begin(); it_pt!=it_path->end(); it_pt++)
          {
            geometry_msgs::Point p;
            p.x = it_pt->first;
            p.y = it_pt->second;
            p.z = 0;
            line_strip.points.push_back(p);
          }
          paths_pub_.publish(line_strip);
        }

        

      }
      
    }

  }

}