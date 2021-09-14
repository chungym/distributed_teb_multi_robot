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

#include <dynamicvoronoi/voronoi_boost_plugin.h>


#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(dynamicvoronoi::BoostPlugin, dynamicvoronoi::BaseVoronoi)

namespace dynamicvoronoi 
{

  BoostPlugin::~BoostPlugin()
  {

  }

  void BoostPlugin::initialize(ros::NodeHandle nh)
  {
    if(!initialized_)
    {

        std::string ns = "";
        {
          using namespace ros;
          ns = this_node::getNamespace();
        }
        
        voronoi_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(ns+"/voronoi",1);

        previous_grid_ = nav_msgs::OccupancyGrid();


        initialized_ = true;
        ROS_DEBUG("voronoi plugin initialized.");
    }
    else
    {
        ROS_WARN("voronoi local plugin has already been initialized, doing nothing.");
    }
  }

  void BoostPlugin::occupancyGridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {   

    boost::mutex::scoped_lock l(og_mutex_);
    current_grid_ = * msg;

  }

  void BoostPlugin::setCostmap2D(costmap_2d::Costmap2D *costmap, std::string global_frame_id)
  {
    global_frame_ = global_frame_id;

    if (!costmap)
    return;

    voronoi_.setCostmap2D(costmap, global_frame_id);

    useCostmap_ = true;
    voronoi_initialized_ = true;
    
  }

  void BoostPlugin::setCostmapTopic(std::string costmap_topic, int sizeX, int sizeY, double resolution)
  {
    occupancyGrid_sub_ = nh_.subscribe(costmap_topic, 1, &BoostPlugin::occupancyGridCB, this);

    voronoi_.disableCostmap2D();
    useCostmap_ = false;

    voronoi_initialized_ = true;
  }


  void BoostPlugin::compute()
  {

    timespec time1;
    timespec time2;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
    if(useCostmap_)
    {
      voronoi_.updateGridFromCostmap();
    }
    else
    {
      voronoi_.setOccupancyGrid(boost::make_shared<nav_msgs::OccupancyGrid>(current_grid_));
    }


    //voronoi_.addCircularObstacle(25,25, 5);
    //voronoi_.addPillObstacle(Eigen::Vector2d(50,50), Eigen::Vector2d(75, 75), 5);


    voronoi_.updateGraph();

    voronoi_pub_.publish(voronoi_.getVisualisation());

    Eigen::Vector2d start(current_grid_.info.origin.position.x+1, current_grid_.info.origin.position.y+1);
    Eigen::Vector2d goal = start + Eigen::Vector2d(current_grid_.info.width/current_grid_.info.resolution*2/3, current_grid_.info.height/current_grid_.info.resolution*2/3);

    boost::optional<std::pair<teb_local_planner::HcGraphVertexType, teb_local_planner::HcGraphVertexType>> start_goal
      = voronoi_.addStartAndGoal(start, goal); 

    int count=0;
    int* countPtr = &count;

    if (start_goal)
    {
      teb_local_planner::HcGraphVertexType start_vtx, goal_vtx;
      start_vtx = (*start_goal).first;
      goal_vtx = (*start_goal).second;

      teb_local_planner::HcGraph graph = *voronoi_.getGraph();
      std::vector<teb_local_planner::HcGraphVertexType> visited;
      visited.push_back(start_vtx);
      bool* visited_array = new bool[boost::num_vertices(graph)]();
      visited_array[start_vtx] = true;

      paths_world.clear();

      DepthFirst( graph, visited, visited_array, goal_vtx, 500);

      delete[] visited_array;
      visited_array = NULL; 
      
    }


    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
    timespec time_diff = timespec_diff(&time2, &time1);
    double run_time = (time_diff.tv_nsec)*1e-6; // milliseconds

    ROS_INFO_STREAM("Total runtime: "<<run_time<<"ms. Total paths: "<< paths_world.size());


  }



  void BoostPlugin::DepthFirst(teb_local_planner::HcGraph& g, std::vector<teb_local_planner::HcGraphVertexType>& visited, bool*& visited_array, const teb_local_planner::HcGraphVertexType& goal, int depth)
  {
    // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths

    //if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    //  return; // We do not need to search for further possible alternative homotopy classes.

    teb_local_planner::HcGraphVertexType back = visited.back();

    /// Examine adjacent nodes
    teb_local_planner::HcGraphAdjecencyIterator it, end;
    for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
    {
      //if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() )
      // constant time access
      if ( visited_array[*it] )
        continue; // already visited

      if ( *it == goal ) // goal reached
      {
        visited.push_back(*it);

        // Add new TEB, if this path belongs to a new homotopy class
        //hcp_->addAndInitNewTeb(visited.begin(), visited.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
        //                       start_orientation, goal_orientation, start_velocity);
        
        std::vector<std::pair<double, double>> path_world;

        for (auto it_v = visited.begin(); it_v != visited.end(); it_v++)
        {
          path_world.push_back( std::pair<double, double>(g[*it_v].pos.x(), g[*it_v].pos.y()) );
        }

        paths_world.push_back(path_world);

        visited.pop_back();
        break;
      }
    }

    if (depth > 0)
    {
      /// Recursion for all adjacent vertices
      for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
      {
        if ( visited_array[*it] || *it == goal)
          continue; // already visited || goal reached


        visited.push_back(*it);
        visited_array[*it] = true;

        // recursion step
        DepthFirst(g, visited, visited_array, goal, depth-1);

        visited.pop_back();
        visited_array[*it] = false;
      }
    }
  }


  std::vector<std::vector<std::pair<double, double> >> BoostPlugin::getPaths_world()
  {
    return paths_world;
  }


}

