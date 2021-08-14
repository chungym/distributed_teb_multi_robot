#include <dynamicvoronoi/boost_voronoi.h>

namespace dynamicvoronoi{
        
    BoostVoronoi::BoostVoronoi() {
        //some initialisation

        og_mutex_ = new boost::mutex();
        graph_mutex_ = new boost::mutex();

        cost_translation_table_ = new char[256];

        // special values:
        cost_translation_table_[0] = 0;  // NO obstacle
        cost_translation_table_[253] = 99;  // INSCRIBED obstacle
        cost_translation_table_[254] = 100;  // LETHAL obstacle
        cost_translation_table_[255] = -1;  // UNKNOWN

        // regular cost values scale the range 1 to 252 (inclusive) to fit
        // into 1 to 98 (inclusive).
        for (int i = 1; i < 253; i++)
        {
        cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
        }

        directCostmap_ = false;
        voronoi_initialized_ = true;
    }

    BoostVoronoi::~BoostVoronoi() {
        //some initialisation
        
        vd.clear();

        delete[] cost_translation_table_;
    }

    boost::mutex* BoostVoronoi::getMutex()
    {
        return graph_mutex_;
    }

    boost::mutex* BoostVoronoi::getGridMutex()
    {
        return og_mutex_;
    }

    void BoostVoronoi::setCostmap2D(costmap_2d::Costmap2D *costmap, std::string global_frame_id)
    {
        global_frame_ = global_frame_id;

        if (!costmap)
        return;

        costmap_ = costmap;     

        current_grid_ = boost::make_shared<nav_msgs::OccupancyGrid>();

        directCostmap_ = true;
    }

    void BoostVoronoi::disableCostmap2D()
    {
        costmap_ = NULL;
        directCostmap_ = false;
    }

    void BoostVoronoi::setOccupancyGrid(nav_msgs::OccupancyGrid::Ptr grid)
    {
        current_grid_ = grid;

        // change of map parameters is allowed
        sizeX_ = current_grid_->info.width;
        sizeY_ = current_grid_->info.height;
        resolution_ = current_grid_->info.resolution;
        origin_.x() = current_grid_->info.origin.position.x;
        origin_.y() = current_grid_->info.origin.position.y;
    }

    void BoostVoronoi::updateGridFromCostmap()
    {

        if (voronoi_initialized_ && directCostmap_)
        {
            // refer to costmap_2d_publisher.cpp
            // https://github.com/ros-planning/navigation/blob/noetic-devel/costmap_2d/src/costmap_2d_publisher.cpp

            costmap_2d::Costmap2D::mutex_t::scoped_lock l(*costmap_->getMutex());
            
            // may have external visualisation
            boost::mutex::scoped_lock l_og(*og_mutex_);

            double resolution = costmap_->getResolution();

            current_grid_->header.frame_id = global_frame_;
            current_grid_->header.stamp = ros::Time::now();
            current_grid_->info.resolution = resolution;

            current_grid_->info.width = costmap_->getSizeInCellsX();
            current_grid_->info.height = costmap_->getSizeInCellsY();

            double wx, wy;
            costmap_->mapToWorld(0, 0, wx, wy);
            current_grid_->info.origin.position.x = wx - resolution / 2;
            current_grid_->info.origin.position.y = wy - resolution / 2;
            current_grid_->info.origin.position.z = 0.0;
            current_grid_->info.origin.orientation.w = 1.0;
            //saved_origin_x_ = costmap_->getOriginX();
            //saved_origin_y_ = costmap_->getOriginY();

            current_grid_->data.resize(current_grid_->info.width * current_grid_->info.height);

            unsigned char* data = costmap_->getCharMap();
            for (unsigned int i = 0; i < current_grid_->data.size(); i++)
            {
                current_grid_->data[i] = cost_translation_table_[ data[ i ]];
            }

            // change of map parameters is allowed
            sizeX_ = current_grid_->info.width;
            sizeY_ = current_grid_->info.height;
            resolution_ = current_grid_->info.resolution;
            origin_.x() = current_grid_->info.origin.position.x;
            origin_.y() = current_grid_->info.origin.position.y;

        }
    }



    void BoostVoronoi::updateGraph()
    {

        try
        {

            boost::mutex::scoped_lock l(*graph_mutex_);

            graph_.clear();     
            vd.clear();

            // compute only after the first valid occupancy grid is received
            if (voronoi_initialized_ && current_grid_->header.stamp != ros::Time() && current_grid_->info.width != 0 && current_grid_->info.height != 0)
            {

                // get a copy instead of using the original occupancy grid in the planning rate is faster than grid publishing rate
                image_ = cv::Mat(sizeY_,sizeX_, CV_8UC1);
                std::memcpy(image_.data, &current_grid_->data[0], sizeY_*sizeX_*sizeof(int8_t));

                // opencv pre processing

                // if we add an obstacle border later, a white border must be added first to prevent intersection of segments
                cv::Rect border(cv::Point(0, 0), image_.size());
                cv::rectangle(image_, border, cv::Scalar(0,0,0), 10);

                cv::threshold( image_, image_, 50, 100, cv::THRESH_BINARY ); // threshold non-lethal cost to zero
                // remove noise
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4,4));
                cv::morphologyEx(image_, image_, cv::MORPH_OPEN, kernel);
                cv::morphologyEx(image_, image_, cv::MORPH_CLOSE, kernel);

                std::vector<std::vector<cv::Point> > contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours( image_, contours, hierarchy, cv::RETR_EXTERNAL  , cv::CHAIN_APPROX_SIMPLE );

                std::vector<std::vector<cv::Point> > appr_contours;
                for( size_t i = 0; i< contours.size(); i++ )
                {
                    double epsilon = 0.01*cv::arcLength(contours[i],true);
                    std::vector<cv::Point> approximate;
                    cv::approxPolyDP(contours[i],approximate,epsilon,true);
                    appr_contours.push_back(approximate);
                }


                // mutex due to external publishing
                boost::mutex::scoped_lock l(*og_mutex_);

                output.header = current_grid_->header;
                output.info.height = sizeY_;
                output.info.width = sizeX_;
                output.info.resolution = resolution_;
                output.info.origin = current_grid_->info.origin;
                output.info.origin.position.z = -0.01;
                output.data.resize(sizeY_ * sizeX_);

                cv::Mat output_image(int(sizeY_), int(sizeX_), CV_8UC1, &output.data[0]);
                output_image.setTo(cv::Scalar(0,0,0));
                
                for( size_t i = 0; i< appr_contours.size(); i++ )
                {
                    cv::drawContours( output_image, appr_contours, (int)i, cv::Scalar(30,0,0), cv::FILLED, cv::LINE_4, hierarchy, 0 );
                }
                
                // opencv post processing for discretization error of grid comparison
                //kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
                //cv::erode(output_image, output_image, kernel);


                std::vector<Point> points;
                std::vector<Segment> segments;

                for( size_t i = 0; i< appr_contours.size(); i++ )
                {
                    for (auto it_pt = appr_contours[i].begin(); it_pt != std::prev(appr_contours[i].end()); it_pt++)
                    {
                        segments.push_back(Segment(it_pt->x, it_pt->y, std::next(it_pt)->x, std::next(it_pt)->y));
                    }
                    segments.push_back(Segment(std::prev(appr_contours[i].end())->x, std::prev(appr_contours[i].end())->y, appr_contours[i].begin()->x, appr_contours[i].begin()->y));
                }

                
                // Add a boundary to the costmap to avoid infinite edges
                segments.push_back(Segment(0, 0, sizeX_-1, 0));
                segments.push_back(Segment(sizeX_-1, 0, sizeX_-1, sizeY_-1));
                segments.push_back(Segment(sizeX_-1, sizeY_-1, 0, sizeY_-1));
                segments.push_back(Segment(0, sizeY_-1, 0, 0));
                
                
                boost::polygon::construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);
                
                std::unordered_map<const Eigen::Vector2d, teb_local_planner::HcGraphVertexType, MyHash> vertices_map;
                std::vector<boost::polygon::voronoi_diagram<double>::const_vertex_iterator> voronoi_vertices;

                int count = 0;
                for (boost::polygon::voronoi_diagram<double>::const_vertex_iterator it = vd.vertices().begin(); it != vd.vertices().end(); ++it)
                {
                    // degenerate vertice are handled by hashtable
                    if ( it->x()>0 && it->x()<sizeX_-1 && it->y()>0 && it->y()<sizeY_-1)
                    {
                        // because teb_local_planner HcGraph use vecS for vertices
                        // vertices without edge cannot be deleted later and must be filter in advance

                        if (output_image.at<int8_t>(lround(it->y()), lround(it->x())) < 10 )
                        {
                            teb_local_planner::HcGraphVertexType v = boost::add_vertex(graph_);
                            graph_[v].pos.x() = it->x() * resolution_ + origin_.x();
                            graph_[v].pos.y() = it->y() * resolution_ + origin_.y();
                            vertices_map.emplace(Eigen::Vector2d(it->x(), it->y()), v);
                            voronoi_vertices.push_back(it);
                            count++;
                        }
                        
                    }
                }

                for (auto it = voronoi_vertices.begin(); it != voronoi_vertices.end(); it++)
                {
                    const boost::polygon::voronoi_diagram<double>::edge_type *edge = (*it)->incident_edge();
                    do 
                    {
                        if (edge->is_primary())
                        {
                            if (edge->is_finite()){
                                
                                // confirm vertices exist 
                                auto v1 = vertices_map.find(Eigen::Vector2d(edge->vertex0()->x(), edge->vertex0()->y()));
                                auto v2 = vertices_map.find(Eigen::Vector2d(edge->vertex1()->x(), edge->vertex1()->y()));

                                if (v1 != vertices_map.end() && v2 != vertices_map.end())
                                {
                                    if ( output_image.at<int8_t>(lround(edge->vertex0()->y()), lround(edge->vertex0()->x())) < 10 && output_image.at<int8_t>(lround(edge->vertex1()->y()), lround(edge->vertex1()->x())) < 10)
                                    boost::add_edge( v1->second, v2->second, graph_);
                                }

                            }
                        }
                        edge = edge->rot_next();
                    } while (edge != (*it)->incident_edge());
                }

                // traverse the boost graph again to ensure correctness
                
                teb_local_planner::HcGraphEdgeIterator it_i, end_i;
                for (boost::tie(it_i,end_i) = boost::edges(graph_); it_i!=end_i; ++it_i)
                {
                int x1, y1, x2, y2;

                x1 =  (graph_[source(*it_i, graph_)].pos.x() - origin_.x()) / resolution_;
                y1 =  (graph_[source(*it_i, graph_)].pos.y() - origin_.y()) / resolution_;
                x2 =  (graph_[target(*it_i, graph_)].pos.x() - origin_.x()) / resolution_;
                y2 =  (graph_[target(*it_i, graph_)].pos.y() - origin_.y()) / resolution_;

                cv::line(output_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(100,0,0), 1, cv::LINE_8);
                }
                
                

                //voronoi_pub_.publish(output);

                //image_.release();
                //output_image.release();


            }


        }
        catch(const std::exception& ex)
        {
        ROS_ERROR_STREAM("Un-handled Error: " << ex.what() << ".");
        }



    }


    boost::optional<std::pair<teb_local_planner::HcGraphVertexType, teb_local_planner::HcGraphVertexType>> BoostVoronoi::addStartAndGoal(const Eigen::Vector2d& start, const Eigen::Vector2d& goal)
    {
        // as the number and the dimension of points  are both small, navie search seems not a bad idea
        boost::mutex::scoped_lock l(*graph_mutex_);
        
        if (boost::num_vertices(graph_) > 0)
        {
            teb_local_planner::HcGraphVertexIterator it_i, end_i;
            boost::tie(it_i,end_i) = boost::vertices(graph_);

            double min_start_dist = INFINITY;
            teb_local_planner::HcGraphVertexIterator min_start_vertex = end_i;
            double min_goal_dist = INFINITY;
            teb_local_planner::HcGraphVertexIterator min_goal_vertex = end_i;

            while (it_i!=end_i)
            {   
                double start_test_dist = (graph_[*it_i].pos - start).norm();
                double goal_test_dist = (graph_[*it_i].pos - goal).norm();
                if (  start_test_dist <= min_start_dist  )
                {
                    min_start_dist = start_test_dist;
                    min_start_vertex = it_i;
                }
                if (  goal_test_dist <= min_goal_dist  )
                {
                    min_goal_dist = goal_test_dist;
                    min_goal_vertex = it_i;
                }

                ++it_i;
            }

            teb_local_planner::HcGraphVertexType v_s =  boost::add_vertex(graph_);
            graph_[v_s].pos = start;

            teb_local_planner::HcGraphVertexType v_g =  boost::add_vertex(graph_);
            graph_[v_g].pos = goal;

            boost::add_edge(v_s, *min_start_vertex, graph_);
            boost::add_edge(*min_goal_vertex, v_g, graph_);

            return std::make_pair(v_s, v_g);
        }

        return boost::none;
    }

    boost::shared_ptr<teb_local_planner::HcGraph> BoostVoronoi::getGraph()
    {
        return boost::make_shared<teb_local_planner::HcGraph> (graph_) ;
    }

    const nav_msgs::OccupancyGrid& BoostVoronoi::getVisualisation()
    {
        return output;
    }

    void BoostVoronoi::addCircularObstacle(double x, double y, double r)
    {
        cv::Mat image(sizeY_, sizeX_, CV_8UC1, &current_grid_->data[0]);

        cv::circle(image, cv::Point((x-origin_.x())/resolution_, (y-origin_.y())/resolution_), lround(r/resolution_), cv::Scalar(100,0,0), cv::FILLED, cv::LINE_8 );

    }

    void BoostVoronoi::addLineObstacle(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, double t)
    {
        double x1 = pt1.x();
        double y1 = pt1.y();
        double x2 = pt2.x();
        double y2 = pt2.y();

        cv::Mat image(int(current_grid_->info.height), int(current_grid_->info.width), CV_8UC1, &current_grid_->data[0]);

        cv::line(   image, cv::Point((x1-origin_.x())/resolution_, (y1-origin_.y())/resolution_), 
                    cv::Point((x2-origin_.x())/resolution_, (y2-origin_.y())/resolution_), cv::Scalar(100,0,0), lround(t/resolution_), cv::LINE_8 );

    }

    void BoostVoronoi::addPillObstacle(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, double r)
    {
        double x1 = pt1.x();
        double y1 = pt1.y();
        double x2 = pt2.x();
        double y2 = pt2.y();

        cv::Mat image(int(current_grid_->info.height), int(current_grid_->info.width), CV_8UC1, &current_grid_->data[0]);

        Eigen::Vector2d dir(x2-x1,y2-y1);
        
        Eigen::Vector2d normal = Eigen::Vector2d(-dir.y(), dir.x()).normalized();

        std::vector<cv::Point> points{  cv::Point( (x1+r*normal.x()-origin_.x())/resolution_, (y1+r*normal.y()-origin_.y())/resolution_), cv::Point((x2+r*normal.x()-origin_.x())/resolution_, (y2+r*normal.y()-origin_.y())/resolution_), 
                                        cv::Point( (x2-r*normal.x()-origin_.x())/resolution_, (y2-r*normal.y()-origin_.y())/resolution_), cv::Point((x1-r*normal.x()-origin_.x())/resolution_, (y1-r*normal.y()-origin_.y())/resolution_)};

        cv::fillConvexPoly(image, points, cv::Scalar(100,0,0), cv::LINE_8 );

        // or should we draw 2 half ellipses instead of full circles?
        cv::circle(image, cv::Point((x1-origin_.x())/resolution_, (y1-origin_.y())/resolution_), r/resolution_, cv::Scalar(100,0,0), cv::FILLED, cv::LINE_8 );
        cv::circle(image, cv::Point((x2-origin_.x())/resolution_, (y2-origin_.y())/resolution_), r/resolution_, cv::Scalar(100,0,0), cv::FILLED, cv::LINE_8 );

    }

    void BoostVoronoi::addPolygonObstacle(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>>& points)
    {
        cv::Mat image(int(current_grid_->info.height), int(current_grid_->info.width), CV_8UC1, &current_grid_->data[0]);

        std::vector<cv::Point> cv_points;
        for (auto it = points.begin(); it != points.end(); it++)
        {
            cv_points.push_back(cv::Point( (it->x()-origin_.x())/resolution_, (it->y()-origin_.y())/resolution_));
        } 
        cv::fillConvexPoly(image, cv_points, cv::Scalar(100,0,0), cv::FILLED, cv::LINE_8 );
        
    }



} // namespace dynamicvoronoi