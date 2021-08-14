#ifndef _BOOST_VORONOI_H_
#define _BOOST_VORONOI_H_


//system 
#include <Eigen/Core>
#include <unordered_map>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/thread.hpp>
#include <boost/polygon/voronoi.hpp>

//ROS
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>

//package
#include <dynamicvoronoi/boost_graph.h>



namespace dynamicvoronoi{

  struct Point {
    int a;
    int b;
    Point(int x, int y) : a(x), b(y) {}
  };

  struct Segment {
    Point p0;
    Point p1;
    Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
  };



  struct MyHash {
    size_t operator()(const Eigen::Vector2d& vertex) const {

      
      std::size_t seed = 0;
      boost::hash_combine(seed, boost::hash_value(vertex.x()));
      boost::hash_combine(seed, boost::hash_value(vertex.y()));
      return seed;
      

      // what if duplicate vertices?
      //return std::hash<const boost::polygon::voronoi_vertex<double>*>()(&vertex);

    }
  };



  class BoostVoronoi
  {
    public:

      BoostVoronoi();
      ~BoostVoronoi();

      boost::mutex* getMutex();
      boost::mutex* getGridMutex();

      void updateGraph();

      void setCostmap2D(costmap_2d::Costmap2D *costmap, std::string global_frame_id);
      void disableCostmap2D();

      void setOccupancyGrid(nav_msgs::OccupancyGrid::Ptr grid);

      void updateGridFromCostmap();

      void addCircularObstacle(double x, double y, double r);
      void addLineObstacle(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, double t);
      void addPillObstacle(const Eigen::Vector2d& pt1, const Eigen::Vector2d& pt2, double r);
      void addPolygonObstacle(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>>& points);

      boost::optional<std::pair<teb_local_planner::HcGraphVertexType, teb_local_planner::HcGraphVertexType>> addStartAndGoal(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

      boost::shared_ptr<teb_local_planner::HcGraph> getGraph();

      const nav_msgs::OccupancyGrid& getVisualisation();

    private:

      bool voronoi_initialized_;

      char* cost_translation_table_;

      teb_local_planner::HcGraph graph_;
      boost::mutex* graph_mutex_;

      costmap_2d::Costmap2D* costmap_; //!< costmap
      std::string global_frame_;
      bool directCostmap_;

      nav_msgs::OccupancyGrid::Ptr current_grid_;

      cv::Mat image_;

      nav_msgs::OccupancyGrid output;
      boost::mutex* og_mutex_;

      int sizeX_;
      int sizeY_;
      double resolution_;
      Eigen::Vector2d origin_;

      boost::polygon::voronoi_diagram<double> vd;

  };


} // namespace dynamicvoronoi


  namespace boost {
    namespace polygon {

      template <>
      struct geometry_concept<dynamicvoronoi::Point> {
        typedef point_concept type;
      };

      template <>
      struct point_traits<dynamicvoronoi::Point> {
        typedef int coordinate_type;

        static inline coordinate_type get(
            const dynamicvoronoi::Point& point, orientation_2d orient) {
          return (orient == HORIZONTAL) ? point.a : point.b;
        }
      };

      template <>
      struct geometry_concept<dynamicvoronoi::Segment> {
        typedef segment_concept type;
      };

      template <>
      struct segment_traits<dynamicvoronoi::Segment> {
        typedef int coordinate_type;
        typedef dynamicvoronoi::Point point_type;

        static inline point_type get(const dynamicvoronoi::Segment& segment, direction_1d dir) {
          return dir.to_int() ? segment.p1 : segment.p0;
        }
      };

    }  // polygon
  }  // boost



#endif
