#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include "bucketedqueue.h"


#include <boost/graph/copy.hpp>

#include <dynamicvoronoi/boost_graph.h>

//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {
  
public:
  DynamicVoronoi();
  ~DynamicVoronoi();

  void setMapInfo(double resolution, double x, double y);

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT> newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist=true);
  //! prune the Voronoi diagram
  void prune();
  //! connect bi-directional directed graph graph
  void createGraph();
  //! connect bi-directional directed graph graph
  void createTEBGraph();
  //! search between start and goal
  void search(int start_x, int start_y, int goal_x, int goal_y);
  //! add start and goal without search
  teb_local_planner::HcGraph getGraph(int start_x, int start_y, int goal_x, int goal_y);
  //! get the results of search
  std::vector<std::vector<std::pair<int, int> >> getPaths();
  std::vector<std::vector<std::pair<double, double> >> getPaths_world();

  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi( int x, int y );
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}

  void move(int minX_s_, int minY_s_, int minX_t_, int minY_t_, int sizeX_, int sizeY);

private:  
  struct dataCell {
    float dist;
    char voronoi;
    char queueing;
    int obstX;
    int obstY;
    bool needsRaise;
    int sqdist;
  };

  typedef enum {voronoiKeep=-4, freeQueued = -3, voronoiRetry=-2, voronoiPrune=-1, free=0, occupied=1} State;
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;


  // methods
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist=true);
  inline void reviveVoroNeighbors(int &x, int &y);

  inline bool isOccupied(int &x, int &y, dataCell &c);
  inline markerMatchResult markerMatch(int x, int y);

  //! depth first visiting
  void DepthFirst(teb_local_planner::HcGraph& g, std::vector<teb_local_planner::HcGraphVertexType>& visited, bool*& visited_array, const teb_local_planner::HcGraphVertexType& goal, int depth);
  //! start from query point, spirally visit cells to find the first cell that is voronoi
  boost::optional<std::pair<int,int>> spiralIsVoronoi( int q_x, int q_y);

  // queues

  BucketPrioQueue open;
  std::queue<INTPOINT> pruneQueue;

  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  // maps
  int sizeY;
  int sizeX;
  dataCell** data;
  bool** gridMap;

  teb_local_planner::HcGraph graph;
  teb_local_planner::HcGraphVertexType** vertices;
  std::vector<std::pair<int, int>> xy_list;

  std::vector<std::vector<std::pair<int, int> >> paths;
  std::vector<std::vector<std::pair<double, double> >> paths_world;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  double resolution_;
  double origin_x;
  double origin_y;

  //  dataCell** getData(){ return data; }
};


#endif

