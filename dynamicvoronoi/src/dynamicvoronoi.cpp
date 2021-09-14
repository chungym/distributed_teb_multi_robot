// for comparison only
// source: http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
// BSD licence


#include "dynamicvoronoi/dynamicvoronoi.h"

#include <math.h>
#include <iostream>

#include <boost/smart_ptr/make_shared.hpp>

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
  vertices = NULL;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }
  if (vertices) {
    for (int x=0; x<sizeX; x++) delete[] vertices[x];
    delete[] vertices;
  }
  graph.clear();
  xy_list.clear();
}

void DynamicVoronoi::setMapInfo(double resolution, double x, double y)
{
  resolution_ = resolution;
  origin_x = x;
  origin_y = y;
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];

  if (initGridMap) {
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];

    if (vertices)
    {
      for (int x=0; x<sizeX; x++) delete[] vertices[x];
      delete[] vertices;
    }
    vertices = new teb_local_planner::HcGraphVertexType*[sizeX];
    for (int x=0; x<sizeX; x++) vertices[x] = new teb_local_planner::HcGraphVertexType[sizeY];

  }
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;

  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(std::vector<INTPOINT> points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }  

  lastObstacles.clear();

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }  
}

void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    if (c.needsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;		
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}

bool DynamicVoronoi::isVoronoi( int x, int y ) {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}


void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, INTPOINT(x,y));
    if (updateRealDist) c.dist  = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { 
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x,y);
          pruneQueue.push(INTPOINT(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(INTPOINT(nx,ny));
        }
      }
    }
  }
}


void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx,ny));
      }
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
  // write pgm files

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (isVoronoi(x,y)) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}


void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}


DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}


void DynamicVoronoi::move(int minX_s_, int minY_s_, int minX_t_, int minY_t_, int sizeX_, int sizeY_)
{

  assert(minX_s_ + sizeX_ <= sizeX);
  assert(minY_s_ + sizeY_ <= sizeY);
  assert(minX_t_ + sizeX_ <= sizeX);
  assert(minY_t_ + sizeY_ <= sizeY);

  // mem alloc
  dataCell** data_temp = new dataCell*[sizeX_];
  for (int x=0; x<sizeX_; x++) data_temp[x] = new dataCell[sizeY_];

  bool** gridMap_temp = new bool*[sizeX];
  for (int x=0; x<sizeX_; x++) gridMap_temp[x] = new bool[sizeY_];
  
  // assignment from source
  for (int x=0; x<sizeX_; x++)
    for (int y=0; y<sizeY_; y++) data_temp[x][y] = data[minX_s_+x][minY_s_+y];

  for (int x=0; x<sizeX_; x++) 
    for (int y=0; y<sizeY_; y++) gridMap_temp[x][y] = gridMap[minX_s_+x][minY_s_+y];
  
  // assignment to target
  for (int x=0; x<sizeX_; x++)
    for (int y=0; y<sizeY_; y++) data[minX_t_+x][minY_t_+y] = data_temp[x][y];

  for (int x=0; x<sizeX_; x++) 
    for (int y=0; y<sizeY_; y++) gridMap[minX_t_+x][minY_t_+y] = gridMap_temp[x][y];

}

void DynamicVoronoi::createGraph() 
{
  
  graph.clear();
  xy_list.clear();

  for(int y = sizeY-1; y >=0; y--)
  {      
      for(int x = 0; x<sizeX; x++)
      {	
          if (isVoronoi(x,y)) 
          {
            teb_local_planner::HcGraphVertexType v = boost::add_vertex(graph);
            graph[v].pos.x() = x * resolution_ + origin_x;
            graph[v].pos.y() = y * resolution_ + origin_y;
            vertices[x][y] = v;

            xy_list.push_back( std::pair<int, int>(x, y) );

          } 
      }
  }



  teb_local_planner::HcGraphVertexIterator it_i, end_i;
  for (boost::tie(it_i,end_i) = boost::vertices(graph); it_i!=(end_i); ++it_i)
  {
    int x = xy_list[*it_i].first;
    int y = xy_list[*it_i].second;

    int x_left = xy_list[*it_i].first-1;
    int x_right = xy_list[*it_i].first+1;
    int y_up = xy_list[*it_i].second+1;
    int y_down = xy_list[*it_i].second-1;
  
    bool isVoronoi_left = false;
    bool isVoronoi_right = false;
    bool isVoronoi_up = false;
    bool isVoronoi_down = false;
    bool isVoronoi_left_up = false;
    bool isVoronoi_left_down = false;
    bool isVoronoi_right_up = false;
    bool isVoronoi_right_down = false;

    if (x_left>=0)
    {
      if (isVoronoi(x_left,y))
      {
        isVoronoi_left = true;
      }
    }
    if (x_right<sizeX)
    {
      if (isVoronoi(x_right,y))
      {
        isVoronoi_right = true;
      }
    }
    if (y_up<sizeY)
    {
      if (isVoronoi(x,y_up))
      {
        isVoronoi_up = true;
      }
    }

    if (y_down>=0)
    {
      if (isVoronoi(x,y_down))
      {
        isVoronoi_down = true;
      }
    }

    if (isVoronoi_left && isVoronoi_up)
    {
      if (isVoronoi(x_left,y_up))
      {
        isVoronoi_left_up = true;
      }
    }

    if (isVoronoi_left && isVoronoi_down)
    {
      if (isVoronoi(x_left,y_down))
      {
        isVoronoi_left_down = true;
      }
    }

    if (isVoronoi_right && isVoronoi_up)
    {
      if (isVoronoi(x_right,y_up))
      {
        isVoronoi_right_up = true;
      }
    }

    if (isVoronoi_right && isVoronoi_down)
    {
      if (isVoronoi(x_right,y_down))
      {
        isVoronoi_right_down = true;
      }
    }

    if (isVoronoi_left && !isVoronoi_left_down)
    {
      boost::add_edge(*it_i, vertices[x_left][y], graph);
    }

    if (isVoronoi_right && !isVoronoi_right_down)
    {
      boost::add_edge(*it_i, vertices[x_right][y], graph);
    }

    if (isVoronoi_up && !isVoronoi_left_up)
    {
      boost::add_edge(*it_i, vertices[x][y_up], graph);
    }
    
    if (isVoronoi_down && !isVoronoi_left_down)
    {
      boost::add_edge(*it_i, vertices[x][y_down], graph);
    }
    
    if (isVoronoi_left_down)
    {
      boost::add_edge(*it_i, vertices[x_left][y_down], graph);
    }
    if (isVoronoi_right_up)
    {
      boost::add_edge(*it_i, vertices[x_right][y_up], graph);
    }
    
  }

}


boost::optional<std::pair<int,int>> DynamicVoronoi::spiralIsVoronoi( int q_x, int q_y){

    int X = (sizeX-1 - q_x)*2+1;
    int Y = (sizeY-1 - q_y)*2+1;

    int x,y,dx,dy;
    x = y = dx =0;
    dy = -1;
    int t = std::max(X,Y);
    int maxI = t*t;
    for(int i =0; i < maxI; i++){

        // check if is in range
        if ( q_x+x >=0 && q_x+x < sizeX && q_y+y >= 0 && q_y+y < sizeY ){
          if (isVoronoi(q_x+x, q_y+y))
          {
            return std::pair<int,int> (q_x+x, q_y+y);
          }
        }

        if( (x == y) || ((x < 0) && (x == -y)) || ((x > 0) && (x == 1-y))){
            t = dx;
            dx = -dy;
            dy = t;
        }
        x += dx;
        y += dy;
    }
  return boost::none;
}


teb_local_planner::HcGraph DynamicVoronoi::getGraph(int start_x, int start_y, int goal_x, int goal_y)
{

  teb_local_planner::HcGraphVertexType v_start = boost::add_vertex(graph);
  graph[v_start].pos.x() = start_x*resolution_+origin_x;
  graph[v_start].pos.y() = start_y*resolution_+origin_y;
  xy_list.push_back(std::pair<int, int>(start_x, start_y));

  boost::optional<std::pair<int,int>> start_near_xy = spiralIsVoronoi(start_x, start_y);

  if (start_near_xy)
  {
    boost::add_edge(v_start, vertices[(*start_near_xy).first][(*start_near_xy).second], graph);
  }
 

  teb_local_planner::HcGraphVertexType v_goal = boost::add_vertex(graph);
  graph[v_goal].pos.x() = goal_x*resolution_+origin_x;
  graph[v_goal].pos.y() = goal_y*resolution_+origin_y;
  xy_list.push_back(std::pair<int, int>(goal_x, goal_y));

  boost::optional<std::pair<int,int>> goal_near_xy = spiralIsVoronoi(goal_x, goal_y);

  if (goal_near_xy)
  {
    boost::add_edge(vertices[(*goal_near_xy).first][(*goal_near_xy).second], v_goal, graph);
  }

  paths.clear();
  paths_world.clear();

  int depth = getSizeX() + getSizeY();

  teb_local_planner::HcGraph temp_graph;
  if (start_near_xy && goal_near_xy)
  {
    boost::copy_graph(graph, temp_graph);
  }

  if (start_near_xy){
    boost::remove_edge(v_start, vertices[(*start_near_xy).first][(*start_near_xy).second], graph);
    remove_vertex(v_start, graph);
    xy_list.pop_back();
  }
  if (goal_near_xy) 
  {
    boost::remove_edge(vertices[(*goal_near_xy).first][(*goal_near_xy).second], v_goal, graph);
    remove_vertex(v_goal, graph);
    xy_list.pop_back();
  }
  
  return temp_graph;
  
}



void DynamicVoronoi::search(int start_x, int start_y, int goal_x, int goal_y)
{

  teb_local_planner::HcGraphVertexType v_start = boost::add_vertex(graph);
  graph[v_start].pos.x() = start_x*resolution_+origin_x;
  graph[v_start].pos.y() = start_y*resolution_+origin_y;
  xy_list.push_back(std::pair<int, int>(start_x, start_y));

  boost::optional<std::pair<int,int>> start_near_xy = spiralIsVoronoi(start_x, start_y);

  if (start_near_xy)
  {
    boost::add_edge(v_start, vertices[(*start_near_xy).first][(*start_near_xy).second], graph);
  }
 

  teb_local_planner::HcGraphVertexType v_goal = boost::add_vertex(graph);
  graph[v_goal].pos.x() = goal_x*resolution_+origin_x;
  graph[v_goal].pos.y() = goal_y*resolution_+origin_y;
  xy_list.push_back(std::pair<int, int>(goal_x, goal_y));

  boost::optional<std::pair<int,int>> goal_near_xy = spiralIsVoronoi(goal_x, goal_y);

  if (goal_near_xy)
  {
    boost::add_edge(vertices[(*goal_near_xy).first][(*goal_near_xy).second], v_goal, graph);
  }

  paths.clear();
  paths_world.clear();

  int depth = getSizeX() + getSizeY();

  if (start_near_xy && goal_near_xy)
  {
    std::vector<teb_local_planner::HcGraphVertexType> visited;
    visited.push_back(v_start);

    bool* visited_array = new bool[num_vertices(graph)]();
    visited_array[v_start] = true;

    DepthFirst(graph,visited, visited_array, v_goal, depth);

    delete[] visited_array;
  }

  if (start_near_xy){
    boost::remove_edge(v_start, vertices[(*start_near_xy).first][(*start_near_xy).second], graph);
    remove_vertex(v_start, graph);
    xy_list.pop_back();
  }
  if (goal_near_xy) 
  {
    boost::remove_edge(vertices[(*goal_near_xy).first][(*goal_near_xy).second], v_goal, graph);
    remove_vertex(v_goal, graph);
    xy_list.pop_back();
  }
  
}

void DynamicVoronoi::DepthFirst(teb_local_planner::HcGraph& g, std::vector<teb_local_planner::HcGraphVertexType>& visited, bool*& visited_array, const teb_local_planner::HcGraphVertexType& goal, int depth)
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
      
      std::vector<std::pair<int,int>> path;
      std::vector<std::pair<double, double>> path_world;

      for (auto it_v = visited.begin(); it_v != visited.end(); it_v++)
      {
        path.push_back(std::pair<int,int>(xy_list[*it_v].first, xy_list[*it_v].second));
        path_world.push_back( std::pair<double, double>(graph[*it_v].pos.x(), graph[*it_v].pos.y()) );
      }

      paths.push_back(path);
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
      if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() || *it == goal)
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


std::vector<std::vector<std::pair<int, int> >> DynamicVoronoi::getPaths()
{
  return paths;
}

std::vector<std::vector<std::pair<double, double> >> DynamicVoronoi::getPaths_world()
{
  return paths_world;
}