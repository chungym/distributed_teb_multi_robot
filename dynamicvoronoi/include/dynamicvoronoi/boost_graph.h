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

#ifndef _BOOST_GRAPH_H_
#define _BOOST_GRAPH_H_


#include <Eigen/Core>

#ifdef BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
  #include <boost/graph/adjacency_list.hpp>
#else
  // Workaround for a bug in boost graph library (concerning directed graphs), boost version 1.48:
  // boost::add_vertex requires a move constructor/assignment operator in one of the underlying boost objects if C++11 is activated,
  // but they are missing. The compiler fails due to an implicit deletion. We just deactivate C++11 default functions for now.
  #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
  #include <boost/graph/adjacency_list.hpp>
  #undef BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
#endif

#ifndef GRAPH_SEARCH_INTERFACE_H

namespace teb_local_planner
{
  struct HcGraphVertex
  {
  public:
    Eigen::Vector2d pos; // position of vertices in the map
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  //! Abbrev. for the homotopy class search-graph type @see HcGraphVertex
  typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, HcGraphVertex, boost::no_property > HcGraph;
  //! Abbrev. for vertex type descriptors in the homotopy class search-graph
  typedef boost::graph_traits<teb_local_planner::HcGraph>::vertex_descriptor HcGraphVertexType;
  //! Abbrev. for edge type descriptors in the homotopy class search-graph
  typedef boost::graph_traits<teb_local_planner::HcGraph>::edge_descriptor HcGraphEdgeType;
  //! Abbrev. for the vertices iterator of the homotopy class search-graph
  typedef boost::graph_traits<teb_local_planner::HcGraph>::vertex_iterator HcGraphVertexIterator;
  //! Abbrev. for the edges iterator of the homotopy class search-graph
  typedef boost::graph_traits<teb_local_planner::HcGraph>::edge_iterator HcGraphEdgeIterator;
  //! Abbrev. for the adjacency iterator that iterates vertices that are adjecent to the specified one
  typedef boost::graph_traits<teb_local_planner::HcGraph>::adjacency_iterator HcGraphAdjecencyIterator;

}

#endif

#endif
