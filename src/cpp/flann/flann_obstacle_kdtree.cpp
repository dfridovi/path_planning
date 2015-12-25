/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This class is a wrapper around the FlannPointKDTree class. The idea is to
// facilitate quick nearest neighbor searching for the nearest obstacle in a
// scene.
//
///////////////////////////////////////////////////////////////////////////////

#include "flann_obstacle_kdtree.h"
#include <glog/logging.h>

namespace path {

  // Add obstacles to the index.
  void FlannObstacleKDTree::AddObstacle(Obstacle::Ptr obstacle) {
    CHECK_NOTNULL(obstacle.get());

    Point::Ptr location = obstacle->GetLocation();
    kd_tree_.AddPoint(location);
    registry_.emplace(location, obstacle);
  }

  void FlannObstacleKDTree::AddObstacles(std::vector<Obstacle::Ptr>& obstacles) {
    for (const auto& obstacle : obstacles)
      AddObstacle(obstacle);
  }

  // Queries the kd tree for the nearest neighbor of 'query'. Returns whether or
  // not a nearest neighbor was found, and if it was found, the nearest neighbor
  // and distance to the nearest neighbor. Note that index is based on the 
  // order in which obstacles were added with AddObstacle() and AddObstacles().
  bool FlannObstacleKDTree::NearestNeighbor(Obstacle::Ptr query,
                                            Obstacle::Ptr& nearest,
                                            double& nn_distance) {
    CHECK_NOTNULL(query.get());

    // Query kd_tree_.
    Point::Ptr nearest_point;
    Point::Ptr query_point = query->GetLocation();
    kd_tree_.NearestNeighbor(query_point, nearest_point, nn_distance);

    // Map from point back to obstacle.
    const auto match = registry_.find(nearest_point);
    nearest = match->second;
  }

}  //\namespace path
