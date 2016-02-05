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
// This class is a wrapper around the FlannPoint2DTree class. The idea is to
// facilitate quick nearest neighbor searching for the nearest obstacle in a
// scene.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_FLANN_OBSTACLE_2DTREE_H
#define PATH_PLANNING_FLANN_OBSTACLE_2DTREE_H

#include <geometry/point2d_helpers.h>
#include <scene/obstacle_2d.h>
#include "flann_point_kdtree.h"
#include "../util/disallow_copy_and_assign.h"

#include <flann/flann.h>
#include <unordered_map>

namespace path {

  class FlannObstacle2DTree {
  public:
    FlannObstacle2DTree() {}
    ~FlannObstacle2DTree() {}

    // Add obstacles to the index.
    void AddObstacle(Obstacle2D& obstacle);
    void AddObstacles(std::vector<Obstacle2D>& obstacles);

    // Queries the kd tree for the nearest neighbor of 'query'. Returns whether or
    // not a nearest neighbor was found, and if it was found, the nearest neighbor
    // and distance to the nearest neighbor.
    bool NearestNeighbor(Point2D& query, Obstacle2D& nearest,
                         double& nn_distance) const;

    // Queries the kd tree for all neighbors of 'query' within the specified radius.
    // Returns whether or not the search exited successfully.
    bool RadiusSearch(Point2D& query, std::vector<Obstacle2D>& neighbors,
                      double radius) const;

  private:
    FlannPoint2DTree kd_tree_;
    std::unordered_map<Point2D&, Obstacle2D&> registry_; // to retrieve obstacles

    DISALLOW_COPY_AND_ASSIGN(FlannObstacle2DTree);
  };  //\class FlannObstacle2DTree

}  //\namespace path

#endif
