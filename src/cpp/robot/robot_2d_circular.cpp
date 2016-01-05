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
 * Author: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This class defines a simple circular robot in two dimensions.
//
///////////////////////////////////////////////////////////////////////////////

#include "robot_2d_circular.h"

namespace path {

  // Test if a particular robot location is feasible.
  bool Robot2DCircular::IsFeasible(Point::Ptr location) {

    // Find nearest obstacle.
    Obstacle::Ptr nearest;
    double nn_distance = -1.0;
    if (!obstacle_tree_.NearestNeighbor(location, nearest, nn_distance))
      return false;

    // Check that it is outside the bounding sphere.
    return nn_distance > radius_ + nearest->GetRadius();;
  }

  // Check if there is a valid linear trajectory between these two points.
  bool Robot2DCircular::LineOfSight(Point::Ptr point1, Point::Ptr point2) const {
    LineSegment line(point1, point2);

    // Check if line segment intersects any obstacle within the appropriate radius.
    Point::Ptr midpoint = line.MidPoint();
    double radius = radius_ + scene_.GetLargestObstacleRadius() + 0.5 * line.GetLength();

    for (const auto& obstacle : scene_.GetObstacles()) {
      if (line.DistanceTo(obstacle->GetLocation()) < obstacle->GetRadius() + radius_)
        return false;
    }

    return true;
  }

} // \namespace path
