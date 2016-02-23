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

#include <robot/robot_2d_circular.h>
#include <scene/obstacle_2d.h>

#include <glog/logging.h>

namespace path {

  // Test if a particular robot location is feasible.
  bool Robot2DCircular::IsFeasible(Point2D::Ptr location) {
    CHECK_NOTNULL(location.get());

    // Find nearest obstacle.
    Obstacle2D::Ptr nearest;
    float nn_distance = -1.0;
    if (!scene_.GetObstacleTree().NearestNeighbor(location, nearest,
                                                  nn_distance))
      return false;

    // Check that it is outside the bounding sphere.
    return nn_distance > radius_ + nearest->GetRadius();;
  }

  // Check if there is a valid linear trajectory between these two points.
  bool Robot2DCircular::LineOfSight(Point2D::Ptr point1,
                                    Point2D::Ptr point2) const {
    CHECK_NOTNULL(point1.get());
    CHECK_NOTNULL(point2.get());

    // Check if line segment intersects any nearby obstacle.
    Point2D::Ptr midpoint = Point2D::MidPoint(point1, point2);
    float max_distance =
      radius_ + scene_.GetLargestObstacleRadius() +
      0.5 * Point2D::DistancePointToPoint(point1, point2);

    FlannObstacle2DTree& obstacle_tree = scene_.GetObstacleTree();
    std::vector<Obstacle2D::Ptr> obstacles;
    if (!obstacle_tree.RadiusSearch(midpoint, obstacles, max_distance)) {
      VLOG(1) << "Radius search failed during LineOfSight() test. "
              << "Returning false.";
      return false;
    }

    for (const auto& obstacle : obstacles) {
      if (Point2D::DistanceLineToPoint(point1, point2,
                                       obstacle->GetLocation()) <
          obstacle->GetRadius() + radius_)
        return false;
    }

    return true;
  }

} // \namespace path
