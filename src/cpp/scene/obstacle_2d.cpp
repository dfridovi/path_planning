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
// This class models 2D point obstacles.
//
///////////////////////////////////////////////////////////////////////////////

#include "obstacle_2d.h"
#include <geometry/point_2d.h>
#include <glog/logging.h>

namespace path {

  // Factory methods.
  Obstacle::Ptr Obstacle2D::Create(double x, double y, double radius) {
    Obstacle::Ptr obstacle(new Obstacle2D(x, y, radius));
    return obstacle;
  }

  Obstacle::Ptr Obstacle2D::Create(Point::Ptr point, double radius) {
    Obstacle::Ptr obstacle(new Obstacle2D(point, radius));
    return obstacle;
  }

  // Is this point feasible?
  bool Obstacle2D::IsFeasible(Point::Ptr point) const {
    CHECK_NOTNULL(point.get());
    if (point->DistanceTo(location_) < radius_)
      return false;
    return true;
  }

  // Is this point feasible?
  bool Obstacle2D::IsFeasible(VectorXd& point) const {
    if ((point - location_->GetVector()).norm() < radius_)
      return false;
    return true;
  }

  // Cost of occupying this point. Either zero or infinity.
  double Obstacle2D::Cost(Point::Ptr point) const {
    if (!IsFeasible(point))
      return std::numeric_limits<double>::infinity();
    return 0.0;
  }

  // Cost of occupying this point. Either zero or infinity.
  double Obstacle2D::Cost(VectorXd& point) const {
    if (!IsFeasible(point))
      return std::numeric_limits<double>::infinity();
    return 0.0;
  }

  // Default constructors. Radius is the minimum distance to the obstacle
  // below which a point is considered infeasible.
  Obstacle2D::Obstacle2D(double x, double y, double radius)
    : Obstacle(Point2D::Create(x, y), radius) {}

  Obstacle2D::Obstacle2D(Point::Ptr point, double radius)
    : Obstacle(point, radius) {
    // Check point type.
    if (!point->IsType(Point::PointType::POINT_2D))
      VLOG(1) << "Caution! Creating Obstacle2D with a point of the wrong type.";
  }

} //\ namespace path
