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
// This class defines the Trajectory datatype. It operates on generic Point objects,
// i.e. a Path is an ordered list of Points, but these Points can be
// in any arbitrary space.
//
///////////////////////////////////////////////////////////////////////////////

#include "trajectory.h"
#include <glog/logging.h>
#include <iostream>


namespace path {

  // Factory methods.
  Trajectory::Ptr Trajectory::Create() {
    Trajectory::Ptr path(new Trajectory());
    return path;
  }

  Trajectory::Ptr Trajectory::Create(std::vector<Point::Ptr>& points) {
    Trajectory::Ptr path(new Trajectory(points));
    return path;
  }

  Trajectory::Ptr Trajectory::Create(std::list<Point::Ptr>& points) {
    Trajectory::Ptr path(new Trajectory(points));
    return path;
  }

  // A Trajectory is just an ordered list of Points.
  Trajectory::Trajectory()
    : length_(0.0) {}

  // Initialize with a set of points.
  Trajectory::Trajectory(std::vector<Point::Ptr>& points) {
    length_ = 0.0;

    // Iterate through points, compute distances, and add to path.
    for (size_t ii = 0; ii < points.size(); ii++) {
      Point::Ptr next_point = points[ii];
      CHECK_NOTNULL(next_point.get());

      points_.push_back(next_point);
      if (ii == 0) continue;

      Point::Ptr last_point = points[ii - 1];
      length_ += last_point->DistanceTo(next_point);
    }
  }

  // Initialize with a set of points.
  Trajectory::Trajectory(std::list<Point::Ptr>& points) {
    length_ = 0.0;

    // Iterate through points, compute distances, and add to path.
    Point::Ptr& last_point = points.front();
    for (const auto& next_point : points) {
      CHECK_NOTNULL(next_point.get());

      points_.push_back(next_point);
      length_ += last_point->DistanceTo(next_point);
      last_point = next_point;
    }
  }

  // Add a point to the path.
  void Trajectory::AddPoint(Point::Ptr point) {
    if (points_.size() == 0) {
      points_.push_back(point);
      return;
    }

    Point::Ptr last_point = points_.back();
    CHECK_NOTNULL(last_point.get());

    points_.push_back(point);
    length_ += last_point->DistanceTo(point);
  }

  // Get path length.
  double Trajectory::GetLength() const { return length_; }
}
