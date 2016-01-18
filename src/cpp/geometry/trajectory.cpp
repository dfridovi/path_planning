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
    : length_(0.0), point_type_(Point::PointType::OTHER) {}

  // Initialize with a set of points.
  Trajectory::Trajectory(std::vector<Point::Ptr>& points) {
    length_ = 0.0;

    // Iterate through points, compute distances, and add to path.
    for (size_t ii = 0; ii < points.size(); ii++) {
      Point::Ptr next_point = points[ii];
      CHECK_NOTNULL(next_point.get());

      if (ii == 0) {
        point_type_ = next_point->GetType();
        points_.push_back(next_point);
        continue;
      } else if (next_point->GetType() != point_type_) {
        VLOG(1) << "Point types do not match. Did not insert all points.";
        break;
      }

      points_.push_back(next_point);
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

      if (points_.size() == 0) {
        point_type_ = next_point->GetType();
        points_.push_back(next_point);
        continue;
      } else if (next_point->GetType() != point_type_) {
        VLOG(1) << "Point types do not match. Did not insert all points.";
        break;
      }

      points_.push_back(next_point);
      length_ += last_point->DistanceTo(next_point);
      last_point = next_point;
    }
  }

  // Recompute length.
  void Trajectory::RecomputeLength() {
    length_ = 0.0;

    for (size_t ii = 0; ii < points_.size() - 1; ii++)
      length_ += points_[ii]->DistanceTo(points_[ii + 1]);
  }

  // Add a point to the path.
  void Trajectory::AddPoint(Point::Ptr point) {
    CHECK_NOTNULL(point.get());

    if (points_.size() == 0) {
      points_.push_back(point);
      point_type_ = point->GetType();
      return;
    } else if (point->GetType() != point_type_) {
      VLOG(1) << "Point is of the wrong type. Did not insert.";
      return;
    }

    Point::Ptr last_point = points_.back();
    CHECK_NOTNULL(last_point.get());

    points_.push_back(point);
    length_ += last_point->DistanceTo(point);
  }

  // Upsample by adding k points linearly between each pair of points
  // in this Trajectory.
  void Trajectory::Upsample(unsigned int k) {
    std::vector<Point::Ptr> old_points(points_);
    points_.clear();

    for (size_t ii = 0; ii < old_points.size() - 1; ii++) {
      Point::Ptr current_point = old_points[ii];
      Point::Ptr next_point = old_points[ii + 1];
      double length = current_point->DistanceTo(next_point);
      double step_size = length / static_cast<double>(k + 1);

      points_.push_back(current_point);
      for (unsigned int jj = 1; jj <= k; jj++) {
        Point::Ptr step =
          current_point->StepToward(next_point,
                                    static_cast<double>(jj) * step_size);
        points_.push_back(step);
      }

      // Make sure to push back final point.
      if (ii == old_points.size() - 1)
        points_.push_back(next_point);
    }
  }

  // Compute the first derivative of the path in time using a 1D symmetric
  // difference filter, assuming uniform (time) sampling. Calculate at the
  // endpoints with forward/backward differences.
  Trajectory::Ptr Trajectory::TimeDerivative() {
    Trajectory::Ptr derivative;

    // Handle corner cases: size 0/1.
    if (points_.size() == 0) {
      VLOG(1) << "Caution! Tried to evaluate the derivative "
              << "of an empty Trajectory.";
      return derivative;
    }

    Point::Ptr zero = points_[0]->Add(points_[0], -1.0);
    if (points_.size() == 1) {
      VLOG(1) << "Caution! Tried to evaluate the derivative "
              << "of a Trajectory with only a single element.";
      derivative->AddPoint(zero);
    }

    // Handle the first point.
    Point::Ptr diff = points_[1]->Add(points_[0], -1.0);
    derivative->AddPoint(diff);

    // Handle middle points. Remember to divide by two for symmetric differences.
    for (size_t ii = 1; ii < points_.size() - 1; ii++) {
      diff = points_[ii + 1]->Add(points_[ii - 1], -1.0);
      derivative->AddPoint(zero->Add(diff, 0.5));
    }

    // Handle the last point.
    diff = points_[points_.size() - 1]->Add(points_[points_.size() - 2], -1.0);
    derivative->AddPoint(diff);

    return derivative;
  }

  // Getters.
  Point::PointType Trajectory::GetType() const { return point_type_; }
  double Trajectory::GetLength() const { return length_; }
  std::vector<Point::Ptr>& Trajectory::GetPoints() { return points_; }
  Point::Ptr Trajectory::GetAt(size_t index) {
    // Check index.
    if (index >= points_.size()) {
      VLOG(1) << "Index is out of bounds. Returning a nullptr.";
      return nullptr;
    }

    return points_[index];
  }

  // Setter.
  void Trajectory::SetAt(Point::Ptr point, size_t index) {
    CHECK_NOTNULL(point.get());

    // Check point type.
    if (point->GetType() != point_type_) {
      VLOG(1) << "Point is of the wrong type. Did not replace.";
      return;
    }

    // Check index.
    if (index >= points_.size()) {
      VLOG(1) << "Index is out of bounds. Did not replace.";
      return;
    }

    // Replace and adjust length_ case-by-case.
    if (index == 0 && points_.size() == 1)
      points_[0] = point;
    else if (index == 0 && points_.size() >= 2) {
      length_ -= points_[0]->DistanceTo(points_[1]);
      length_ += point->DistanceTo(points_[1]);
      points_[0] = point;
    } else if (index == points_.size() - 1) {
      length_ -= points_[index]->DistanceTo(points_[index - 1]);
      length_ += point->DistanceTo(points_[index - 1]);
      points_[index] = point;
    } else {
      length_ -= points_[index]->DistanceTo(points_[index - 1]);
      length_ -= points_[index]->DistanceTo(points_[index + 1]);
      length_ += point->DistanceTo(points_[index - 1]);
      length_ += point->DistanceTo(points_[index + 1]);
      points_[index] = point;
    }
  }
}
