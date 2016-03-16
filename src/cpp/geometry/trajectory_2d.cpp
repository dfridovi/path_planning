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

#include <geometry/trajectory_2d.h>
#include <geometry/point_2d.h>
#include <glog/logging.h>
#include <iostream>


namespace path {

  // Factory methods.
  Trajectory2D::Ptr Trajectory2D::Create() {
    Trajectory2D::Ptr path(new Trajectory2D());
    return path;
  }

  Trajectory2D::Ptr Trajectory2D::Create(std::vector<Point2D::Ptr>& points) {
    Trajectory2D::Ptr path(new Trajectory2D(points));
    return path;
  }

  Trajectory2D::Ptr Trajectory2D::Create(std::list<Point2D::Ptr>& points) {
    Trajectory2D::Ptr path(new Trajectory2D(points));
    return path;
  }

  // A Trajectory2D is just an ordered list of Points.
  Trajectory2D::Trajectory2D()
    : length_(0.0) {}

  // Initialize with a set of points.
  Trajectory2D::Trajectory2D(std::vector<Point2D::Ptr>& points) {
    length_ = 0.0;

    // Iterate through points, compute distances, and add to path.
    for (size_t ii = 0; ii < points.size(); ii++) {
      Point2D::Ptr next_point = points[ii];
      points_.push_back(next_point);

      if (ii > 0) {
        Point2D::Ptr last_point = points[ii - 1];
        length_ += Point2D::DistancePointToPoint(last_point, next_point);
      }
    }
  }

  // Initialize with a set of points.
  Trajectory2D::Trajectory2D(std::list<Point2D::Ptr>& points) {
    length_ = 0.0;

    // Iterate through points, compute distances, and add to path.
    Point2D::Ptr last_point = points.front();
    for (const auto& next_point : points) {
      points_.push_back(next_point);
      length_ += Point2D::DistancePointToPoint(last_point, next_point);
      last_point = next_point;
    }
  }

  // Recompute length.
  void Trajectory2D::RecomputeLength() {
    length_ = 0.0;

    for (size_t ii = 0; ii < points_.size() - 1; ii++)
      length_ += Point2D::DistancePointToPoint(points_[ii],
                                               points_[ii + 1]);
  }

  // Add a point to the path.
  void Trajectory2D::AddPoint(Point2D::Ptr point) {
    CHECK_NOTNULL(point.get());

    if (points_.size() > 0) {
      Point2D::Ptr last_point = points_.back();
      length_ += Point2D::DistancePointToPoint(last_point, point);
    }

    points_.push_back(point);
  }

  // Upsample by adding k points linearly between each pair of points
  // in this Trajectory2D.
  void Trajectory2D::Upsample(unsigned int k) {
    std::vector<Point2D::Ptr> old_points(points_);
    points_.clear();

    for (size_t ii = 0; ii < old_points.size() - 1; ii++) {
      Point2D::Ptr current_point = old_points[ii];
      Point2D::Ptr next_point = old_points[ii + 1];
      double length = Point2D::DistancePointToPoint(current_point,
                                                    next_point);
      double step_size = length / static_cast<double>(k + 1);

      points_.push_back(current_point);
      for (unsigned int jj = 1; jj <= k; jj++) {
        Point2D::Ptr step =
          Point2D::StepToward(current_point, next_point,
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
  Trajectory2D::Ptr Trajectory2D::TimeDerivative() {
    Trajectory2D::Ptr derivative = Trajectory2D::Create();

    // Handle corner cases: size 0/1.
    if (points_.size() == 0) {
      VLOG(1) << "Caution! Tried to evaluate the derivative "
              << "of an empty Trajectory2D.";
      return derivative;
    }

    Point2D::Ptr zero = Point2D::Create(0.0, 0.0);
    if (points_.size() == 1) {
      VLOG(1) << "Caution! Tried to evaluate the derivative "
              << "of a Trajectory2D with only a single element.";
      derivative->AddPoint(zero);
      return derivative;
    }

    // Handle the first point.
    Point2D::Ptr diff = Point2D::Add(points_[1], points_[0], -1.0);
    derivative->AddPoint(diff);

    // Handle middle points. Remember to divide by two for symmetric differences.
    for (size_t ii = 1; ii < points_.size() - 1; ii++) {
      diff = Point2D::Add(points_[ii + 1], points_[ii - 1], -1.0);
      derivative->AddPoint(Point2D::Add(zero, diff, 0.5));
    }

    // Handle the last point.
    diff = Point2D::Add(points_[points_.size() - 1],
                               points_[points_.size() - 2], -1.0);
    derivative->AddPoint(diff);

    return derivative;
  }

  // Getters.
  double Trajectory2D::GetLength() const { return length_; }
  std::vector<Point2D::Ptr>& Trajectory2D::GetPoints() { return points_; }
  Point2D::Ptr Trajectory2D::GetAt(size_t index) {
    // Check index.
    if (index >= points_.size()) {
      LOG(ERROR) << "Index is out of bounds. Returning a nullptr.";
      return nullptr;
    }

    return points_[index];
  }

  // Setter.
  void Trajectory2D::SetAt(Point2D::Ptr point, size_t index) {
    // Check index.
    if (index >= points_.size()) {
      VLOG(1) << "Index is out of bounds. Did not replace.";
      return;
    }

    // Replace and adjust length_ case-by-case.
    if (index == 0 && points_.size() == 1)
      points_[0] = point;
    else if (index == 0 && points_.size() >= 2) {
      length_ -= Point2D::DistancePointToPoint(points_[0],
                                               points_[1]);
      length_ += Point2D::DistancePointToPoint(point,
                                               points_[1]);
      points_[0] = point;
    } else if (index == points_.size() - 1) {
      length_ -= Point2D::DistancePointToPoint(points_[index],
                                               points_[index - 1]);
      length_ += Point2D::DistancePointToPoint(point,
                                               points_[index - 1]);
      points_[index] = point;
    } else {
      length_ -= Point2D::DistancePointToPoint(points_[index],
                                               points_[index - 1]);
      length_ -= Point2D::DistancePointToPoint(points_[index],
                                               points_[index + 1]);
      length_ += Point2D::DistancePointToPoint(point,
                                               points_[index - 1]);
      length_ += Point2D::DistancePointToPoint(point,
                                               points_[index + 1]);
      points_[index] = point;
    }
  }
}
