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
// This class defines a 2D orientation, which is a child class of Point.
//
///////////////////////////////////////////////////////////////////////////////

#include "orientation_2d.h"
#include "point_2d.h"

using Eigen::VectorXd;

namespace path {

  // Factory method.
  Point::Ptr Orientation2D::Create(double x, double y, double theta) {
    Point::Ptr orientation(new Orientation2D(x, y, theta));
    return orientation;
  }

  // Getters.
  Point::Ptr Orientation2D::GetPoint() const {
    Point::Ptr point = Point2D::Create(coordinates_(0), coordinates_(1));
    return point;
  }

  double Orientation2D::GetTheta() const {
    return coordinates_(2);
  }

  // Compute the distance to a 2D point.
  double Orientation2D::DistanceTo(Point::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Type check.
    if (!point->IsType(Point::PointType::POINT_2D)) {
      VLOG(1) << "Point types do not match. Returning a distance of -1.0.";
      return -1.0;
    }

    VectorXd position = point->GetVector();
    double dx = coordinates_(0) - position(0);
    double dy = coordinates_(1) - position(1);

    return std::sqrt(dx*dx + dy*dy);
  }

  // Compute the relative angle to a 2D point.
  double Orientation2D::AngleTo(Point::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Type check.
    if (!point->IsType(Point::PointType::POINT_2D)) {
      VLOG(1) << "Point types do not match. Returning an angle of 0.0.";
      return 0.0;
    }

    VectorXd position = point->GetVector();
    double dx = coordinates_(0) - position(0);
    double dy = coordinates_(1) - position(1);

    return coordinates_(2) - std::atan2(dy, dx);
  }

  // Step toward the given orientation. Returns a null pointer.
  Point::Ptr Orientation2D::StepToward(Point::Ptr point,
                                       double step_size) const {
    return Point::Ptr(nullptr);
  }

  // Default constructor.
  Orientation2D::Orientation2D(double x, double y, double theta)
    : Point((VectorXd(3) << x, y, theta).finished(),
            Point::PointType::ORIENTATION_2D) {}

} //\ namespace path
