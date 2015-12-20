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
// This class defines a 2D point, which is a child class of Point.
//
///////////////////////////////////////////////////////////////////////////////

#include "point_2d.h"
#include <glog/logging.h>

namespace path {

  // Default constructor.
  Point2D::Point2D(double x, double y) {
    pt_(0) = x;
    pt_(1) = y;

    SetType(Point::PointType::POINT_2D);
  }

  // Factory method.
  Point::Ptr Point2D::Create(double x, double y) {
    Point::Ptr point(new Point2D(x, y));
    return point;
  }

  // Setters.
  void Point2D::SetX(double x) { pt_(0) = x; }
  void Point2D::SetY(double y) { pt_(1) = y; }

  // Getters.
  double Point2D::GetX() const { return pt_(0); }
  double Point2D::GetY() const { return pt_(1); }
  Vector2d& Point2D::GetVector() { return pt_; };

  // Compute the distance to another 2D point.
  double Point2D::DistanceTo(Point::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Type check.
    if (!point->IsType(Point::PointType::POINT_2D)) {
      VLOG(1) << "Point types do not match. Returning a distance of -1.0.";
      return -1.0;
    }

    Point2D *point2d = std::static_pointer_cast<Point2D>(point).get();

    double dx = pt_(0) - point2d->pt_(0);
    double dy = pt_(1) - point2d->pt_(1);

    return std::sqrt(dx*dx + dy*dy);
  }

} //\ namespace path
