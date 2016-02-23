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

#include <geometry/orientation_2d.h>
#include <geometry/point_2d.h>

namespace path {

  // Factory method.
  Orientation2D::Ptr Orientation2D::Create(float x, float y, float theta) {
    Orientation2D::Ptr orientation(new Orientation2D(x, y, theta));
    return orientation;
  }

  // Default constructor.
  Orientation2D::Orientation2D(float x, float y, float theta)
    : x_(x), y_(y), theta_(theta) {}

  // Getters.
  Point2D::Ptr Orientation2D::GetPoint2D() const {
    Point2D::Ptr point = Point2D::Create(x_, y_);
    return point;
  }

  float Orientation2D::GetTheta() const {
    return theta_;
  }

  // Compute the distance to a 2D point.
  float Orientation2D::DistanceTo(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    float dx = point->x - x_;
    float dy = point->y - y_;
    return std::sqrt(dx*dx + dy*dy);
  }

  // Compute the relative angle to a 2D point.
  float Orientation2D::AngleTo(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    float dx = point->x - x_;
    float dy = point->y - y_;

    return theta_ - std::atan2(dy, dx);
  }

} //\ namespace path
