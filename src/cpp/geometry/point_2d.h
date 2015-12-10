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

#ifndef PATH_PLANNING_POINT_2D_H
#define PATH_PLANNING_POINT_2D_H

#include "point.h"
#include <util/disallow_copy_and_assign.h>

namespace path {

  // Derive from this class when defining a new Point type.
  class Point2D : public Point {
  public:
    // Factory method. Use this one for convenience.
    Point::Ptr Create(double x, double y);

    // Setters.
    void SetX(double x);
    void SetY(double y);

    // Getters.
    double GetX() const;
    double GetY() const;

    // Compute the distance to another 2D point.
    virtual double DistanceTo(Point2D& point);

  private:
    double x_;
    double y_;

    // Default constructor is private.
    Point2D(double x, double y);

    DISALLOW_COPY_AND_ASSIGN(Point2D);
  };

// ---------------------------- Implementation ------------------------------ //

  // Factory method.
  Point::Ptr Create(double x, double y) {
    Point::Ptr point(new Point2D(x, y));
    return point;
  }

  // Default constructor.
  Point2D::Point2D(double x, double y)
    : x_(x), y_(y) {}

  // Setters.
  Point2D::SetX(double x) { x_ = x; }
  Point2D::SetY(double y) { y_ = y; }

  // Getters.
  Point2D::GetX() { return x_; }
  Point2D::GetY() { return y_; }

  // Compute the distance to another 2D point.
  virtual double Point2D::DistanceTo(Point2D::Ptr point) {
    double dx = x_ - point.x_;
    double dy = y_ - point.y_;

    return std::sqrt(dx*dx + dy*dy);
  }

} //\ namespace path

#endif
