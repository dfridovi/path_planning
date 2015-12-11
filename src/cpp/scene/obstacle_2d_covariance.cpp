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
// This class models 2D point obstacles with some ellipse of uncertainty.
//
///////////////////////////////////////////////////////////////////////////////

#include "obstacle_2d_covariance.h"
#include <geometry/point_2d.h>
#include <memory>
#include <cmath>

namespace path {

  // Factory method.
  static Obstacle::Ptr Obstacle2DCovariance::Create(double x, double y,
                              double sigma_x, double sigma_y) {
    Obstacle::Ptr obstacle(new Obstacle2DCovariance(x, y,
                                                    sigma_x, sigma_y));
    return obstacle;
  }

  // Is this point feasible?
  bool Obstacle2DCovariance::IsFeasible(Point::Ptr point) const {
    CHECK_NOTNULL(point.get());
    Point2D *point2d = std::static_pointer_cast<Point2D>(point).get();

    if (point->GetX() == x_ && point->GetY() == y_)
      return false;
    return true;
  }

  // What is the cost of occupying this point?
  double Obstacle2DCovariance::Cost(Point::Ptr point) const {
    
  }

  // Default constructor.
  Obstacle2DCovariance::Obstacle2DCovariance(double x, double y,
                                             double sigma_x, double sigma_y)
    : x_(x), y_(y),
      sigma_x_(sigma_x), sigma_y_(sigma_y) {}

} //\ namespace path

#endif
