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

#ifndef PATH_PLANNING_OBSTACLE_2D_H
#define PATH_PLANNING_OBSTACLE_2D_H

#include "../util/types.h"
#include "../geometry/point2d_helpers.h"
#include <Eigen/Dense>

using Eigen::Matrix2f;
using Eigen::Vector2f;

namespace path {

  class Obstacle2D {
  public:
    // Default constructors.
    Obstacle2D(float x, float y,
               float sigma_xx, float sigma_yy,
               float sigma_xy, float radius_zscore = 0.05);
    Obstacle2D(float x, float y, float radius);

    // Getter.
    Point2D& GetLocation();

    // Feasibility, cost, and derivative evaluation.
    bool IsFeasible(Point2D& point) const;
    float Cost(Point2D& point) const;
    Point2D& Derivative(Point2D& point) const;

  private:
    Vector2f mean_;
    Matrix2f cov_;
    Point2D location_;

    // For speed.
    Matrix2f inv_;
    float det_;
  };

} //\ namespace path

#endif
