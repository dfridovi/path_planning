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

#ifndef PATH_PLANNING_OBSTACLE_2D_GAUSSIAN_H
#define PATH_PLANNING_OBSTACLE_2D_GAUSSIAN_H

#include "obstacle.h"
#include <Eigen/Dense>

using Eigen::Matrix2d;
using Eigen::Vector2d;

namespace path {

  // Derive from this class when defining a new Obstacle type.
  class Obstacle2DGaussian : public Obstacle {
  public:
    // Factory method.
    static Obstacle::Ptr Create(double x, double y,
                                double sigma_xx, double sigma_yy,
                                double sigma_xy, double threshold = 0.9);

    // Define these methods in a derived class.
    bool IsFeasible(Point::Ptr point) const;
    bool IsFeasible(VectorXd& point) const;
    double Cost(Point::Ptr point) const;
    double Cost(VectorXd& point) const;

  private:
    Vector2d mean_;
    Matrix2d cov_;
    double threshold_;

    // For speed.
    Point::Ptr mean_point_;
    Matrix2d inv_;
    double det_;

    // Default constructor. Threshold is the minimum distance to the obstacle
    //  below which a point is considered infeasible.
    Obstacle2DGaussian(double x, double y,
                       double sigma_xx, double sigma_yy,
                       double sigma_xy, double threshold = 0.05);
  };

} //\ namespace path

#endif
