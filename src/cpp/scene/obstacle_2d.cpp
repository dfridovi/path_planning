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

#include <scene/obstacle_2d.h>

#include <memory>
#include <cmath>
#include <iostream>
#include <Eigen/SVD>
#include <glog/logging.h>

using Eigen::Vector2f;

namespace path {

  // Factory methods.
  Obstacle2D::Ptr Obstacle2D::Create(float x, float y,
                                     float sigma_xx, float sigma_yy,
                                     float sigma_xy, float radius_zscore) {
    Obstacle2D::Ptr obstacle(new Obstacle2D(x, y, sigma_xx, sigma_yy, sigma_xy,
                                            radius_zscore));
    return obstacle;
  }

  Obstacle2D::Ptr Obstacle2D::Create(float x, float y, float radius) {
    Obstacle2D::Ptr obstacle(new Obstacle2D(x, y, radius));
    return obstacle;
  }

  // Default constructor.
  Obstacle2D::Obstacle2D(float x, float y,
                         float sigma_xx, float sigma_yy,
                         float sigma_xy, float radius_zscore) {
    // Set mean and covariance.
    mean_(0) = x;
    mean_(1) = y;
    location_ = Point2D::Create(x, y);

    cov_(0, 0) = sigma_xx;
    cov_(0, 1) = sigma_xy;
    cov_(1, 0) = sigma_xy;
    cov_(1, 1) = sigma_yy;

    // Precalculate determinant and inverse.
    det_ = cov_.determinant();
    inv_ = cov_.inverse();

    // Determine radius from zscore. Note that the largest eigenvalue of the
    // covariance matrix is the variance along the principle axis.
    Eigen::JacobiSVD<Matrix2f> svd(cov_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    radius_ = radius_zscore * svd.singularValues()(0);
  }

  // Radius-based initialization.
  Obstacle2D::Obstacle2D(float x, float y, float radius) {
    // Set mean and identity covariance.
    mean_(0) = x;
    mean_(1) = y;
    location_ = Point2D::Create(x, y);

    cov_(0, 0) = radius * radius / 3.0;
    cov_(0, 1) = 0.0;
    cov_(1, 0) = 0.0;
    cov_(1, 1) = radius * radius / 3.0;

    // Precalculate determinant and inverse.
    det_ = cov_.determinant();
    inv_ = cov_.inverse();

    // Set radius.
    radius_ = radius;
  }

  // Get location.
  Point2D::Ptr Obstacle2D::GetLocation() {
    return location_;
  }

  // Get radius.
  float Obstacle2D::GetRadius() {
    return radius_;
  }

  // Is this point feasible?
  bool Obstacle2D::IsFeasible(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    if (Point2D::DistancePointToPoint(point, location_) < radius_)
      return false;
    return true;
  }

  // What is the cost of occupying this point?
  float Obstacle2D::Cost(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    Vector2f query(point->x, point->y);
    return std::exp(-0.5 * (query - mean_).transpose() * inv_ * (query - mean_)) /
      std::sqrt((2.0 * M_PI) * (2.0 * M_PI) * det_);
  }

  // Derivative of the cost function by position. This is used for
  // trajectory optimization.
  Point2D::Ptr Obstacle2D::Derivative(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    Vector2f point_vector(point->x, point->y);
    Vector2f vector_derivative = -Cost(point) * inv_ * (point_vector - mean_);
    return Point2D::Create(vector_derivative(0), vector_derivative(1));
  }

} //\ namespace path
