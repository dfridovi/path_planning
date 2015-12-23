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
// This is a helper class to model line segments, intended for use with motion
// planners for tasks like intersection-checking.
//
///////////////////////////////////////////////////////////////////////////////

#include "line_segment.h"
#include <glog/logging.h>
#include <math/random_generator.h>

using Eigen::VectorXd;

namespace path {

  // A LineSegment is just a pair of points.
  LineSegment::LineSegment(Point::Ptr point1, Point::Ptr point2) {
    CHECK_NOTNULL(point1.get());
    CHECK_NOTNULL(point2.get());

    point1_ = point1;
    point2_ = point2;

    if (!point1->IsSameTypeAs(point2)) {
      VLOG(1) << "Point types do not match. setting the second point equal "
        "to the first point.";
      point2_ = point1;
    }
  }


  // Segment length.
  double LineSegment::GetLength() const {
    return point1_->DistanceTo(point2);
  }

  // Test intersection with an obstacle. Uses Monte Carlo simulation
  // with 1000 trials by default.
  bool LineSegment::Intersects(Obstacle::Ptr obstacle,
                               unsigned int niter) const {
    CHECK_NOTNULL(obstacle.get());

    for (unsigned int ii = 0; ii < niter; ii++) {
      VectorXd& random_vector = GetRandomVector();
      if (!obstacle->IsFeasible(random_vector))
        return false;
    }

    return true;
  }

  // Pick a random point along the line segment.
  VectorXd& LineSegment::GetRandomVector() const {
    math::RandomGenerator rng(math::RandomGenerator::Seed());

    // Parameterize segment as weighted sum of two endpoints.
    double weight = rng.Double();
    VectorXd& vector1 = point1_->GetVector();
    VectorXd& vector2 = point2_->GetVector();
    VectorXd random_vector(vector1_.size());

    random_vector = weight * vector1 + (1.0 - weight) * vector2;
    return random_vector;
  }


} //\ namespace path
