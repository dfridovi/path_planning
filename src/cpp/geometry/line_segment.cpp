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
#include <math/random_generator.h>
#include <glog/logging.h>
#include <cmath>

namespace path {

  // A LineSegment is just a pair of points.
  LineSegment::LineSegment(Point::Ptr point1, Point::Ptr point2) {
    CHECK_NOTNULL(point1.get());
    CHECK_NOTNULL(point2.get());

    point1_ = point1;
    point2_ = point2;
    point_type_ = point1_->GetType();

    if (!point1->IsSameTypeAs(point2)) {
      VLOG(1) << "Point types do not match. setting the second point equal "
        "to the first point.";
      point2_ = point1;
    }
  }

  // Segment length.
  double LineSegment::GetLength() const {
    return point1_->DistanceTo(point2_);
  }

  // Distance between a point and this line segment.
  double LineSegment::DistanceTo(Point::Ptr point) const {
    CHECK_NOTNULL(point.get());
    if (point->GetType() != point_type_) {
      VLOG(1) << "Point is of the wrong type. Returning infinity.";
      return std::numeric_limits<double>::infinity();
    }

    // Get vectors from point1_ to point2_ and from point1_ to point.
    VectorXd direction = point2_->GetVector() - point1_->GetVector();
    VectorXd vector = point->GetVector() - point1_->GetVector();
    direction /= direction.norm();

    // Test if point projects onto line segment.
    double dot_product = direction.dot(vector);
    if (dot_product <= GetLength() && dot_product >= 0.0) {
      // Projects onto line segment.
      return std::sqrt(vector.squaredNorm() - dot_product * dot_product);
    } else {
      // Doesn't project onto the line segment.
      double dist1 = point1_->DistanceTo(point);
      double dist2 = point2_->DistanceTo(point);
      return std::min(dist1, dist2);
    }
  }

} //\ namespace path
