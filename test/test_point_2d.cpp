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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

#include <geometry/point_2d.h>
#include <math/random_generator.h>

#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include <glog/logging.h>

namespace path {

  // Test that we can construct and destroy a bunch of 2D points.
  TEST(Point2D, TestPoint2D) {
    math::RandomGenerator rng(0);

    for (size_t ii = 0; ii < 1000; ++ii) {
      double x1 = rng.Double();
      double y1 = rng.Double();
      Point::Ptr point1 = Point2D::Create(x1, y1);
      CHECK_NOTNULL(point1.get());

      double x2 = x1 + 0.1 * rng.DoubleUniform(-1.0, 1.0);
      double y2 = y1 + 0.1 * rng.DoubleUniform(-1.0, 1.0);
      Point::Ptr point2 = Point2D::Create(x2, y2);
      CHECK_NOTNULL(point2.get());

      EXPECT_NEAR(point1->DistanceTo(point2), 0.0, 0.1 * std::sqrt(2.0));
    }
  }

} //\ namespace path
