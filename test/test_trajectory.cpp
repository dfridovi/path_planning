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

#include <geometry/trajectory_2d.h>
#include <geometry/point2d_helpers.h>
#include <util/types.h>
#include <math/random_generator.h>

#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <iostream>

namespace path {

  // Test that we can construct and destroy a Trajectory.
  TEST(Trajectory2D, TestTrajectory) {
    math::RandomGenerator rng(0);

    // Empty Trajectory.
    Trajectory2D::Ptr path1 = Trajectory2D::Create();

    // Vector of Points.
    std::vector<Point2D> points;
    Point2D* last_point;
    double length = 0.0;

    // Make a bunch of points.
    for (size_t ii = 0; ii < 1000; ++ii) {
      float x = rng.Double();
      float y = rng.Double();
      Point2D point = Point2DHelpers::Create(x, y);

      // Keep track of length for comparison.
      if (ii != 0)
        length += Point2DHelpers::DistancePointToPoint(last_point, point);

      // Add to path and vector.
      path1->AddPoint(point);
      points.push_back(point);
      last_point = &point;
    }

    // Make second Trajectory from vector.
    Trajectory2D::Ptr path2 = Trajectory2D::Create(points);

    // Checks lengths.
    EXPECT_NEAR(path1->GetLength(), path2->GetLength(), 1e-16);
    EXPECT_NEAR(path1->GetLength(), length, 1e-16);
  }

} //\ namespace path
