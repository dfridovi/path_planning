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
 *          Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <flann/flann_point_2dtree.h>
#include <geometry/point_2d.h>
#include <math/random_generator.h>
#include <util/types.h>

#include <limits>
#include <memory>
#include <gflags/gflags.h>
#include <gtest/gtest.h>

namespace path {

  TEST(FlannPoint2DTree, TestFlannPoint2DTree) {
    math::RandomGenerator rng(0);

    // Make a FLANN kd-tree.
    FlannPoint2DTree point_2dtree;

    // Make a bunch of points and incrementally insert them into the kd tree.
    std::vector<Point2D::Ptr> points;
    for (int ii = 0; ii < 100; ++ii) {
      float x = static_cast<float>(rng.Double());
      float y = static_cast<float>(rng.Double());
      Point2D::Ptr point = Point2D::Create(x, y);

      points.push_back(point);
      point_2dtree.AddPoint(point);
      EXPECT_EQ(point_2dtree.Size(), points.size());
    }

    // Add a batch of points at once.
    std::vector<Point2D::Ptr> points2;
    for (int ii = 0; ii < 100; ++ii) {
      float x = static_cast<float>(rng.Double());
      float y = static_cast<float>(rng.Double());
      Point2D::Ptr point = Point2D::Create(x, y);
      points2.push_back(point);
    }
    point_2dtree.AddPoints(points2);
    EXPECT_EQ(point_2dtree.Size(), points.size() + points2.size());

    // Query the kd tree for nearest neighbor.
    Point2D::Ptr query = Point2D::Create(static_cast<float>(rng.Double()),
                                         static_cast<float>(rng.Double()));
    Point2D::Ptr nearest;
    float nn_distance = -1.0;
    EXPECT_TRUE(point_2dtree.NearestNeighbor(query, nearest, nn_distance));
    EXPECT_NE(nearest, nullptr);

    // Manually compute distance between all points and the query.
    points.insert(points.end(), points2.begin(), points2.end());

    float min_distance = std::numeric_limits<float>::max();
    size_t min_distance_index = 0;
    for (size_t ii = 0; ii < points.size(); ++ii) {
      float distance = Point2D::DistancePointToPoint(points[ii],
                                                     query);
      if (distance < min_distance) {
        min_distance = distance;
        min_distance_index = ii;
      }
    }

    EXPECT_NEAR(Point2D::DistancePointToPoint(points[min_distance_index],
                                              nearest),
                0, 1e-8);
    EXPECT_NEAR(min_distance, nn_distance, 1e-8);
  }

}  //\namespace path
