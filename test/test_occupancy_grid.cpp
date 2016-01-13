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
#include <geometry/orientation_2d.h>
#include <math/random_generator.h>
#include <occupancy/occupancy_grid_2d.h>
#include <sensing/sensor_2d_radial.h>

#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <iostream>
#include <Eigen/Dense>

DEFINE_bool(visualize_occupancy, true, "Visualize occupancy grids?");

namespace path {

  // Test that we can construct and destroy a 2D occupancy grid.
  TEST(OccupancyGrid, TestOccupancyGrid2D) {
    math::RandomGenerator rng(0);

    // Create an empty occupancy grid.
    OccupancyGrid2D grid(0.0, 1.0, 0.0, 1.0, 0.05);

    // Create a bunch of points and add to the grid.
    for (size_t ii = 0; ii < 1000; ii++) {
      double x = rng.Double();
      double y = rng.Double();
      Point::Ptr point = Point2D::Create(x, y);
      grid.Insert(point);
    }
  }

  // Test that we can construct and destroy a 2D occupancy grid.
  TEST(OccupancyGrid, TestSensor2DRadial) {
    math::RandomGenerator rng(0);

    // Create an empty occupancy grid.
    OccupancyGrid2D grid(0.0, 1.0, 0.0, 1.0, 0.002);

    // Create a bunch of points in a circle and add to the grid.
    for (double theta = 0.0; theta < 2.0 * M_PI; theta += 0.1) {
      Point::Ptr point = Point2D::Create(0.5 + 0.2 * std::cos(theta),
                                         0.5 + 0.2 * std::sin(theta));
      grid.Insert(point);
    }

    grid.Visualize("Dummy.");

    // Create a new sensor.
    Sensor2DRadial sensor(grid, 0.3);

    // Ensure sensor can see all points from the center.
    EXPECT_EQ(sensor.GetObstacleCount(Orientation2D::Create(0.5, 0.5, 0.0)),
              grid.GetTotalCount());
  }

} //\ namespace path
