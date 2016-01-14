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

#include <geometry/trajectory.h>
#include <geometry/point_2d.h>
#include <planning/rrt_planner.h>
#include <robot/robot_2d_circular.h>
#include <math/random_generator.h>
#include <scene/scene_2d_continuous.h>
#include <scene/obstacle_2d.h>
#include <image/image.h>

#include <vector>
#include <cmath>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <iostream>
#include <Eigen/Dense>

DEFINE_bool(visualize_planner, false, "Visualize path?");

namespace path {

  // Test that we can construct and destroy a scene.
  TEST(Planner, TestRRTPlanner) {
    math::RandomGenerator rng(math::RandomGenerator::Seed());

    // Create a bunch of obstacles.
    std::vector<Obstacle::Ptr> obstacles;
    for (size_t ii = 0; ii < 200; ii++) {
      double x = rng.Double();
      double y = rng.Double();
      double radius = rng.DoubleUniform(0.01, 0.02);

      Obstacle::Ptr obstacle =
        Obstacle2D::Create(x, y, radius);
      obstacles.push_back(obstacle);
    }

    // Create a 2D continous scene.
    Scene2DContinuous scene(0.0, 1.0, 0.0, 1.0, obstacles);

    // Create a robot.
    Robot2DCircular robot(scene, 0.005);

    // Choose origin/goal.
    Point::Ptr origin, goal;
    while (!origin) {
      double x = rng.Double();
      double y = rng.Double();
      Point::Ptr point = Point2D::Create(x, y);

      if (robot.IsFeasible(point))
        origin = point;
    }
    while (!goal) {
      double x = rng.Double();
      double y = rng.Double();
      Point::Ptr point = Point2D::Create(x, y);

      if (robot.IsFeasible(point) && origin->DistanceTo(point) > 0.4)
        goal = point;
    }

    // Visualize the map.
    if (FLAGS_visualize_planner) {
      Trajectory::Ptr direct_route = Trajectory::Create();
      direct_route->AddPoint(origin);
      direct_route->AddPoint(goal);
      scene.Visualize("Direct route", direct_route);
    }

    // Plan a route.
    RRTPlanner planner(robot, scene, origin, goal, 0.05);
    Trajectory::Ptr route = planner.PlanTrajectory();

    // If visualize flag is set, query a grid and show the cost map.
    if (FLAGS_visualize_planner) {
      scene.Visualize("RRT route", route);
    }
  }

} //\ namespace path
