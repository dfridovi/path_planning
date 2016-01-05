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
// This file defines the RRT (rapidly-exploring random tree) class, which is
// a derived class based on the Planner class. It is implemented according to
// the following descriptions:
// + https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf
// + http://msl.cs.uiuc.edu/rrt/about.html
//
///////////////////////////////////////////////////////////////////////////////

#include "rrt_planner.h"
#include <iostream>
#include <glog/logging.h>

namespace path {

  // The algorithm. See header for references.
  Trajectory::Ptr RRTPlanner::PlanTrajectory() {

    // Check if a path already exists.
    if (tree_.Contains(goal_))
      return tree_.GetTrajectory(goal_);

    // Initialize the tree.
    tree_.Insert(origin_);

    // Algorithm:
    // 1. Choose a random point.
    // 2. Take a step toward that point if possible.
    Point::Ptr last_point;
    while (!tree_.Contains(goal_) && tree_.Size() < 10000) {
      std::cout << "Tree size: " << tree_.Size() << std::endl;

      // Pick a random point in the scene.
      Point::Ptr random_point = scene_.GetRandomPoint();
      std::cout << "Random point : " << random_point->GetVector().transpose() << std::endl;

      // Find nearest point in the tree.
      Point::Ptr nearest = tree_.GetNearest(random_point);
      std::cout << "Nearest neighbor : " << nearest->GetVector().transpose() << std::endl;

      // Take a step toward the random point.
      Point::Ptr step = nearest->StepToward(random_point, step_size_);
      std::cout << "Step : " << step->GetVector().transpose() << std::endl;
      if (robot_.LineOfSight(nearest, step)) {
        std::cout << "Passed line of sight test. Inserting..." << std::endl;
        if (!tree_.Insert(step)) {
          VLOG(1) << "Could not insert this point. Skipping.";
          std::cout << "Could not insert this point. Skipping." << std::endl;
        }
        last_point = step;
      }

      // Insert the goal if it is visible.
      std::cout << "Checking if goal is visible..." << std::endl;
      if (robot_.LineOfSight(step, goal_)) {
        if (!tree_.Insert(goal_, step))
          VLOG(1) << "Could not insert the goal point.";
        std::cout << "Success! Inserted goal." << std::endl;
        std::cout << tree_.Contains(goal_) << std::endl;
        last_point = goal_;
      }
    }

    // Return the trajectory.
    return tree_.GetTrajectory(last_point);
  }

} //\ namespace path
