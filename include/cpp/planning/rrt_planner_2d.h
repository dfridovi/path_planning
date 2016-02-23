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
// This file defines the RRT (rapidly-exploring random tree) class.
// It is implemented according to the following descriptions:
// + https://www.cs.cmu.edu/~motionplanning/lecture/lec20.pdf
// + http://msl.cs.uiuc.edu/rrt/about.html
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_RRT_PLANNER_2D_H
#define PATH_PLANNING_RRT_PLANNER_2D_H

#include <geometry/trajectory_2d.h>
#include <geometry/point_2d.h>
#include <geometry/rrt_2d.h>
#include <robot/robot_2d_circular.h>
#include <scene/scene_2d_continuous.h>
#include <util/types.h>
#include <util/disallow_copy_and_assign.h>

namespace path {

  // Derived from base class Planner.
  class RRTPlanner2D {
  public:
    RRTPlanner2D(Robot2DCircular& robot, Scene2DContinuous& scene,
                 Point2D::Ptr origin, Point2D::Ptr goal, float step_size = 0.1)
      : robot_(robot), scene_(scene),
        origin_(origin), goal_(goal),
        step_size_(step_size) {}
    ~RRTPlanner2D() {}

    // The algorithm. See header for references.
    Trajectory2D::Ptr PlanTrajectory();

  private:
    Robot2DCircular& robot_;
    Scene2DContinuous& scene_;
    Point2D::Ptr origin_;
    Point2D::Ptr goal_;

    RRT2D tree_;
    const float step_size_;

    DISALLOW_COPY_AND_ASSIGN(RRTPlanner2D)
  };

} //\ namespace path

#endif
