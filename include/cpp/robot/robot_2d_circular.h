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
// This class defines a simple circular robot in two dimensions.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_ROBOT_2D_CIRCULAR_H
#define PATH_PLANNING_ROBOT_2D_CIRCULAR_H

#include <util/types.h>
#include <geometry/point_2d.h>
#include <scene/scene_2d_continuous.h>

namespace path {

  // Simple 2D circular robot.
  class Robot2DCircular {
  public:
    Robot2DCircular(Scene2DContinuous& scene, float radius)
      : scene_(scene), radius_(radius) {}
    ~Robot2DCircular() {}

    // Test if a particular point is feasible.
    bool IsFeasible(Point2D::Ptr point);
    bool LineOfSight(Point2D::Ptr point1, Point2D::Ptr point2) const;

  private:
    Scene2DContinuous& scene_;
    float radius_;

    DISALLOW_COPY_AND_ASSIGN(Robot2DCircular);

  };

} // \namespace path

#endif
