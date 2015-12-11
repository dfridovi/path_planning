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
// This file defines the base class for all motion planners. For example,
// an RRT implementation could be derived from this class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <path/path.h>
#include <geometry/point.h>

namespace path {

  // Derive from this class when defining a specific path planner.
  template <typename RobotModelType, typename SceneModelType>
    class Planner {
  public:
    Planner() {}
    virtual ~Planner() {}

    // Set robot and scene models.
    virtual inline void SetRobotModel(RobotModelType& robot) {}
    virtual inline void SetSceneModel(SceneModelType& scene) {}

    // Define these methods in a derived class.
    virtual Path PlanPath() const = 0;

  private:
    RobotModelType robot_;
    SceneModelType scene_;
  }

// ---------------------------- Implementation ------------------------------ //

  template<typename RobotModelType, typename SceneModelType>
    void Planner<RobotModelType,
                 SceneModelType>::SetRobotModel(RobotModelType& robot) {
    robot_ = robot;
  }

  template<typename RobotModelType, typename SceneModelType>
    void Planner<RobotModelType,
                 SceneModelType>::SetSceneModel(SceneModelType& scene) {
    scene_ = scene;
  }

} // \namespace path

#endif
