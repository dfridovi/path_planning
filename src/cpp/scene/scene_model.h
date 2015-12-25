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
// This class defines the base struct for all scene models. Derive from this
// class to parameterize a particular scene (e.g. R^3, with known obstacles).
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_SCENE_MODEL_H
#define PATH_PLANNING_SCENE_MODEL_H

#include <geometry/point.h>
#include <geometry/line_segment.h>
#include <robot/robot_model.h>
#include "obstacle.h"
#include <util/disallow_copy_and_assign.h>

#include <vector>

namespace path {

  // Derive from this class when defining a specific scene model.
  class SceneModel {
  public:
    SceneModel() {}
    virtual ~SceneModel() {}

    // Get obstacles.
    virtual inline std::vector<Obstacle::Ptr>& GetObstacles();

    // Define these methods in a derived class.
    virtual bool IsFeasible(Point::Ptr point) const = 0;
    virtual double Cost(Point::Ptr point) const = 0;
    virtual Point::Ptr GetRandomPoint() const = 0;
    virtual bool LineOfSight(Point::Ptr point1, Point::Ptr point2,
                             RobotModel& robot) const = 0;

  protected:
    std::vector<Obstacle::Ptr> obstacles_;

  private:
    DISALLOW_COPY_AND_ASSIGN(SceneModel);
  };

// ---------------------------- Implementation ------------------------------ //

  std::vector<Obstacle::Ptr>& SceneModel::GetObstacles() { return obstacles_; }

} // \namespace path

#endif
