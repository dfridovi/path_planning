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
// This class defines the base class for all robot models. Derive from this
// class to parameterize a particular robot (e.g. the DJI Matrice 100).
//
// The idea here is to provide a way to specify constraints on a robot's
// configuration. For example, a RobotModel can test whether or not it is
// occupying free space in a SceneModel.
//
// For simplicity, we assume that all robots are bounded by a sphere.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_ROBOT_MODEL_H
#define PATH_PLANNING_ROBOT_MODEL_H

#include <scene/scene_model.h>
#include <geometry/point.h>
#include <flann/flann_obstacle_kdtree.h>
#include <util/disallow_copy_and_assign.h>

namespace path {

  // Derive from this class when defining a specific robot model.
  class RobotModel {
  public:
    inline RobotModel(SceneModel& scene, double radius);
    virtual ~RobotModel() {}

    // Get radius.
    virtual inline double GetRadius() const;

    // Define these methods in a derived class.
    virtual bool IsFeasible(Point::Ptr location) = 0;
    virtual bool LineOfSight(Point::Ptr point1, Point::Ptr point2) const = 0;

  protected:
    SceneModel& scene_;
    FlannObstacleKDTree obstacle_tree_;
    double radius_;

  private:
    DISALLOW_COPY_AND_ASSIGN(RobotModel);
  };

// ---------------------------- Implementation ------------------------------ //

  RobotModel::RobotModel(SceneModel& scene, double radius)
    : scene_(scene), radius_(radius) {

    // Extract all obstacles and add to kd_tree_.
    std::vector<Obstacle::Ptr>& obstacles = scene_.GetObstacles();
    obstacle_tree_.AddObstacles(obstacles);
  }

  double RobotModel::GetRadius() const { return radius_; }

} // \namespace path

#endif
