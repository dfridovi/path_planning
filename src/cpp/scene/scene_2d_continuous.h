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
// This class defines a 2D continuous scene, templated on the type of obstacle.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_SCENE_2D_CONTINUOUS_H
#define PATH_PLANNING_SCENE_2D_CONTINUOUS_H

#include "scene_model.h"
#include "obstacle.h"
#include <geometry/point.h>
#include <geometry/trajectory.h>
#include <image/image.h>

#include <vector>
#include <string>

namespace path {

  // Derived class to model 2D continuous scenes.
  class Scene2DContinuous : public SceneModel {
  public:
    Scene2DContinuous();
    Scene2DContinuous(double xmin, double xmax,
                      double ymin, double ymax);
    Scene2DContinuous(double xmin, double xmax,
                      double ymin, double ymax,
                      std::vector<Obstacle::Ptr>& obstacles);

    // Setter.
    void SetBounds(double xmin, double xmax, double ymin, double ymax);

    // Is this point feasible?
    bool IsFeasible(Point::Ptr point) const;

    // What is the cost of occupying this point?
    double Cost(Point::Ptr point) const;

    // Compute the derivative of cost by position. This is used for
    // trajectory optimization.
    Point::Ptr Derivative(Point::Ptr point) const;

    // Get a random point in the scene.
    Point::Ptr GetRandomPoint() const;

    // Optimize the given trajectory to minimize cost.
    Trajectory::Ptr OptimizeTrajectory(Trajectory::Ptr path,
                                       double gradient_weight,
                                       double min_avg_displacement,
                                       size_t max_iters) const;

    // Visualize this scene. Optionally pass in the number of pixels
    // in the x-direction.
    void Visualize(const std::string& title,
                   int xsize = 500) const;

    // Visualize a Trajectory in this scene. Optionally pass in the
    // number of pixels in the x-direction.
    void Visualize(const std::string& title,
                   Trajectory::Ptr path, int xsize = 500) const;

  private:
    double xmin_;
    double xmax_;
    double ymin_;
    double ymax_;
  };

} // \namespace path

#endif
