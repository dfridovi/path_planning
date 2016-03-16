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

#include <scene/obstacle_2d.h>
#include <geometry/point_2d.h>
#include <geometry/trajectory_2d.h>
#include <flann/flann_obstacle_2dtree.h>
#include <util/types.h>
#include <image/image.h>
#include <math/random_generator.h>

#include <vector>
#include <string>

namespace path {

  // Derived class to model 2D continuous scenes.
  class Scene2DContinuous {
  public:
    Scene2DContinuous();
    Scene2DContinuous(float xmin, float xmax,
                      float ymin, float ymax);
    Scene2DContinuous(float xmin, float xmax,
                      float ymin, float ymax,
                      std::vector<Obstacle2D::Ptr>& obstacles);

    // Add an obstacle.
    void AddObstacle(Obstacle2D::Ptr obstacle);

    // Get obstacles.
    std::vector<Obstacle2D::Ptr>& GetObstacles();
    FlannObstacle2DTree& GetObstacleTree();
    float GetLargestObstacleRadius() const;
    int GetObstacleCount() const;

    // Setter.
    void SetBounds(float xmin, float xmax, float ymin, float ymax);

    // Is this point feasible?
    bool IsFeasible(Point2D::Ptr point) const;

    // What is the cost of occupying this point?
    float Cost(Point2D::Ptr point) const;

    // Compute the derivative of cost by position. This is used for
    // trajectory optimization.
    Point2D::Ptr CostDerivative(Point2D::Ptr point) const;

    // Get a random point in the scene.
    Point2D::Ptr GetRandomPoint() const;

    // Optimize the given trajectory to minimize cost.
    Trajectory2D::Ptr OptimizeTrajectory(Trajectory2D::Ptr path,
                                         float gradient_weight = 1e-6,
                                         float curvature_penalty = 1e-5,
                                         float max_point_displacement = 1e-2,
                                         float min_avg_displacement = 1e-4,
                                         size_t max_iters = 100) const;

    // Visualize this scene. Optionally pass in the number of pixels
    // in the x-direction.
    void Visualize(const std::string& title,
                   int xsize = 500) const;

    // Visualize a Trajectory in this scene. Optionally pass in the
    // number of pixels in the x-direction.
    void Visualize(const std::string& title,
                   Trajectory2D::Ptr path, int xsize = 500) const;

  private:
    std::vector<Obstacle2D::Ptr> obstacles_;
    FlannObstacle2DTree obstacle_tree_;
    math::RandomGenerator rng_;
    float largest_obstacle_radius_;

    float xmin_;
    float xmax_;
    float ymin_;
    float ymax_;

    DISALLOW_COPY_AND_ASSIGN(Scene2DContinuous);
  };

} // \namespace path

#endif
