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

#include <scene/scene_2d_continuous.h>
#include <math/random_generator.h>
#include <geometry/point_2d.h>

#include <glog/logging.h>
#include <Eigen/Dense>
#include <memory>
#include <iostream>

using Eigen::MatrixXf;
using Eigen::Vector2d;

namespace path {

  // Dummy constructor. MUST call SetBounds after this.
  Scene2DContinuous::Scene2DContinuous()
    : largest_obstacle_radius_(0.0),
      xmin_(0.0), xmax_(0.0),
      ymin_(0.0), ymax_(0.0) {}

  // Better to use these constructors if possible.
  Scene2DContinuous::Scene2DContinuous(float xmin, float xmax,
                                       float ymin, float ymax)
    : largest_obstacle_radius_(0.0),
      xmin_(xmin), xmax_(xmax),
      ymin_(ymin), ymax_(ymax) {}

  Scene2DContinuous::Scene2DContinuous(float xmin, float xmax,
                                       float ymin, float ymax,
                                       std::vector<Obstacle2D::Ptr>& obstacles)
    : obstacles_(obstacles),
      xmin_(xmin), xmax_(xmax),
      ymin_(ymin), ymax_(ymax) {

    // Largest obstacle radius.
    largest_obstacle_radius_ = 0.0;
    for (const auto& obstacle : obstacles) {
      CHECK_NOTNULL(obstacle.get());

      if (obstacle->GetRadius() > largest_obstacle_radius_)
        largest_obstacle_radius_ = obstacle->GetRadius();
    }

    // Obstacle tree.
    obstacle_tree_.AddObstacles(obstacles);
  }

  // Add an obstacle.
  void Scene2DContinuous::AddObstacle(Obstacle2D::Ptr obstacle) {
    CHECK_NOTNULL(obstacle.get());

    obstacles_.push_back(obstacle);
    obstacle_tree_.AddObstacle(obstacle);
  }

  // Get obstacles.
  std::vector<Obstacle2D::Ptr>& Scene2DContinuous::GetObstacles() {
    return obstacles_;
  }

  FlannObstacle2DTree& Scene2DContinuous::GetObstacleTree() {
    return obstacle_tree_;
  }

  float Scene2DContinuous::GetLargestObstacleRadius() const {
    return largest_obstacle_radius_;
  }

  int Scene2DContinuous::GetObstacleCount() const {
    return static_cast<int>(obstacles_.size());
  }

  // Setter.
  void Scene2DContinuous::SetBounds(float xmin, float xmax,
                                    float ymin, float ymax) {
    xmin_ = xmin;
    xmax_ = xmax;
    ymin_ = ymin;
    ymax_ = ymax;
  }

  // Is this point feasible?
  bool Scene2DContinuous::IsFeasible(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Get a list of all obstacles in range.
    std::vector<Obstacle2D::Ptr> obstacles_in_range;
    if (!obstacle_tree_.RadiusSearch(point, obstacles_in_range,
                                     largest_obstacle_radius_)) {
      VLOG(1) << "Radius search failed during feasibility test. "
              << "Returning false.";
      return false;
    }

    // Check each obstacle.
    for (const auto& obstacle : obstacles_in_range) {
      if (!obstacle->IsFeasible(point))
        return false;
    }

    return true;
  }

  // What is the cost of occupying this point? For speed, only compute
  // cost from obstacles within a specific radius.
  float Scene2DContinuous::Cost(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Get a list of all obstacles in range.
    std::vector<Obstacle2D::Ptr> obstacles_in_range;
    if (!obstacle_tree_.RadiusSearch(point, obstacles_in_range,
                                     10.0 * largest_obstacle_radius_)) {
      VLOG(1) << "Radius search failed during cost evaluation. "
              << "Returning infinite cost.";
      return std::numeric_limits<float>::infinity();
    }

    // Iterate over all obstacles in range.
    float total_cost = 0.0;
    for (const auto& obstacle : obstacles_in_range)
      total_cost += obstacle->Cost(point);

    return total_cost;
  }

  // Compute the derivative of cost by position. This is used for
  // trajectory optimization.
  Point2D::Ptr Scene2DContinuous::CostDerivative(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Get a list of all obstacles in range.
    std::vector<Obstacle2D::Ptr> obstacles_in_range;
    if (!obstacle_tree_.RadiusSearch(point, obstacles_in_range,
                                     10.0 * largest_obstacle_radius_)) {
      VLOG(1) << "Radius search failed during derivative evaluation. "
              << "Returning zero derivative.";
      return Point2D::Create(0.0, 0.0);
    }

    // Iterate over all obstacles in range.
    Vector2d total_derivative = Vector2d::Zero();
    for (const auto& obstacle : obstacles_in_range) {
      Point2D::Ptr derivative = obstacle->Derivative(point);
      total_derivative(0) += derivative->x;
      total_derivative(1) += derivative->y;
    }

    return Point2D::Create(total_derivative(0), total_derivative(1));
  }

  // Get a random point in the scene.
  Point2D::Ptr Scene2DContinuous::GetRandomPoint() const {
    float x = static_cast<float>(rng_.DoubleUniform(xmin_, xmax_));
    float y = static_cast<float>(rng_.DoubleUniform(ymin_, ymax_));
    return Point2D::Create(x, y);
  }

  // Optimize the given trajectory to minimize cost.
  Trajectory2D::Ptr Scene2DContinuous::OptimizeTrajectory(
                                                 Trajectory2D::Ptr path,
                                                 float gradient_weight,
                                                 float curvature_penalty,
                                                 float max_point_displacement,
                                                 float min_avg_displacement,
                                                 size_t max_iters) const {
    CHECK_NOTNULL(path.get());

    // Extract points from the given trajectory.
    std::vector<Point2D::Ptr> points(path->GetPoints());
    float num_points = static_cast<float>(points.size());

    // Create a new trajectory from these soon-to-be-optimized points.
    Trajectory2D::Ptr optimized = Trajectory2D::Create(points);

    // While the average displacement of all points is large and the total
    // number of iterations does not exceed the threshold, iterate over
    // all points and move each point a small distance in the direction
    // of the negative gradient.
    float total_displacement = std::numeric_limits<float>::infinity();
    size_t num_iters = 0;
    while (total_displacement / num_points > min_avg_displacement &&
           num_iters < max_iters) {
      total_displacement = 0.0;

      // Compute second time derivative of the trajectory.
      Trajectory2D::Ptr time_derivative1 = optimized->TimeDerivative();
      Trajectory2D::Ptr time_derivative2 = time_derivative1->TimeDerivative();

      for (size_t ii = 1; ii < points.size() - 1; ii++) {
        Point2D::Ptr point = optimized->GetAt(ii);
        Point2D::Ptr cost_derivative = CostDerivative(point);
        Point2D::Ptr time_derivative = time_derivative2->GetAt(ii);

        Point2D::Ptr derivative = Point2D::Add(cost_derivative, time_derivative,
                                               -curvature_penalty);
        float step_x = point->x - gradient_weight * derivative->x;
        float step_y = point->y - gradient_weight * derivative->y;
        Point2D::Ptr initial_step = Point2D::Create(step_x, step_y);
        Point2D::Ptr truncated_step = Point2D::StepToward(point, initial_step,
                                                          max_point_displacement);
        total_displacement += Point2D::DistancePointToPoint(point,
                                                            truncated_step);

        optimized->SetAt(truncated_step, ii);
      }

      num_iters++;
    }

    return optimized;
  }

  // Visualize this scene. Optionally pass in the number of pixels
  // in the x-direction.
  void Scene2DContinuous::Visualize(const std::string& title, int xsize) const {
    Visualize(title, Trajectory2D::Ptr(nullptr), xsize);
  }

  // Visualize a Trajectory in this scene. Optionally pass in the
  // number of pixels in the x-direction.
  void Scene2DContinuous::Visualize(const std::string& title,
                                    Trajectory2D::Ptr path, int xsize) const {

    // Initialize a matrix to represent the scene.
    int ysize = static_cast<int>(static_cast<float>(xsize) *
                                 (ymax_ - ymin_) / (xmax_ - xmin_));
    MatrixXf map_matrix(xsize, ysize);
    map_matrix = MatrixXf::Zero(xsize, ysize);

    // Calculate cost at each pixel.
    for (size_t ii = 0; ii < ysize; ii++) {
      for (size_t jj = 0; jj < xsize; jj++) {
        float x = xmin_ + (xmax_ - xmin_) *
          static_cast<float>(jj) / static_cast<float>(xsize);
        float y = ymin_ + (ymax_ - ymin_) *
          static_cast<float>(ysize - ii) / static_cast<float>(ysize);
        Point2D::Ptr point = Point2D::Create(x, y);

        map_matrix(ii, jj) = Cost(point);
      }
    }

    // Normalize.
    map_matrix /= map_matrix.maxCoeff();

    // Set infeasible points to 1.0.
    for (size_t ii = 0; ii < ysize; ii++) {
      for (size_t jj = 0; jj < xsize; jj++) {
        float x = xmin_ + (xmax_ - xmin_) *
          static_cast<float>(jj) / static_cast<float>(xsize);
        float y = ymin_ + (ymax_ - ymin_) *
          static_cast<float>(ysize - ii) / static_cast<float>(ysize);
        Point2D::Ptr point = Point2D::Create(x, y);

        if (!IsFeasible(point))
          map_matrix(ii, jj) = 1.0;
      }
    }

    // Convert to an Image.
    Image map_image(map_matrix);

    // Add Trajectory.
    if (path) {
      CHECK_NOTNULL(path.get());

      // Convert to RGB.
      map_image.ConvertToRGB();

      // Draw each point, and put line segments between them.
      Point2D::Ptr last_point;
      size_t cnt = 0;
      for (const auto& next_point : path->GetPoints()) {
        float u1, v1;
        u1 = static_cast<float>(ysize) * (1.0 - (next_point->y - ymin_) / (ymax_ - ymin_));
        v1 = static_cast<float>(xsize) * (next_point->x - xmin_) / (xmax_ - xmin_);

        float heat = static_cast<float>(cnt) /
          static_cast<float>(path->GetPoints().size() - 1);
        map_image.Circle(static_cast<unsigned int>(v1),
                         static_cast<unsigned int>(u1),
                         4,  // radius
                         2,  // line thickness
                         heat);

        if (last_point) {
          unsigned int u2, v2;
          u2 = static_cast<float>(ysize) *
            (1.0 - (last_point->y - ymin_) / (ymax_ - ymin_));
          v2 = static_cast<float>(xsize) *
            (last_point->x - xmin_) / (xmax_ - xmin_);

          map_image.Line(static_cast<unsigned int>(v2), static_cast<unsigned int>(u2),
                         static_cast<unsigned int>(v1), static_cast<unsigned int>(u1),
                         2, // line thickness
                         heat);
        }

        last_point = next_point;
        cnt++;
      }
    }

    // Display.
    map_image.ImShow(title);
  }

} // \namespace path
