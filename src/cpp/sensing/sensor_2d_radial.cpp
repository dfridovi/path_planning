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
// This class defines the simplest type of sensor model -- a 2D radially
// symmetric sensor.
//
///////////////////////////////////////////////////////////////////////////////

#include "sensor_2d_radial.h"
#include <geometry/point.h>
#include <geometry/point_2d.h>
#include <geometry/orientation_2d.h>
#include <robot/robot_2d_circular.h>

#include <glog/logging.h>

using Eigen::MatrixXi;

namespace path {

  // How many known obstacles are visible to the robot?
  int Sensor2DRadial::GetObstacleCount(Point::Ptr pose) const {
    CHECK_NOTNULL(pose.get());

    // Check pose type.
    if (!pose->IsType(Point::PointType::ORIENTATION_2D)) {
      VLOG(1) << "Point (pose) is not of type ORIENTATION_2D. Returning -1.";
      return -1;
    }

    // Create robot and test point.
    Robot2DCircular robot(grid_.GetScene(), 0.0 /* radius */);
    Orientation2D *orientation =
      std::static_pointer_cast<Orientation2D>(pose).get();
    Point::Ptr location = orientation->GetPoint();

    // Get bounding box.
    VectorXd location_vector = location->GetVector();
    double xmin = std::max(location_vector(0) - radius_, grid_.GetXMin());
    double xmax = std::min(location_vector(0) + radius_, grid_.GetXMax());
    double ymin = std::max(location_vector(1) - radius_, grid_.GetYMin());
    double ymax = std::min(location_vector(1) + radius_, grid_.GetYMax());
    double step = grid_.GetBlockSize();

    Point::Ptr tl_bin = grid_.GetBinCenter(Point2D::Create(xmin, ymax));
    Point::Ptr br_bin = grid_.GetBinCenter(Point2D::Create(xmax, ymin));
    xmin = tl_bin->GetVector()(0);
    ymax = tl_bin->GetVector()(1);
    xmax = br_bin->GetVector()(0);
    ymin = br_bin->GetVector()(1);

    // For all visible points, check line of sight.
    int obstacle_count = 0;
    for (double y = ymax; y >= ymin; y -= step) {
      for (double x = xmin; x <= xmax; x += step) {
        Point::Ptr test = Point2D::Create(x, y);
        Point::Ptr bin = grid_.GetBinCenter(test);
        if (!bin) continue;

        // Check in range.
        if (location->DistanceTo(bin) > radius_) {
          continue;
        }

        // Check occupancy.
        int occupancy = grid_.GetCountAt(test);
        if (occupancy == 0) {
          continue;
        }

        // Check line of sight.
        Point::Ptr close = bin->StepToward(location, step);
        if (robot.LineOfSight(location, close)) {
          obstacle_count += occupancy;
        }
      }
    }

    return obstacle_count;
  }

  // Visualize which voxels are visible to this robot.
  void Sensor2DRadial::Visualize(Point::Ptr pose,
                                 const std::string& title) const {
    CHECK_NOTNULL(pose.get());

    // Check pose type.
    if (!pose->IsType(Point::PointType::ORIENTATION_2D)) {
      VLOG(1) << "Point (pose) is not of type ORIENTATION_2D.";
      return;
    }

    // Create robot and test point.
    Robot2DCircular robot(grid_.GetScene(), 0.0 /* radius */);
    Orientation2D *orientation =
      std::static_pointer_cast<Orientation2D>(pose).get();
    Point::Ptr location = orientation->GetPoint();

    // Create sensor view matrix.
    MatrixXf view_matrix = MatrixXf::Zero(grid_.GetNRows(), grid_.GetNCols());

    // Get bounding box.
    VectorXd location_vector = location->GetVector();
    double xmin = std::max(location_vector(0) - radius_, grid_.GetXMin());
    double xmax = std::min(location_vector(0) + radius_, grid_.GetXMax());
    double ymin = std::max(location_vector(1) - radius_, grid_.GetYMin());
    double ymax = std::min(location_vector(1) + radius_, grid_.GetYMax());
    double step = grid_.GetBlockSize();

    Point::Ptr tl_bin = grid_.GetBinCenter(Point2D::Create(xmin, ymax));
    Point::Ptr br_bin = grid_.GetBinCenter(Point2D::Create(xmax, ymin));
    xmin = tl_bin->GetVector()(0);
    ymax = tl_bin->GetVector()(1);
    xmax = br_bin->GetVector()(0);
    ymin = br_bin->GetVector()(1);

    // For all visible points, check line of sight.
    for (double y = ymax; y >= ymin; y -= step) {
      for (double x = xmin; x <= xmax; x += step) {
        Point::Ptr test = Point2D::Create(x, y);
        Point::Ptr bin = grid_.GetBinCenter(test);
        if (!bin) continue;

        // Check in range.
        if (location->DistanceTo(bin) > radius_) {
          continue;
        }

        // Check line of sight.
        Point::Ptr close = bin->StepToward(location, step);
        if (robot.LineOfSight(location, close)) {
          int jj = static_cast<int>((x - grid_.GetXMin()) / step);
          int ii = static_cast<int>((y - grid_.GetYMin()) / step);
          ii = grid_.GetNRows() - ii - 1;
          if (view_matrix(ii, jj) > 0.0) {
            std::cout << "Double-counting a bin." << std::endl;
          }

          view_matrix(ii, jj) = 1.0;
        }
      }
    }

    // Display.
    Image view_image(view_matrix);
    view_image.ImShow(title);
  }


} // \namespace path
