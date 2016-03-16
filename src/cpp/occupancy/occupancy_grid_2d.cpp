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
// This class defines a 2D occupancy grid.
//
///////////////////////////////////////////////////////////////////////////////

#include <occupancy/occupancy_grid_2d.h>
#include <geometry/point_2d.h>
#include <scene/obstacle_2d.h>

#include <cmath>
#include <glog/logging.h>

using Eigen::MatrixXf;

namespace path {

  // Constructor.
  OccupancyGrid2D::OccupancyGrid2D(float xmin, float xmax,
                                   float ymin, float ymax,
                                   float block_size)
    : xmin_(xmin), xmax_(xmax),
      ymin_(ymin), ymax_(ymax), count_(0) {

    // Set scene.
    scene_.SetBounds(xmin, xmax, ymin, ymax);

    // Determine nrows and ncols.
    nrows_ = static_cast<int>(std::ceil((ymax_ - ymin_) / block_size));
    ncols_ = static_cast<int>(std::ceil((xmax_ - xmin_) / block_size));

    block_size_ = std::max((xmax_ - xmin_) / static_cast<float>(ncols_),
                           (ymax_ - ymin_) / static_cast<float>(nrows_));

    // Create the grid.
    grid_ = MatrixXi::Zero(nrows_, ncols_);
    for (size_t ii = 0; ii < nrows_; ii++)
      for (size_t jj = 0; jj < ncols_; jj++)
        grid_(ii, jj) = 0;
  }

  // Insert a point.
  void OccupancyGrid2D::Insert(Point2D::Ptr point) {
    if (!IsValidPoint(point)) return;

    // Find the nearest bin and insert.
    int jj = static_cast<int>((point->x - xmin_) / block_size_);
    int ii = static_cast<int>((point->y - ymin_) / block_size_);
    ii = nrows_ - ii - 1;
    grid_(ii, jj)++;

    // Increment count_.
    count_++;

    // Add to scene if bin is empty.
    if (grid_(ii, jj) == 1) {
      Point2D::Ptr bin_center = GetBinCenter(point);
      Obstacle2D::Ptr obstacle =
        Obstacle2D::Create(bin_center->x, bin_center->y, 0.5 * block_size_);
      scene_.AddObstacle(obstacle);
    }
  }

  // Get number of points in the bin containing the specified point.
  int OccupancyGrid2D::GetCountAt(Point2D::Ptr point) const {
    if (!IsValidPoint(point)) return -1;

    // Get count.
    int jj = static_cast<int>((point->x - xmin_) / block_size_);
    int ii = static_cast<int>((point->y - ymin_) / block_size_);
    ii = nrows_ - ii - 1;

    return grid_(ii, jj);
  }

  // Return the center of the bin which includes the given point.
  Point2D::Ptr OccupancyGrid2D::GetBinCenter(Point2D::Ptr point) const {
    if (!IsValidPoint(point)) return nullptr;

    // Get rounded coordinates.
    int jj = static_cast<int>((point->x - xmin_) / block_size_);
    int ii = static_cast<int>((point->y - ymin_) / block_size_);

    return Point2D::Create((static_cast<float>(jj) + 0.5) * block_size_,
                           (static_cast<float>(ii) + 0.5) * block_size_);
  }

  // Visualize this occupancy grid.
  void OccupancyGrid2D::Visualize(const std::string& title) const {
    MatrixXf map_matrix = grid_.cast<float>();
    map_matrix /= map_matrix.maxCoeff();

    // Convert to an Image and display.
    Image map_image(map_matrix);
    map_image.ImShow(title);
  }

  // Check if a point is valid.
  bool OccupancyGrid2D::IsValidPoint(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());

    // Check bounds.
    if (point->x < xmin_ || point->x > xmax_ ||
        point->y < ymin_ || point->y > ymax_) {
      VLOG(1) << "Error. Point is out of bounds.";
      return false;
    }

    return true;
  }

} // \namespace path
