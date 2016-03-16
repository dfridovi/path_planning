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
 * Author: James Smith   ( james.smith@berkeley.edu )
 */

#include <mapper/mapper.h>
#include <iostream>

namespace path {

  Mapper::Mapper()
    : cull_saturated_(true) {}

  Mapper::Mapper(bool cull_saturated)
    : cull_saturated_(cull_saturated) {}

  pcl::PointCloud<pcl::PointXYZ> Mapper::ProjectDepthMap(const DepthMap& map) const {
    // Unproject all of the points in the depth map before adding them to the point cloud
    // The reason is because we dont know how many points exactly we will unproject, 
    // considering that some of them may be culled.
    std::vector<Vector3d> points;
    for (size_t u = 0; u < map.Width(); ++u) {
      for (size_t v = 0; v < map.Height(); ++v) {
        if (!cull_saturated_ || !map.SaturatedAt(u, v)) {
            points.push_back(map.Unproject(u, v));
          }
      }
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.is_dense = false;
    // TODO: Include sensor orientation information?

    cloud.points.resize(points.size());
    size_t ii = 0;
    for (auto const& point : points) {
        cloud.points[ii].x = point(0);
        // JDS: Not sure why the projections are coming upside down
        cloud.points[ii].y = -point(1);
        cloud.points[ii].z = point(2);
        ii++;
    }

    return cloud;
  }

  void Mapper::AddDepthMap(const DepthMap& map) {
    pcl::PointCloud<pcl::PointXYZ> cloud = ProjectDepthMap(map);
    cloud_ = cloud_ + cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> Mapper::GetMap() const {
    return cloud_;
  }
}
