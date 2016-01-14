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

#ifndef PATH_PLANNING_SENSOR_MODEL_2D_RADIAL_H
#define PATH_PLANNING_SENSOR_MODEL_2D_RADIAL_H

#include "sensor_model.h"
#include <occupancy/occupancy_grid_2d.h>

namespace path {

  // Derive from this class when defining a specific type of sensor model.
  class Sensor2DRadial : public SensorModel {
  public:
    Sensor2DRadial(OccupancyGrid2D& grid, double radius)
      : grid_(grid), radius_(radius) {}
    ~Sensor2DRadial() {}

    // How many known obstacles are visible to the robot?
    int GetObstacleCount(Point::Ptr pose) const;

    // Visualize which voxels are visible to this robot.
    void Visualize(Point::Ptr pose,
                   const std::string& title = std::string()) const;

  private:
    OccupancyGrid2D& grid_;
    const double radius_;
  };

} // \namespace path

#endif
