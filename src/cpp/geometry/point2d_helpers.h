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
// This is a helper class to do various useful operations on Point2D objects.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_POINT2D_HELPERS_H
#define PATH_PLANNING_POINT2D_HELPERS_H

#include "../util/types.h"

namespace path {

  // Helper functions for Point2D.
  struct Point2DHelpers {

    // Create a Point2D.
    static Point2D& Create(float x, float y);

    // Midpoint.
    static Point2D& MidPoint(Point2D& point1, Point2D& point2);

    // Distance between a point and this line segment.
    static float DistanceLineToPoint(Point2D& point1,
                                     Point2D& point2,
                                     Point2D& point3);

    // Distance between two points.
    static float DistancePointToPoint(Point2D& point1,
                                      Point2D& point2);

    // Take a step toward point2 from point1.
    static Point2D& StepToward(Point2D& point1, Point2D& point2,
                               float step_size);

    // Add two points with a scale factor.
    static Point2D& Add(Point2D& point1, Point2D& point2,
                        float scale);
  }; //\ struct Point2DHelpers
  
} //\ namespace path

#endif
