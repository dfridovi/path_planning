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
// This class defines the Trajectory2D datatype. It operates on Point2D objects.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_TRAJECTORY_2D_H
#define PATH_PLANNING_TRAJECTORY_2D_H

#include <geometry/point_2d.h>
#include <vector>
#include <list>

namespace path {

  // A Trajectory is just an ordered list of points.
  class Trajectory2D {
  public:
    typedef std::shared_ptr<Trajectory2D> Ptr;

    ~Trajectory2D() {}

    // Factory methods.
    static Ptr Create();
    static Ptr Create(std::vector<Point2D::Ptr>& points);
    static Ptr Create(std::list<Point2D::Ptr>& points);

    // Recompute length.
    void RecomputeLength();

    // Add a point to the path.
    void AddPoint(Point2D::Ptr point);

    // Upsample by adding k points linearly between each pair of points
    // in this Trajectory2D.
    void Upsample(unsigned int k);

    // Compute the (time) derivative of the path, assuming uniform sampling.
    // Calculate at the endpoints by implicit padding.
    Trajectory2D::Ptr TimeDerivative();

    // Getters.
    double GetLength() const;
    std::vector<Point2D::Ptr>& GetPoints();
    Point2D::Ptr GetAt(size_t index);

    // Setter.
    void SetAt(Point2D::Ptr point, size_t index);

  private:
    std::vector<Point2D::Ptr> points_;
    double length_;

    // Private constructors. Use factory methods instead.
    Trajectory2D();
    Trajectory2D(std::vector<Point2D::Ptr>& points);
    Trajectory2D(std::list<Point2D::Ptr>& points);
  };

} //\ namespace path

#endif
