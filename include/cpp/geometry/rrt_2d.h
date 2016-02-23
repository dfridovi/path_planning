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
// This class defines an N-ary tree of Points.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_RRT_2D_H
#define PATH_PLANNING_RRT_2D_H

#include <geometry/trajectory_2d.h>
#include <geometry/point_2d.h>
#include <geometry/node_2d.h>
#include <util/disallow_copy_and_assign.h>
#include <flann/flann_point_2dtree.h>

#include <memory>
#include <vector>
#include <unordered_map>

namespace path {

  // N-ary tree of Point2Ds.
  class RRT2D {
  public:
    RRT2D() {}
    ~RRT2D() {}

    // Insert a point. Returns true if successful.
    bool Insert(Point2D::Ptr point);
    bool Insert(Point2D::Ptr point, Point2D::Ptr parent);

    // Does the tree contain this point?
    bool Contains(Point2D::Ptr point) const;

    // Tree size.
    int Size() const;

    // Get nearest point in the tree.
    Point2D::Ptr GetNearest(Point2D::Ptr point);

    // Get the path from the head to a particular goal point.
    Trajectory2D::Ptr GetTrajectory(Point2D::Ptr goal);

  private:
    Node2D::Ptr head_;
    std::unordered_map<Point2D::Ptr, Node2D::Ptr> registry_;
    FlannPoint2DTree kd_tree_;

    DISALLOW_COPY_AND_ASSIGN(RRT2D);
  };

} //\ namespace path

#endif
