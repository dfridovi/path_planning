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

#ifndef PATH_PLANNING_POINT_TREE_H
#define PATH_PLANNING_POINT_TREE_H

#include "point.h"
#include "trajectory.h"
#include "flann_point_kdtree.h"
#include "nary_node.h"
#include <util/disallow_copy_and_assign.h>

#include <memory>
#include <vector>
#include <unordered_map>
#include <glog/logging.h>

namespace path {

  // N-ary tree of Points.
  class PointTree {
  public:
    PointTree() {}
    ~PointTree() {}

    // Insert a point. Returns true if successful.
    bool Insert(Point::Ptr point);

    // Does the tree contain this point?
    bool Contains(Point::Ptr point);

    // Get nearest point in the tree.
    Point::Ptr GetNearest(Point::Ptr point);

    // Get the path from the head to a particular goal point.
    Trajectory::Ptr GetTrajectory(Point::Ptr goal);

  private:
    Node::Ptr head_;
    std::unordered_map<Point::Ptr, Node::Ptr> registry_;
    FlannPointKDTree kd_tree_;

    DISALLOW_COPY_AND_ASSIGN(PointTree);
  };

} //\ namespace path

#endif
