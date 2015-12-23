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
// This class defines an N-ary tree of Points for use in an RRT. Insertion
// is based on finding the nearest neighbor.
//
///////////////////////////////////////////////////////////////////////////////

#include "point_tree.h"
#include <list>

namespace path {

  PointTree::PointTree();

  // Insert a point. Returns true if successful.
  bool PointTree::Insert(Point::Ptr point) {
    CHECK_NOTNULL(point.get());

    // Base case -- empty tree.
    if (registry_.size() == 0) {
      head_ = Node<Point::Ptr>::Create(point);
      registry_.emplace(point, head_);
      kd_tree_.AddPoint(point);
      return true;
    }

    // Don't insert if the tree already contains this point.
    if (Contains(point)) return false;

    // Find nearest point.
    Point::Ptr nearest = GetNearest(point);

    // Make a new Node and insert.
    Node<Point::Ptr>::Ptr node = Node<Point::Ptr>::Create(point);

    const auto match = registry_.find(nearest);
    CHECK_NOTNULL(match);
    Node<Point::Ptr>::Ptr parent = match->second;
    node->SetParent(parent);
    parent->AddChild(node);
  }

  // Does the tree contain this point?
  bool PointTree::Contains(Point::Ptr point) {
    return registry_.count(point) > 0;
  }

  // Get nearest point in the tree.
  Point::Ptr PointTree::GetNearest(Point::Ptr point) {
    Point::Ptr nearest;
    double distance;
    if (!kd_tree.NearestNeighbor(point, nearest, distance))
      VLOG(1) << "Did not find a nearest neighbor.";
    return nearest;
  }

  // Get the path from the head to a particular goal point.
  Trajectory& PointTree::GetTrajectory(Point::Ptr goal) {

    // Return empty path if goal is not in the tree.
    if (!Contains(goal)) {
      VLOG(1) << "Tree does not contain the goal point. Returning "
        "empty trajectory.";
      Trajectory path;
      return path;
    }

    // Trace the tree and populate the path.
    std::list<Point::Ptr> trace;
    const auto match = registry_.find(goal);
    Node<Point::Ptr>::Ptr current_node = match->second;
    while (current_node != nullptr) {
      trace.push_front(current_node->GetData());
      current_node = current_node->GetParent();
    }

    Trajectory path(trace);
    return path;
  }

} //\ namespace path
