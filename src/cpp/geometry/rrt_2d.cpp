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

#include <geometry/rrt_2d.h>
#include <list>
#include <iostream>
#include <glog/logging.h>

namespace path {

  // Insert a point. Returns true if successful.
  bool RRT2D::Insert(Point2D::Ptr point) {
    CHECK_NOTNULL(point.get());

    // Base case -- empty tree.
    if (registry_.size() == 0) {
      head_ = Node2D::Create(point);
      registry_.emplace(point, head_);
      kd_tree_.AddPoint(point);
      return true;
    }

    // Don't insert if the tree already contains this point.
    if (Contains(point)) return false;

    // Find nearest point.
    Point2D::Ptr nearest = GetNearest(point);

    // Make a new Node2D and insert.
    Node2D::Ptr node = Node2D::Create(point);
    Node2D::Ptr parent = registry_.at(nearest);
    node->SetParent(parent);
    parent->AddChild(node);

    // Update the registry and add to the kdtree.
    registry_.emplace(point, node);
    kd_tree_.AddPoint(point);

    return true;
  }

  // Insert a point at a specified node. Returns true if successful.
  bool RRT2D::Insert(Point2D::Ptr point, Point2D::Ptr parent) {
    CHECK_NOTNULL(point.get());
    CHECK_NOTNULL(parent.get());

    // Ensure parent exists.
    if (!Contains(parent)) {
      VLOG(1) << "Specified parent does not exist. Did not insert.";
      return false;
    }

    // Don't insert if the tree already contains this point.
    if (Contains(point)) return false;

    // Make a new Node2D and insert.
    Node2D::Ptr node = Node2D::Create(point);
    Node2D::Ptr parent_node = registry_.at(parent);
    node->SetParent(parent_node);
    parent_node->AddChild(node);

    // Update the registry and add to the kdtree.
    registry_.emplace(point, node);
    kd_tree_.AddPoint(point);

    return true;
  }

  // Does the tree contain this point?
  bool RRT2D::Contains(Point2D::Ptr point) const {
    CHECK_NOTNULL(point.get());
    return registry_.count(point) > 0;
  }

  // Tree size.
  int RRT2D::Size() const {
    return static_cast<int>(kd_tree_.Size());
  }

  // Get nearest point in the tree.
  Point2D::Ptr RRT2D::GetNearest(Point2D::Ptr point) {
    CHECK_NOTNULL(point.get());

    Point2D::Ptr nearest;
    float distance;
    if (!kd_tree_.NearestNeighbor(point, nearest, distance))
      VLOG(1) << "Did not find a nearest neighbor.";
    return nearest;
  }

  // Get the path from the head to a particular goal point.
  Trajectory2D::Ptr RRT2D::GetTrajectory(Point2D::Ptr goal) {

    // Return empty path if goal is not in the tree.
    if (!Contains(goal)) {
      VLOG(1) << "Tree does not contain the goal point. Returning "
              << "nullptr.";
      return Trajectory2D::Ptr(nullptr);
    }

    // Trace the tree and populate the path.
    std::list<Point2D::Ptr> trace;
    Node2D::Ptr current_node = registry_.at(goal);
    while (current_node) {
      trace.push_front(current_node->GetData());
      current_node = current_node->GetParent();
    }

    Trajectory2D::Ptr path = Trajectory2D::Create(trace);
    return path;
  }

} //\ namespace path
