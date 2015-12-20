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
// This class defines a generic N-ary tree of Points.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_POINT_TREE_H
#define PATH_PLANNING_POINT_TREE_H

#include "point.h"
#include <util/disallow_copy_and_assign.h>
#include <memory>
#include <vector>
#include <glog/logging.h>

namespace path {

  // Tree of Points.
  class PointTree {
  public:
    ~Trajectory() {}

    // Constructors.
    Trajectory();
    Trajectory(std::vector<Point::Ptr>& points);

    // Add a point to the path.
    void AddPoint(Point::Ptr point);

    // Get path length.
    double GetLength() const;

  private:
    Node::Ptr head_;
    DISALLOW_COPY_AND_ASSIGN(PointTree);

  };

  class Node {
  public:
    typedef std::shared_ptr<Node> Ptr;
    typedef std::shared_ptr<const Node> ConstPtr;

    ~Node() {}

    // Factory method.
    Create(Node::Ptr parent);

    // Add a child.
    AddChild(Node::Ptr child);

  private:
    // Private constructor. Use the factory method instead.
    Node(Node::Ptr parent) {}

    Node::Ptr parent_;
    std::vector<Node::Ptr> children_;
  };

// ---------------------------- Implementation ------------------------------ //

  Node::Create(Node::Ptr parent) {
    Node::Ptr node(new Node(parent));
    return node;
  }

  Node::Node(Node::Ptr parent)
    : parent_(parent) {}

  Node::AddChild(Node::Ptr child) {
    CHECK_NOTNULL(child.get());
    children_.push_back(child);
  }

} //\ namespace path

#endif
