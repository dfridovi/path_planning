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
// This class defines a Node in an N-ary tree of Point2D::Ptr types.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_NODE_2D_H
#define PATH_PLANNING_NODE_2D_H

#include <geometry/point_2d.h>
#include <util/types.h>
#include <util/disallow_copy_and_assign.h>

#include <memory>
#include <vector>
#include <glog/logging.h>

namespace path {

  // Helper class for use with a tree class.
  class Node2D {
  public:
    typedef std::shared_ptr<Node2D> Ptr;

    ~Node2D() {}

    // Factory method.
    static Node2D::Ptr Create(const Point2D::Ptr data);

    // Add a child.
    void AddChild(Node2D::Ptr child);

    // Set parent.
    void SetParent(Node2D::Ptr parent);

    // Get children.
    std::vector<Node2D::Ptr>& GetChildren();

    // Get parent.
    Node2D::Ptr GetParent();

    // Get data.
    Point2D::Ptr GetData();

  private:
    Point2D::Ptr data_;
    Node2D::Ptr parent_;
    std::vector<Node2D::Ptr> children_;

    // Private constructor. Use the factory method instead.
    Node2D(const Point2D::Ptr data)
      : data_(data) {}

    DISALLOW_COPY_AND_ASSIGN(Node2D);
  };

} //\ namespace path

#endif
