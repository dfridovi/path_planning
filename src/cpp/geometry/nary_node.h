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
// This class defines a Node in an N-ary tree of Points.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_NARY_NODE_H
#define PATH_PLANNING_NARY_NODE_H

#include <util/types.h>
#include <util/disallow_copy_and_assign.h>
#include <memory>
#include <vector>
#include <glog/logging.h>

namespace path {

  // Helper class for use with a tree class.
  typename<typename DataType>
  class Node {
  public:
    typedef std::shared_ptr<Node> Ptr;

    ~Node() {}

    // Factory method.
    static Node::Ptr Create(const DataType& data);

    // Add a child.
    void AddChild(Node::Ptr child);

    // Set parent.
    void SetParent(Node::Ptr parent);

    // Get children.
    std::vector<Node::Ptr>& GetChildren();

    // Get parent.
    Node::Ptr GetParent();

    // Get data.
    DataType& GetData();

  private:
    DataType data_;
    Node::Ptr parent_;
    std::vector<Node::Ptr> children_;

    // Private constructor. Use the factory method instead.
    Node(const DataType& data)
      : data_(data) {}

    DISALLOW_COPY_AND_ASSIGN(Node);
  };

} //\ namespace path

#endif
