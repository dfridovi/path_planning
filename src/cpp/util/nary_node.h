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
// This class defines a Node in an N-ary tree of generic objects.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_NARY_NODE_H
#define PATH_PLANNING_NARY_NODE_H

#include <util/disallow_copy_and_assign.h>
#include <memory>
#include <vector>
#include <glog/logging.h>

namespace path {

  // Helper class for use with a tree class.
  template<typename DataType>
  class Node {
  public:
    typedef std::shared_ptr<Node> Ptr;
    typedef std::shared_ptr<const Node> ConstPtr;

    ~Node() {}

    // Factory method.
    Node::Ptr Create(const DataType data);

    // Add a child.
    void AddChild(Node::Ptr child);

    // Set parent.
    void SetParent(Node::Ptr parent);

    // Get children.
    std::vector<Node::Ptr>& GetChildren();

    // Get parent.
    Node::Ptr GetParent();

    // Get data.
    DataType GetData();

  private:
    // Private constructor. Use the factory method instead.
    Node(const DataType data) {}

    DataType data_;
    Node::Ptr parent_;
    std::vector<Node::Ptr> children_;
  };

// ---------------------------- Implementation ------------------------------ //

  template<typename DataType>
  Node<DataType>::Node(DataType data)
    : data_(data) {}

  template<typename DataType>
  Node<DataType>::Ptr Node<DataType>::Create(DataType data) {
    Node<DataType>::Ptr node(new Node<DataType>(data));
    return node;
  }

  template<typename DataType>
  void Node<DataType>::SetParent(Node<DataType>::Ptr parent) {
    parent_ = parent;
  }

  template<typename DataType>
  void Node<DataType>::AddChild(Node<DataType>::Ptr child) {
    children_.push_back(child);
  }

  template<typename DataType>
  Node<DataType>::Ptr Node<DataType>::GetParent() {
    return parent_;
  }

  template<typename DataType>
  std::vector< Node<DataType>::Ptr >& Node<DataType>::GetChildren() {
    return children_;
  }

  template<typename DataType>
  DataType Node<DataType>::GetData() {
    return data_;
  }

} //\ namespace path

#endif
