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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 *          Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Wrapper around the FLANN library for approximate nearest neighbor searches.
// This is useful for finding the nearest Point in a collection -- e.g. for
// insertion into a RRT.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_FLANN_POINT_2DTREE_H
#define PATH_PLANNING_FLANN_POINT_2DTREE_H

#include <geometry/point_2d.h>
#include <util/types.h>
#include <util/disallow_copy_and_assign.h>

#include <flann/flann.h>

namespace path {

  class FlannPoint2DTree {
  public:
    FlannPoint2DTree() {}
    ~FlannPoint2DTree();

    // Number of points in the tree.
    int Size() const;

    // Add points to the index.
    void AddPoint(Point2D::Ptr point);
    void AddPoints(std::vector<Point2D::Ptr>& points);

    // Queries the kd tree for the nearest neighbor of 'query'. Returns whether or
    // not a nearest neighbor was found, and if it was found, the nearest neighbor
    // and distance to the nearest neighbor.
    bool NearestNeighbor(Point2D::Ptr query, Point2D::Ptr& nearest,
                         float& nn_distance) const;

    // Queries the kd tree for all neighbors of 'query' within the specified radius.
    // Returns whether or not the search exited successfully.
    bool RadiusSearch(Point2D::Ptr query, std::vector<Point2D::Ptr>& neighbors,
                      float radius) const;

  private:
    std::shared_ptr< flann::Index< flann::L2<double> > > index_;
    std::vector<Point2D::Ptr> registry_; // to retrieve original points

    DISALLOW_COPY_AND_ASSIGN(FlannPoint2DTree);

  };  //\class FlannPoint2DTree

}  //\namespace path

#endif
