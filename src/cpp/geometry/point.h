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
// This class is the basis for all Point objects. For example a Point3D class
// could be derived from this class, where all Points live in R^3.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_POINT_H
#define PATH_PLANNING_POINT_H

#include <Eigen/Dense>
#include <memory>
#include <util/disallow_copy_and_assign.h>
#include <glog/logging.h>

using Eigen::VectorXd;

namespace path {

  // Derive from this class when defining a new Point type.
  class Point {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<Point> Ptr;
    typedef std::shared_ptr<const Point> ConstPtr;

    // Each Point can only be of a single type.
    enum PointType {POINT_2D, OTHER};

    // Derived classes must write their own constructors.
    Point();
    virtual ~Point() {}

    // Set point type. This should be called in the derived class' constructor.
    virtual inline void SetType(const PointType type);

    // Check point type.
    virtual inline PointType GetType();
    virtual inline bool IsType(PointType type);
    virtual inline bool IsSameTypeAs(Point::Ptr point);

    // Get vector.
    virtual inline VectorXd& GetVector();

    // Define these methods in a derived class.
    virtual double DistanceTo(Point::Ptr point) const = 0;

  protected:
    VectorXd coordinates_;

  private:
    PointType type_;
    DISALLOW_COPY_AND_ASSIGN(Point);
  };

// ---------------------------- Implementation ------------------------------ //

  // Don't use this!
  Point::Point()
    : type_(Point::PointType::OTHER) {}

  Point::PointType Point::GetType() { return type_; }
  void Point::SetType(const Point::PointType type) { type_ = type; }

  bool Point::IsType(Point::PointType type) {
    if (type_ == type)
      return true;
    return false;
  }

  bool Point::IsSameTypeAs(Point::Ptr point) {
    CHECK_NOTNULL(point.get());

    if (type_ == point->type_)
      return true;
    return false;
  }

  VectorXd& Point::GetVector() {
    return coordinates_;
  }

} //\ namespace path

#endif

