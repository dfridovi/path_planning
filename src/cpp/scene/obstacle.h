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
// This class is the basis for all Obstacle objects. For example, one type
// of obstacle we care about is point obstacles with some covariance.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef PATH_PLANNING_OBSTACLE_H
#define PATH_PLANNING_OBSTACLE_H

#include <geometry/point.h>
#include <geometry/line_segment.h>
#include <util/disallow_copy_and_assign.h>
#include <memory>
#include <Eigen/Dense>

using Eigen::VectorXd;

namespace path {

  // Derive from this class when defining a new Obstacle type.
  class Obstacle {
  public:
    typedef std::shared_ptr<Obstacle> Ptr;
    typedef std::shared_ptr<const Obstacle> ConstPtr;

    inline Obstacle();
    inline Obstacle(Point::Ptr location, double radius);
    virtual ~Obstacle() {}

    // Get radius.
    virtual inline double GetRadius() const;
    virtual inline Point::Ptr GetLocation() const;

    // Define these methods in a derived class.
    virtual bool IsFeasible(Point::Ptr point) const = 0;
    virtual bool IsFeasible(const VectorXd& point) const = 0;
    virtual double Cost(Point::Ptr point) const = 0;
    virtual double Cost(const VectorXd& point) const = 0;
    virtual Point::Ptr Derivative(Point::Ptr point) const = 0;

  protected:
    Point::Ptr location_;
    double radius_;

  private:
    DISALLOW_COPY_AND_ASSIGN(Obstacle);
  };

// ---------------------------- Implementation ------------------------------ //

  // Default base constructors.
  Obstacle::Obstacle()
    : location_(nullptr), radius_(-1.0) {}
  Obstacle::Obstacle(Point::Ptr location, double radius)
    : radius_(radius), location_(location) {}

  // Getters.
  double Obstacle::GetRadius() const { return radius_; }
  Point::Ptr Obstacle::GetLocation() const { return location_; }

} //\ namespace path

#endif
