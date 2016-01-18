/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *	  1. Redistributions of source code must retain the above copyright
 *		 notice, this list of conditions and the following disclaimer.
 *
 *	  2. Redistributions in binary form must reproduce the above
 *		 copyright notice, this list of conditions and the following
 *		 disclaimer in the documentation and/or other materials provided
 *		 with the distribution.
 *
 *	  3. Neither the name of the copyright holder nor the names of its
 *		 contributors may be used to endorse or promote products derived
 *		 from this software without specific prior written permission.
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
 * Author: James Smith	 ( james.smith@berkeley.edu )
 */

#include <mapper/depthmap.h>
#include <geometry/rotation.h>

namespace path
{

DepthMap::DepthMap()
	: inverted_( false )
{
}

DepthMap::DepthMap( bool inverted )
    : inverted_( inverted )
{
}

void DepthMap::SetInverted( bool value )
{
    inverted_ = value;
}

bool DepthMap::IsInverted() const
{
    return inverted_;
}

uchar DepthMap::GetValue( size_t u, size_t v ) const
{
    uchar value = at<uchar>( u, v );
    if( IsInverted() )
    {
        value = 255 - value;
    }
    return value;
}

Camera DepthMap::GetCameraFromDepthMap( const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation ) const
{
    Pose cameraPose( rotation, position );
    CameraExtrinsics extrinsics( cameraPose );

    // JDS: Not sure how to properly calculate focal length of a point camera
    float focal_length = 120 * 0.35/0.36;
    Matrix3d A;
    A << focal_length, 0, Width()/2, 0, focal_length, Height()/2, 0, 0, 1;

    CameraIntrinsics intrinsics( A, Width(), Height() );

    Camera c( extrinsics, intrinsics );

    return c;
}

Camera DepthMap::GetCameraFromDepthMap( const double X, const double Y, const double Z, const double Phi, const double Theta, const double Psi )
{
    Eigen::Vector3d position( X, Y, Z );
    Matrix3d rotation = EulerAnglesToMatrix( Phi, Theta, Psi );
    return GetCameraFromDepthMap( position, rotation );
}

}
