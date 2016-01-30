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
 * Author: James Smith   ( james.smith@berkeley.edu )
 */

#ifndef PATH_DEPTHMAP_H
#define PATH_DEPTHMAP_H

#include <image/image.h>
#include <camera/camera.h>
#include <pose/pose.h>

namespace path
{

class DepthMap : public Image
{
public:
	using Image::Image;

	DepthMap();
	DepthMap( bool inverted );

	uchar GetValue( size_t u, size_t v ) const;
	Camera CreateCamera( const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation ) const;
	Camera CreateCamera( const double X, const double Y, const double Z, const double Phi, const double Theta, const double Psi ) const;
	Eigen::Vector3d Unproject( size_t u, size_t v ) const;
	bool SaturatedAt( size_t u, size_t v ) const;

	// Setters.
	void SetInverted( bool value );
	void SetCamera( const Camera& c );

	// Getters.
	bool IsInverted() const;
	Camera GetCamera() const;

private:
	Camera camera;
	bool inverted_;
};

}

#endif
