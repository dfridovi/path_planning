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

#include <mapper/mapper.h>
#include <iostream>

namespace path
{

Mapper::Mapper()
	: cullSaturated_(true)
{
}

Mapper::Mapper( Camera& c, bool cullSaturated )
	: camera( c ), cullSaturated_(cullSaturated)
{
}

pcl::PointCloud<pcl::PointXYZ> Mapper::ProjectDepthMap( const DepthMap& map )
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = map.Width();
	cloud.height = map.Height();
	cloud.is_dense = false;
	// TODO: Include sensor orientation information?

	cloud.points.resize( cloud.width * cloud.height );

	double cameraX = 0.0;
	double cameraY = 0.0;
	double cameraZ = 0.0;

	double worldX = 0.0;
	double worldY = 0.0;
	double worldZ = 0.0;

	int ii = 0;
	for( size_t u = 0; u < map.Height(); ++u )
	{
		for( size_t v = 0; v < map.Width(); ++v )
		{
			if( !cullSaturated_ || (map.GetValue( u, v * 3 ) > 0 && map.GetValue( u, v * 3 ) < 255) )
			{
				camera.ImageToDirection( u, v, &cameraX, &cameraY );
				cameraZ = map.GetValue( u, v * 3 ) / 256.0;
				camera.CameraToWorld( cameraX, cameraY, cameraZ, &worldX, &worldY, &worldZ );

				cloud.points[ii].x = worldX;
				cloud.points[ii].y = worldY;
				cloud.points[ii].z = worldZ;
				ii++;
			}
		}
	}
	return cloud;
}

void Mapper::SetCamera( const Camera& c )
{
	camera = c;
}

Camera Mapper::GetCamera() const
{
	return camera;
}

}
