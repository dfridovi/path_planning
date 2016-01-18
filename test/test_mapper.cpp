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

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <mapper/mapper.h>
#include <math/random_generator.h>

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>

#include <file/csv_writer.h>
#include <strings/join_filepath.h>

namespace path
{
	using Eigen::Matrix3d;
	using Eigen::Vector3d;

	// Check if various constructors work
	TEST( Mapper, TestMapperConstruction )
	{
		CHECK_NOTNULL( new Mapper() );

		cv::Mat M = cv::Mat( 10, 10, CV_8UC1 );
		DepthMap dm(M);
		Camera c = dm.GetCameraFromDepthMap(0, 0, 0, 0, 0, 0);
		CHECK_NOTNULL( new Mapper(c) );
	}

	// Generate a depth map and verify that it reprojects to the correct point
	TEST( Mapper, TestMapperGenerated )
	{
		cv::Mat M = cv::Mat( 10, 10, CV_8UC1 );

		int rows = M.rows;
		int cols = M.cols * M.channels();
		for( int x = 0; x < rows; ++x )
		{
			uchar* row = M.ptr<uchar>(x);
			for( int y = 0; y < cols; ++y )
			{
				row[y] = (int)((y / (cols - 1.0f)) * 255);
			}
		}
	
		DepthMap dm(M);
		dm.SetInverted( true );
		Camera c = dm.GetCameraFromDepthMap( 0, -100, 10, 0, 0, 0 );
    	Mapper m( c );

    	PointList pl = m.ProjectDepthMap( dm );
		// TODO Check projection against source
	}

	// Load a depth map and generate a pointcloud file to verify correctness
	// TODO: Automatically verify correctness (save the a correct point cloud file and compare them)
	TEST( Mapper, TestMapperLoaded )
	{
		const std::string depth_map_file = strings::JoinFilepath( PATH_TEST_DATA_DIR, "depth_scene.png" );
		const std::string point_cloud_file = strings::JoinFilepath( PATH_TEST_DATA_DIR, "mapper_point_cloud.csv" );

		cv::Mat M = cv::imread(depth_map_file.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		EXPECT_TRUE(M.data);

		DepthMap dm(M);
		dm.SetInverted( true );
    	Camera c = dm.GetCameraFromDepthMap( 0, -100, 10, 0, 0, 0 );
    	Mapper m( c );
    	PointList pl = m.ProjectDepthMap( dm );

		std::remove(point_cloud_file.c_str());
		
		file::CsvWriter csv_writer(point_cloud_file);
		EXPECT_TRUE( csv_writer.IsOpen() );
		
		for( size_t i = 0; i < pl.size(); ++i )
		{
			Eigen::Vector3d v = pl.at(i);
			csv_writer.WriteLine(v);
		}
		csv_writer.Close();
	}
} //\ namespace path
