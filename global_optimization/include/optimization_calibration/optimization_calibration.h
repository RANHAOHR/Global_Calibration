/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *	 Ran Hao <rxh349@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *	 copyright notice, this list of conditions and the following
 *	 disclaimer in the documentation and/or other materials provided
 *	 with the distribution.
 *   * Neither the name of Case Western Reserve University, nor the names of its
 *	 contributors may be used to endorse or promote products derived
 *	 from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPTIMIZATIONCALIBRATION_H
#define OPTIMIZATIONCALIBRATION_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <math.h>

#include <string>
#include <cstring>

#include <tool_model/tool_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Transform.h>
#include <cwru_davinci_interface/davinci_interface.h>

#include <cwru_davinci_interface/davinci_interface.h>
#include <cwru_davinci_kinematics/davinci_kinematics.h>

#include <xform_utils/xform_utils.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>

class OptCalibration {

private:

	ros::NodeHandle node_handle;

/**
 * @brief The ToolModel Object
 */
	ToolModel newToolModel;

/**
 * @brief predicted_real_pose is used to get the predicted real tool pose from using the forward kinematics
 */
    std::vector<ToolModel::toolModel> tool_poses;

/**
 * @brief Left and right rendered images for comparing and testing
 */
    std::vector<cv::Mat> left_raw_images; 
    std::vector<cv::Mat> right_raw_images; 

/**
 * @brief Left and right segemented images
 */
	std::vector<cv::Mat> segmented_left;
	std::vector<cv::Mat> segmented_right;

/**
 * @brief Left and right rendered Image for ARM 1
 */
	cv::Mat toolImage_left_arm_1;
	cv::Mat toolImage_right_arm_1; 

/**
 * The joint angle vectors contains the 7 joint sensor for all poses
 */
    std::vector<std::vector<double> > joint_sensor;

/**
 * The fwd kinematics object
 */
    Davinci_fwd_solver kinematics;

/**
 * The offset bewteen the left and right camera, is used to compute the right one using the left
 */
    cv::Mat g_cr_cl;

/**
 * L is the dimension of the state vector
 */
	int L;

/**
 * Number of pictures from data set
 */
     int nData;

/**
 * Projection matrices
 */
	cv::Mat P_left;
	cv::Mat P_right;

public:

/**
* @brief The default constructor
*/
    OptCalibration(ros::NodeHandle *nodehandle);

/**
 * @brief The deconstructor
 */
	~OptCalibration();

/**
 * @brief get a coarse initialzation using forward kinematics
 */
    void getToolPoses();
/**
 * @brief Main optimization Function
 */
	void optimizationMain();

/**
 * @brief Compute the total error for all poses using one input calibration results
 * @param cam_matrices_left
 * @return a total error computed using the data set and input camera-robot base transformation
 */
    double computeError(cv::Mat & cam_vector_left);

/**
 * @brief get the p(z_t|x_t), compute the matching score based on the camera view image and rendered image
 * @param toolImage_left
 * @param toolImage_right
 * @param toolPose
 * @param segmented_left
 * @param segmented_right
 * @param Cam_left
 * @param Cam_right
 * @return matching score using matching functions: chamfer matching
 */
	double measureFuncSameCam(cv::Mat & toolImage_left, cv::Mat & toolImage_right, ToolModel::toolModel &toolPose,
							  const cv::Mat &segmented_left, const cv::Mat &segmented_right, cv::Mat &Cam_left, cv::Mat &Cam_right);

/**
 * @brief Main optimization function
 * @param g_CB_vec : the seed 6x1D vector of the camera matrix 
 */
	void particleSwarmOptimization(const cv::Mat &g_CB_vec);

/**
 * @brief a boundtry check if necessary
 * @param particle : the particle vector of each camera matrix 
 */
	void boundaryCheck(cv::Mat &particle);

/**
 * @brief Given a state vector, output a SE(3) matrix represent the transformation
 * @param vec_6_1
 * @param outputGeometry : a SE(3) matrix
 */
	void computeSE3(const cv::Mat &vec_6_1, cv::Mat &outputGeometry);

/**
 * @brief The segmentation function using Canny
 */
    cv::Mat segmentation(cv::Mat &InputImg);

/**
 * @brief Convert joint sensor vector to tool_model vector
 */
    void convertJointToPose();

/**
 * @brief Given a Eigen::Affine3d as a transformation matrix, extract the rotation as Rodrigues vector 
 * @param trans : input transformation with Eigen::Affine3d form
 * @param rot_vec : output Rodrigues vector
 */
    void computeRodriguesVec(const Eigen::Affine3d &trans, cv::Mat &rot_vec);
};

#endif
