/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *	Ran Hao <rxh349@case.edu>
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
 *
 */

#include <optimization_calibration/optimization_calibration.h>  //everything inside here

using namespace std;

OptCalibration::OptCalibration(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle), L(6), nData(70)
{
    /********** using calibration results: camera-base transformation *******/
    g_cr_cl = cv::Mat::eye(4, 4, CV_64FC1);

    cv::Mat rot(3, 3, CV_64FC1);
    cv::Mat rot_vec = (cv::Mat_<double>(3, 1) << 0.0001, -0.004, 0.001); //-0.01163, -0.024, 0.00142
    cv::Rodrigues(rot_vec, rot);
    rot.copyTo(g_cr_cl.colRange(0, 3).rowRange(0, 3));

    cv::Mat p = (cv::Mat_<double>(3, 1) << 0.00, 0.0, 0.00); //-0.011, 0.0, 0.00
    p.copyTo(g_cr_cl.colRange(3, 4).rowRange(0, 3));

	toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);

	P_left = cv::Mat::zeros(3,4,CV_64FC1);
	P_right = cv::Mat::zeros(3,4,CV_64FC1);

    P_left = (cv::Mat_<double>(3, 4) << 893.78525, 0, 288.4443, 0,
            0, 893.78525, 259.7727, 0,
            0, 0, 1, 0);

    P_right = (cv::Mat_<double>(3, 4) << 893.78525, 0, 288.4443, 4.73295,
            0, 893.78525, 259.7727, 0,
            0, 0, 1, 0);

    kinematics = Davinci_fwd_solver();

    getToolPoses();

};

OptCalibration::~OptCalibration() {

};

void OptCalibration::optimizationMain(){

    /**
     * some seeds
     */
    /* -0.14643016064445, -0.05682715421036513, 0.018036308562240, 0.9376132688181528, 2.841738662007055, -0.1992759086311982 */
    /* -0.1573825011333455, -0.062592196488725, 0.02830238685851418, 0.92371292898318, 2.816467943665938, -0.2305205298700717 */

    /* a good one -0.1468711888945467, -0.05000594917652904, 0.01870526756691652, 1.006991441692619, 2.758528500676225, -0.1812563712077466 */
    /* a good one -0.1464792194963849, -0.04897846826936547, 0.01796164720820059, 1.006875736072832, 2.757973804101557, -0.1812948125567952 */
    cv::Mat Cam_left_vec = (cv::Mat_<double>(6,1) << -0.142606765370334, -0.04837197008497222, 0.0133186983763136, 1.006896511217604, 2.757040952370038, -0.1818372562586458);

    particleSwarmOptimization(Cam_left_vec);

};

void OptCalibration::getToolPoses(){

    std::string data_pkg = ros::package::getPath("global_optimization");

    left_raw_images.resize(nData);
    right_raw_images.resize(nData);
    char index[16];
    /**
     * read images from left and right image pool
     */
    for (int i = 0; i < nData; ++i) {
        sprintf(index, "%d", i);
        string ipic(index);

        string left_fname = data_pkg + "/left_pics/" + ipic + ".png";
        left_raw_images[i] = cv::imread(left_fname);

        string right_fname = data_pkg + "/right_pics/" + ipic + ".png";
        right_raw_images[i] = cv::imread(right_fname);

    }
    /**
     * get segmented images
     */
    segmented_left.resize(nData);
    segmented_right.resize(nData);
    for (int j = 0; j < nData; ++j) {
        segmented_left[j] = segmentation(left_raw_images[j]);
        segmented_right[j] = segmentation(right_raw_images[j]);
    }
    /**
     * get the corresponding joint sensor poses for all images
     */
    joint_sensor.resize(nData);
    for (int k = 0; k < nData; ++k) {
        joint_sensor[k].resize(7);
    }

    string jsp_path = data_pkg + "/playfiles/81_pts.jsp";
    fstream jspfile(jsp_path.c_str(), std::ios_base::in);
    std::vector<double > temp_sensor;
    double filedata;

    while(jspfile >> filedata){
        temp_sensor.push_back(filedata);
    }

    for (int i = 0; i < nData; ++i) {
        for (int j = 0; j < 7; ++j) {
            int Idx = i* 7 + j;
            joint_sensor[i][j] = temp_sensor[Idx];
        }
    }
    ROS_INFO_STREAM("joint_sensor size is : " << joint_sensor.size());

    convertJointToPose();
};

void OptCalibration::convertJointToPose(){

    tool_poses.resize(nData); //initialize the vector

    int joint_size = joint_sensor.size();
    if( joint_size != nData){
        ROS_ERROR("-Size of the data sets are incorrrect!-");
    }else{
        for (int i = 0; i < joint_sensor.size(); ++i) {
            Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0],
                                                                    joint_sensor[i][0] + DH_q_offset0);
            Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1],
                                                                    joint_sensor[i][1] + DH_q_offset1);
            Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], joint_sensor[i][2] + DH_q_offset2,
                                                                    DH_alpha_params[2], 0.0);
            Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3], DH_d4, DH_alpha_params[3],
                                                                    joint_sensor[i][3] + DH_q_offset3);

            Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3 * a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
            Eigen::Vector3d a1_trans = a1_pos.translation();

            cv::Mat a1_rvec = cv::Mat::zeros(3, 1, CV_64FC1);
            computeRodriguesVec(a1_pos, a1_rvec);

            tool_poses[i].tvec_cyl(0) = a1_trans[0];
            tool_poses[i].tvec_cyl(1) = a1_trans[1];
            tool_poses[i].tvec_cyl(2) = a1_trans[2];
            tool_poses[i].rvec_cyl(0) = a1_rvec.at<double>(0, 0);
            tool_poses[i].rvec_cyl(1) = a1_rvec.at<double>(1, 0);
            tool_poses[i].rvec_cyl(2) = a1_rvec.at<double>(2, 0);

            newToolModel.computeEllipsePose(tool_poses[i], joint_sensor[i][4], joint_sensor[i][5], joint_sensor[i][6]);
        }

    }

}

/** The optimization functions **/
double OptCalibration::computeError(cv::Mat & cam_vector_left)
{
    ros::spinOnce();
	/***Update according to the max score***/
	double matchingerror;
	double totalScore = 0.0; //total score
    /**
     * Compute the right camera matrix using the offset matrix
     */
    cv::Mat cam_matrices_left;
    computeSE3(cam_vector_left, cam_matrices_left);

    cv::Mat cam_matrices_right = g_cr_cl * cam_matrices_left;

	/*** do the sampling and get the matching score ***/
    int pose_size = tool_poses.size();
    for (int i = 0; i < pose_size; ++i) {
        matchingerror = measureFuncSameCam(toolImage_left_arm_1, toolImage_right_arm_1, tool_poses[i], segmented_left[i], segmented_right[i], cam_matrices_left, cam_matrices_right);
        totalScore += matchingerror;

        /** debug */
        cv::Mat test_seg_l = segmented_left[i].clone();
        cv::Mat test_seg_r = segmented_right[i].clone();

        newToolModel.renderTool(test_seg_l, tool_poses[i], cam_matrices_left, P_left);
        newToolModel.renderTool(test_seg_r, tool_poses[i], cam_matrices_right, P_right);

        cv::imshow("debug left ", test_seg_l );
        cv::imshow("debug right ", test_seg_r );

        ROS_INFO_STREAM("matchingerror " << matchingerror);

        cv::waitKey();
    }

    return  totalScore;
};

double OptCalibration::measureFuncSameCam(cv::Mat & toolImage_left, cv::Mat & toolImage_right, ToolModel::toolModel &toolPose,
		const cv::Mat &segmented_left, const cv::Mat &segmented_right, cv::Mat &Cam_left, cv::Mat &Cam_right) {

	toolImage_left.setTo(0);
	toolImage_right.setTo(0);

	/***do the sampling and get the matching score***/
	//first get the rendered image using 3d model of the tool
	newToolModel.renderTool(toolImage_left, toolPose, Cam_left, P_left);
	double left = newToolModel.calculateChamferScore(toolImage_left, segmented_left);  //get the matching score

	newToolModel.renderTool(toolImage_right, toolPose, Cam_right, P_right);
	double right = newToolModel.calculateChamferScore(toolImage_right, segmented_right);

	double matchingScore = sqrt(pow(left, 2) + pow(right, 2));

	return matchingScore;
};

void OptCalibration::particleSwarmOptimization(const cv::Mat &g_CB_vec) {

    int Num = 600;  //number of particles 40000
    double c1 = 2; //flying weights according to the local best
    double c2 = 2; //flying weights according to the global best
    int MaxIter = 10;  //max iteration
    double w = 0.75;  //speed weight

    //initialization
    cv::Mat vec_CB(L, 1, CV_64FC1);  ///a new vec for representing G_CB
    vec_CB = g_CB_vec.clone();
    ROS_INFO_STREAM("vec_CB " << vec_CB);

    std::vector<double> local_errorG_CB;

    std::vector<cv::Mat> particles;
    std::vector<cv::Mat> local_best_paticles;
    std::vector<cv::Mat> velocities;

    particles.resize(Num);
    local_best_paticles.resize(Num);
    velocities.resize(Num);
    local_errorG_CB.resize(Num);

    cv::Mat temp_vel(L, 1, CV_64FC1);  ///a new vec for representing G_CB
    cv::Mat temp_particle(L, 1, CV_64FC1);  ///a new vec for representing G_CB

    double temp_errorValue;

    cv::Mat final_G_CB = cv::Mat::eye(4,4,CV_64FC1); ///final G_CT1

    //initialization for all particles
    for (int i = 0; i < Num; i++) {

        //give random velocity
        if(i % 500 == 0){
            ROS_INFO_STREAM(" i = " << i);
        }

        double dev_x = newToolModel.randomNumber(0.001, 0.0);
        double dev_y = newToolModel.randomNumber(0.001, 0.0);
        double dev_z = newToolModel.randomNumber(0.0001, 0.0);
        double dev_roll = newToolModel.randomNumber(0.0002, 0);
        double dev_pitch = newToolModel.randomNumber(0.0002, 0);
        double dev_yaw = newToolModel.randomNumber(0.0002, 0);

         dev_x = 0.0;
         dev_y = 0.0;
         dev_z = 0.0;
         dev_roll = 0.0; //
         dev_pitch = 0.0; //
         dev_yaw = 0.0; //

        ///random velocity
        temp_vel.at<double>(0, 0) = dev_x;
        temp_vel.at<double>(1, 0) = dev_y;
        temp_vel.at<double>(2, 0) = dev_z;
        temp_vel.at<double>(3, 0) = dev_roll;
        temp_vel.at<double>(4, 0) = dev_pitch;
        temp_vel.at<double>(5, 0) = dev_yaw;

        //random particles
        temp_particle.at<double>(0, 0) = vec_CB.at<double>(0,0) + dev_x;
        temp_particle.at<double>(1, 0) = vec_CB.at<double>(1,0) + dev_y;
        temp_particle.at<double>(2, 0) = vec_CB.at<double>(2,0) + dev_z;
        temp_particle.at<double>(3, 0) = vec_CB.at<double>(3,0) + dev_roll;
        temp_particle.at<double>(4, 0) = vec_CB.at<double>(4,0) + dev_pitch;
        temp_particle.at<double>(5, 0) = vec_CB.at<double>(5,0) + dev_yaw;

        ////Opencv is stupid so don't use push_back unless you like to debug for really long time
        velocities[i] = temp_vel.clone();
        temp_errorValue = computeError(temp_particle);
        local_errorG_CB[i] = temp_errorValue; //temporary local best
        particles[i] = temp_particle.clone();
        local_best_paticles[i] = temp_particle.clone();
    }
    ROS_WARN(" -FINISHED Initialization- ");
    ///initialize global best
    cv::Mat global_best(L, 1, CV_64FC1);
    global_best = particles[0].clone();

    double best_value = computeError(global_best);
    for (int i = 1; i < Num; i++) {

        if (local_errorG_CB[i] < best_value){
            global_best = particles[i].clone();
            best_value = computeError(global_best);
        }
    }

    ////main iteration
    ROS_WARN(" -START ITERATION- ");
    for (int iter = 0; iter < MaxIter; iter++) {

        double dev_x = newToolModel.randomNumber(0.0004, 0.0);
        double dev_y = newToolModel.randomNumber(0.0004, 0.0);
        double dev_z = newToolModel.randomNumber(0.00001, 0.0);
        double dev_roll = newToolModel.randomNumber(0.0001, 0.0); //
        double dev_pitch = newToolModel.randomNumber(0.0001, 0.0); //
        double dev_yaw = newToolModel.randomNumber(0.0001, 0.0); //

        for (int n = 0; n < Num; n++) {

            if(n % 200 == 0){
                ROS_INFO_STREAM(" n = " << n);
                ROS_INFO_STREAM("global best vec: " << global_best);
                computeSE3(global_best, final_G_CB);
                ROS_INFO_STREAM(" final_G_CB = " << final_G_CB);

            }

            //update have to use different metric
            velocities[n].at<double>(0,0) = w * velocities[n].at<double>(0,0) + c1 * dev_x *(local_best_paticles[n].at<double>(0,0) - particles[n].at<double>(0,0)) + c2 * dev_x * (global_best.at<double>(0,0) - particles[n].at<double>(0,0));
            velocities[n].at<double>(1,0) = w * velocities[n].at<double>(1,0) + c1 * dev_y *(local_best_paticles[n].at<double>(1,0) - particles[n].at<double>(1,0)) + c2 * dev_y * (global_best.at<double>(1,0) - particles[n].at<double>(1,0));
            velocities[n].at<double>(2,0) = w * velocities[n].at<double>(2,0) + c1 * dev_z *(local_best_paticles[n].at<double>(2,0) - particles[n].at<double>(2,0)) + c2 * dev_z * (global_best.at<double>(2,0) - particles[n].at<double>(2,0));
            velocities[n].at<double>(3,0) = w * velocities[n].at<double>(3,0) + c1 * dev_roll *(local_best_paticles[n].at<double>(3,0) - particles[n].at<double>(3,0)) + c2 * dev_roll * (global_best.at<double>(3,0) - particles[n].at<double>(3,0));
            velocities[n].at<double>(4,0) = w * velocities[n].at<double>(4,0) + c1 * dev_pitch *(local_best_paticles[n].at<double>(4,0) - particles[n].at<double>(4,0)) + c2 * dev_pitch * (global_best.at<double>(4,0) - particles[n].at<double>(4,0));
            velocities[n].at<double>(5,0) = w * velocities[n].at<double>(5,0) + c1 * dev_yaw *(local_best_paticles[n].at<double>(5,0) - particles[n].at<double>(5,0)) + c2 * dev_yaw * (global_best.at<double>(5,0) - particles[n].at<double>(5,0));

            particles[n] = particles[n] + velocities[n];

            ///if need to do boundary check
            //boundaryCheck(particles[n]);

            temp_errorValue = computeError(particles[n]);  // the temp error for n-th new G_CM0

            if (local_errorG_CB[n] > temp_errorValue) { /// update local best if new error is smaller

                local_errorG_CB[n] = temp_errorValue;
                local_best_paticles[n] = particles[n].clone();
            }

            best_value = computeError(global_best);
            if (best_value > local_errorG_CB[n]) {  ///if local error is smaller than global best then update
                global_best = local_best_paticles[n].clone();
                ROS_INFO("update global best");
            }
        }

        best_value = computeError(global_best);
        ROS_INFO_STREAM("minimal error : " << best_value);
        computeSE3(global_best, final_G_CB);
        ROS_INFO_STREAM("global best G_CB: " << final_G_CB);
        ROS_INFO_STREAM("global best vec: " << global_best);
        ROS_INFO("------------------------");
    }

};

void OptCalibration::boundaryCheck(cv::Mat &particle){

    ///need to do boundary check
    double temp_x = particle.at<double>(0,0);
    double temp_y = particle.at<double>(1,0);
    double temp_z = particle.at<double>(2,0);

    double temp_roll = particle.at<double>(3,0);
    double temp_pitch = particle.at<double>(4,0);
    double temp_yaw = particle.at<double>(5,0);

    /********numbers should be close to the real constrains*******/
    if (temp_x > -0.012 || temp_x < -0.015) {
        particle.at<double>(0,0) = newToolModel.randomNum(-0.015, -0.012);
    }
    if (temp_y > -0.04 || temp_y < -0.06) {
        particle.at<double>(1,0) = newToolModel.randomNum(-0.06, -0.04);
    }
    if (temp_z > 0.02 || temp_z < 0.017) {
        particle.at<double>(2,0) = newToolModel.randomNum(0.017, 0.02);
    }
    if (temp_roll > 1.19 || temp_roll < 0.9) {
        particle.at<double>(3,0) = newToolModel.randomNum(0.9, 1.19);
    }
    if (temp_pitch > 2.8 || temp_pitch < 2.5) {
        particle.at<double>(4,0) = newToolModel.randomNum(2.5, 2.8);
    }
    if (temp_yaw > -0.19 || temp_yaw < -0.21) {
        particle.at<double>(5,0) = newToolModel.randomNum(-0.21, -0.19);
    }

};

void OptCalibration::computeSE3(const cv::Mat &vec_6_1, cv::Mat &outputGeometry){

    outputGeometry = cv::Mat::eye(4,4,CV_64FC1);
    cv::Mat Rotation_CB(3,3,CV_64FC1);
    cv::Mat Rvec_CB(3,1,CV_64FC1);
    cv::Mat Tvec_CB(3,1,CV_64FC1);

    Tvec_CB.at<double>(0,0) = vec_6_1.at<double>(0,0);
    Tvec_CB.at<double>(1,0) = vec_6_1.at<double>(1,0);
    Tvec_CB.at<double>(2,0) = vec_6_1.at<double>(2,0);

    Rvec_CB.at<double>(0,0) = vec_6_1.at<double>(3,0);
    Rvec_CB.at<double>(1,0) = vec_6_1.at<double>(4,0);
    Rvec_CB.at<double>(2,0) = vec_6_1.at<double>(5,0);

    cv::Rodrigues(Rvec_CB, Rotation_CB );  //get rotation mat

    Rotation_CB.copyTo(outputGeometry.colRange(0,3).rowRange(0,3));
    Tvec_CB.copyTo(outputGeometry.colRange(3,4).rowRange(0,3));

};

cv::Mat OptCalibration::segmentation(cv::Mat &InputImg) {

    cv::Mat src, src_gray;
    cv::Mat grad;

    cv::Mat res;
    src = InputImg;
    cv::resize(src, src, cv::Size(), 1, 1);

    double lowThresh = 24;
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    cv::blur(src_gray, src_gray, cv::Size(3, 3));
    cv::Canny(src_gray, grad, lowThresh, 4 * lowThresh, 3); //use Canny segmentation

    grad.convertTo(res, CV_32FC1);

    return res;

};


void OptCalibration::computeRodriguesVec(const Eigen::Affine3d &trans, cv::Mat &rot_vec) {
    Eigen::Matrix3d rot_affine = trans.rotation();

    cv::Mat rot(3, 3, CV_64FC1);
    rot.at<double>(0, 0) = rot_affine(0, 0);
    rot.at<double>(0, 1) = rot_affine(0, 1);
    rot.at<double>(0, 2) = rot_affine(0, 2);
    rot.at<double>(1, 0) = rot_affine(1, 0);
    rot.at<double>(1, 1) = rot_affine(1, 1);
    rot.at<double>(1, 2) = rot_affine(1, 2);
    rot.at<double>(2, 0) = rot_affine(2, 0);
    rot.at<double>(2, 1) = rot_affine(2, 1);
    rot.at<double>(2, 2) = rot_affine(2, 2);

    rot_vec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Rodrigues(rot, rot_vec);
    //ROS_INFO_STREAM("rot_vec " << rot_vec);
};