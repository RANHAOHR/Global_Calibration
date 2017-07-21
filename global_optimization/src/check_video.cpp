//
// Created by ranhao on 7/19/17.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>

bool freshImage;

using namespace std;
using namespace cv_projective;

//get image size from camera model, or initialize segmented images
cv::Mat rawImage_left = cv::Mat::zeros(480, 640, CV_8UC3);//CV_32FC1
cv::Mat rawImage_right = cv::Mat::zeros(480, 640, CV_8UC3);

void newImageCallback(const sensor_msgs::ImageConstPtr &msg, cv::Mat *outputImage) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);
        outputImage[0] = cv_ptr->image;
        freshImage = true;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "check_video");

    ros::NodeHandle nodeHandle;

    freshImage = false;

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));

    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    ROS_INFO("---- done subscribe -----");

/*    cv::Mat Cam_test = (cv::Mat_<double>(4, 4) << -0.6413935823375527, 0.6453383793439481, 0.4149128205803325, -0.1906,
    0.5337893571955913, 0.7638139772504854, -0.3628458767872552, -0.0718,
    -0.551074581777202, -0.0112509689592025, -0.8343801417918338, 0.01832,
    0, 0, 0, 1);

    cv::Mat rot(3, 3, CV_64FC1);
    rot = Cam_test.colRange(0,3).rowRange(0,3);
    cv::Mat rot_vec(3,1,CV_64FC1); //-0.01163, -0.024, 0.00142 //-0.01, -0.008, 0.024
    cv::Rodrigues(rot, rot_vec);
    ROS_INFO_STREAM("rot_vec" << rot_vec);*/


    cv::Mat cam_left = cv::Mat::eye(4,4,CV_64FC1);
    cv::Mat p = (cv::Mat_<double>(3, 1) << -0.12033, -0.04072, 0.01142);

    cv::Mat rot(3, 3, CV_64FC1);
    cv:: Mat rot_vec = (cv::Mat_<double>(3,1) << 1.02296677, 2.6807519, -0.110696); //1.0996677, 2.6502519, -0.20696
    cv::Rodrigues(rot_vec, rot);
    rot.copyTo(cam_left.colRange(0,3).rowRange(0,3));
    p.copyTo(cam_left.colRange(3,4).rowRange(0,3));

    ROS_INFO_STREAM("cam_left: " << cam_left);

//    while (nodeHandle.ok()) {
//        ros::spinOnce();
//
//        if (freshImage) {
//            cv::imshow("raw image left: ", rawImage_left);
//            cv::imshow("raw image right: ", rawImage_right);
//            cv::waitKey(10);
//        }
//        freshImage = false;
//
//    }

}

