//
// Created by ranhao on 7/19/17.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>

#include <iostream>
#include <fstream>

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
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_rect", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));

    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_rect", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    ROS_INFO("---- done subscribe -----");


     while (nodeHandle.ok()) {
         ros::spinOnce();

         if (freshImage) {
             cv::imshow("raw image left: ", rawImage_left);
             cv::imshow("raw image right: ", rawImage_right);
             cv::waitKey(10);
         }
         freshImage = false;
     }

}



