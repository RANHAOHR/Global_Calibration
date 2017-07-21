#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <optimization_calibration/optimization_calibration.h>

using namespace std;
using namespace cv_projective;

int main(int argc, char **argv) {

	ros::init(argc, argv, "tracking_node");

	ros::NodeHandle nh;
	/******  initialization  ******/
	OptCalibration OptCalibration(&nh);

    ROS_INFO("---- done subscribe -----");

    ros::Duration(2).sleep();

	while (nh.ok()) {
		ros::spinOnce();
		/*** make sure camera information is ready ***/

        OptCalibration.optimizationMain(); //with rendered tool and segmented img

	}

}
