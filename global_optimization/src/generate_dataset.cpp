//
// Created by ranhao on 7/15/17.
//

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <optimization_calibration/optimization_calibration.h>


bool freshImage;

using namespace std;
using namespace cv_projective;



int main(int argc, char **argv) {

    ros::init(argc, argv, "generate_datasets");

    ros::NodeHandle nodeHandle;

/*    cv::Mat test_image = cv::Mat::zeros(480, 640, CV_8UC3);
    std::string data_pkg = ros::package::getPath("global_optimization");

    char index[16];

    int a = 1;
    sprintf(index, "%d", a);
    string ipic(index);

    string filename = data_pkg + "/left_pics/" + ipic + ".png";
    test_image = cv::imread(filename);

    cv::imshow("test_image: ", test_image);
    cv::waitKey(10);

    std::vector<std::vector<double> > joint_sensor;

    joint_sensor.resize(3);
    for (int k = 0; k < 3; ++k) {
        joint_sensor[k].resize(7);
    }

    string jsp_path = data_pkg + "/touchy.jsp";
    fstream jspfile(jsp_path.c_str(), std::ios_base::in);
    std::vector<double > temp_sensor;
    double temp;
    while(jspfile >> temp){
        temp_sensor.push_back(temp);
        printf("%f ", temp);
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 7; ++j) {
            int index = i* 7 + j;
            joint_sensor[i][j] = temp_sensor[index];

        }

    }*/


}
