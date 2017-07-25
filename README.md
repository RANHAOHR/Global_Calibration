# Global_Calibration

This package is aimed for performing a fine calibration using Virtual Tool Rendering, and PSO algorithm.
Using the global optimization algorithm to find a Camera-robot base transformation (g_CB) that better fits the whole state space 

#Dependences:
cwru_vision: https://github.com/cwru-robotics/cwru_vision.git

glm library: sudo apt-get install libglm-dev

vision_opencv: https://github.com/ros-perception/vision_opencv.git

cwru_davinci: https://github.com/cwru-robotics/cwru_davinci 

- tool model package: load the surgical tool model, construct tool geometry and acheive virtual rendering.

To check the tool geometry and virtual rendering performance, run:

`rosrun tool_model_optimization tool_model_main`

- Optimization package: 

Before Run the optimization, get the data sets:

- To get the pictures, run:

`roslaunch global_optimization generate_datasets.launch`

which generates the pictures for the left and right cameras and the "check_video.cpp" aims for making sure getting the right images


To run the optimization algorithm:

`rosrun global_optimization optimization_calibration_main`









