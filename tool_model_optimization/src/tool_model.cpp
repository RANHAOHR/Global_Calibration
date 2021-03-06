/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *    Ran Hao <rxh349@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include <ros/ros.h>
#include <boost/random.hpp>

#include <tool_model/tool_model.h>


using cv_projective::reprojectPoint;
using cv_projective::transformPoints;
using namespace std;

boost::mt19937 rng((const uint32_t &) time(0));

ToolModel::ToolModel() {

    ///adjust the model params according to the tool geometry

//    offset_body = 0.4560; //0.4560
//    offset_ellipse = offset_body;
//    offset_gripper = offset_ellipse+ 0.007;

    offset_body = 0.4608; //0.4560
    offset_ellipse = offset_body -0.0067;
    offset_gripper = offset_body -0.006;

    /****initialize the vertices fo different part of tools****/
    tool_model_pkg = ros::package::getPath("tool_model_optimization");

    std::string cylinder = tool_model_pkg + "/tool_parts/cyliner_tense_end_face.obj"; //"/tense_cylinde_2.obj", test_cylinder_3, cyliner_tense_end_face, refine_cylinder_3
    std::string ellipse = tool_model_pkg + "/tool_parts/refine_ellipse_3.obj";
    std::string gripper1 = tool_model_pkg + "/tool_parts/gripper2_1.obj";
    std::string gripper2 = tool_model_pkg + "/tool_parts/gripper2_2.obj";

    load_model_vertices(cylinder.c_str(),
                        body_vertices, body_Vnormal, body_faces, body_neighbors);
    load_model_vertices(ellipse.c_str(),
                        ellipse_vertices, ellipse_Vnormal, ellipse_faces, ellipse_neighbors);
    load_model_vertices(gripper1.c_str(), griper1_vertices,
                        griper1_Vnormal, griper1_faces, griper1_neighbors);
    load_model_vertices(gripper2.c_str(), griper2_vertices,
                        griper2_Vnormal, griper2_faces, griper2_neighbors);

    modify_model_(body_vertices, body_Vnormal, body_Vpts, body_Npts, offset_body, body_Vmat, body_Nmat);
    modify_model_(ellipse_vertices, ellipse_Vnormal, ellipse_Vpts, ellipse_Npts, offset_ellipse, ellipse_Vmat,
                  ellipse_Nmat);
    modify_model_(griper1_vertices, griper1_Vnormal, griper1_Vpts, griper1_Npts, offset_gripper, gripper1_Vmat,
                  gripper1_Nmat);
    modify_model_(griper2_vertices, griper2_Vnormal, griper2_Vpts, griper2_Npts, offset_gripper, gripper2_Vmat,
                  gripper2_Nmat);

    getFaceInfo(body_faces, body_Vpts, body_Npts, bodyFace_normal, bodyFace_centroid);
    getFaceInfo(ellipse_faces, ellipse_Vpts, ellipse_Npts, ellipseFace_normal, ellipseFace_centroid);
    getFaceInfo(griper1_faces, griper1_Vpts, griper1_Npts, gripper1Face_normal, gripper1Face_centroid);
    getFaceInfo(griper2_faces, griper2_Vpts, griper2_Npts, gripper2Face_normal, gripper2Face_centroid);

    /* prepare to get the oval normals for UKF */
    std::string oval_normal = tool_model_pkg + "/tool_parts/new_less_normal.obj";  //contains only the faces with useful normals
    load_model_vertices(oval_normal.c_str(),
                        oval_normal_vertices, oval_normal_Vnormal, oval_normal_faces, oval_normal_neighbors);
    modify_model_(oval_normal_vertices, oval_normal_Vnormal, oval_normal_Vpts, oval_normal_Npts, offset_ellipse, oval_normal_Vmat, oval_normal_Nmat);
    getFaceInfo(oval_normal_faces, oval_normal_Vpts, oval_normal_Npts, oval_normalFace_normal, oval_normalFace_centroid);

    srand((unsigned) time(NULL)); //for the random number generator, use only once
};

double ToolModel::randomNumber(double stdev, double mean) {

    boost::normal_distribution<> nd(mean, stdev);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > var_nor(rng, nd);
    double d = var_nor();

    return d;

};


double ToolModel::randomNum(double min, double max){

    /// srand((unsigned) time( NULL));  //do this in main or constructor
    int N = 999;

    double randN = rand() % (N + 1) / (double) (N + 1);  // a rand number frm 0 to 1
    double res = randN * (max - min) + min;

    return res;
};

void ToolModel::ConvertInchtoMeters(std::vector<cv::Point3d> &input_vertices) {

    int size = (int) input_vertices.size();
    for (int i = 0; i < size; ++i) {
        input_vertices[i].x = input_vertices[i].x * 0.0254;
        input_vertices[i].y = input_vertices[i].y * 0.0254;
        input_vertices[i].z = input_vertices[i].z * 0.0254;
    }
};

void ToolModel::load_model_vertices(const char *path, std::vector<glm::vec3> &out_vertices,
                                    std::vector<glm::vec3> &vertex_normal,
                                    std::vector<std::vector<int> > &out_faces,
                                    std::vector<std::vector<int> > &neighbor_faces) {

    std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
    std::vector<glm::vec2> temp_uvs;

    std::vector<int> temp_face;
    temp_face.resize(6);  //need three vertex and corresponding normals

    FILE *file = fopen(path, "r");
    if (file == NULL) {
        printf("Impossible to open the file ! Are you in the right path ?\n");
        return;
    }

    while (1) {

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader
        if (strcmp(lineHeader, "v") == 0) {
            glm::vec3 vertex;

            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
            //temp_vertices.push_back(vertex);
            out_vertices.push_back(vertex);
        } else if (strcmp(lineHeader, "vt") == 0) {
            glm::vec2 uv;
            fscanf(file, "%f %f\n", &uv.x, &uv.y);
            // cout<<"uv"<<uv.x<<endl;
            temp_uvs.push_back(uv);
        } else if (strcmp(lineHeader, "vn") == 0) {
            glm::vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
            //temp_normals.push_back(normal);
            vertex_normal.push_back(normal);

        } else if (strcmp(lineHeader, "f") == 0) {

            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0],
                                 &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2],
                                 &normalIndex[2]);
            if (matches != 9) {
                ROS_ERROR("File can't be read by our simple parser : ( Try exporting with other options\n");
            }

            /* this mean for later use, just in case */
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            uvIndices.push_back(uvIndex[0]);
            uvIndices.push_back(uvIndex[1]);
            uvIndices.push_back(uvIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);

            temp_face[0] = vertexIndex[0] - 1;
            temp_face[1] = vertexIndex[1] - 1;
            temp_face[2] = vertexIndex[2] - 1;
            temp_face[3] = normalIndex[0] - 1;
            temp_face[4] = normalIndex[1] - 1;
            temp_face[5] = normalIndex[2] - 1;

            out_faces.push_back(temp_face);
        }
    }

    /***find neighbor faces***/
    neighbor_faces.resize(out_faces.size());
    std::vector<int> temp_vec;

    for (int i = 0; i < neighbor_faces.size(); ++i) {
        /*********find the neighbor faces************/
        for (int j = 0; j < out_faces.size(); ++j) {
            if (j != i) {  //don't repeat yourself
                int match = Compare_vertex(out_faces[i], out_faces[j], temp_vec);

                if (match == 2) //so face i and face j share an edge
                {
                    //ROS_INFO_STREAM("SIZE FO THE TEMP VEC: " << temp_vec.size() );
                    neighbor_faces[i].push_back(j); // mark the neighbor face index
                    neighbor_faces[i].push_back(temp_vec[0]);  //first vertex
                    neighbor_faces[i].push_back(temp_vec[1]);  //corresponding normal
                    neighbor_faces[i].push_back(temp_vec[2]);  //second vertex
                    neighbor_faces[i].push_back(temp_vec[3]);  //corresponding normal
                    //ROS_INFO_STREAM("neighbor_faces[i]" << neighbor_faces[i].size());
                }
                temp_vec.clear();
            }

        }

    }

    printf("loaded file %s successfully.\n", path);
};

void ToolModel::Convert_glTocv_pts(std::vector<glm::vec3> &input_vertices, std::vector<cv::Point3d> &out_vertices) {

    unsigned long vsize = input_vertices.size();

    out_vertices.resize(vsize);
    for (int i = 0; i < vsize; ++i) {
        out_vertices[i].x = input_vertices[i].x;
        out_vertices[i].y = input_vertices[i].y;
        out_vertices[i].z = input_vertices[i].z;
    }
};


/* find the camera view point, should it be (0,0,0), input faces stores the indices of the vertices and normals,
which are not related to the pose of the tool object*/
/*** TODO: camera transformations ***/
cv::Mat ToolModel::camTransformMats(cv::Mat &cam_mat, cv::Mat &input_mat) {
    /*cam mat should be a 4x4 extrinsic parameter*/
    cv::Mat output_mat = cam_mat * input_mat; //transform the obj to camera frames, g_CT

    return output_mat;
};

/*************** using Vertices to draw the contour *******************/
void ToolModel::Compute_Silhouette(const std::vector<std::vector<int> > &input_faces,
                                   const std::vector<std::vector<int> > &neighbor_faces,
                                   const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                                   cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                                   const cv::Mat &P, cv::OutputArray jac) {
//    cv::Mat adjoint_mat = (cv::Mat_<double>(4,4) << -1,0,0,0,
//            0,1,0,0,
//            0,0,-1,0,
//            0,0,0,1);
//
//    input_Vmat = adjoint_mat * input_Vmat;
//    input_Nmat = adjoint_mat * input_Nmat;

    cv::Mat new_Vertices = transformPoints(input_Vmat, rvec, tvec);

    new_Vertices = camTransformMats(CamMat, new_Vertices); //transform every point under camera frame

    cv::Mat new_Normals = transformPoints(input_Nmat, rvec, tvec);
    new_Normals = camTransformMats(CamMat, new_Normals); //transform every surface normal under camera frame

    unsigned long neighbor_num = 0;
    cv::Mat temp(4, 1, CV_64FC1);

    cv::Mat ept_1(4, 1, CV_64FC1);
    cv::Mat ept_2(4, 1, CV_64FC1);

    for (int i = 0; i < input_faces.size(); ++i) {
        neighbor_num = (neighbor_faces[i].size()) / 5;  //each neighbor has two vertices, used to be 3 when normals

        if (neighbor_num > 0) {
            int v1 = input_faces[i][0];
            int v2 = input_faces[i][1];
            int v3 = input_faces[i][2];
            int n1 = input_faces[i][3];
            int n2 = input_faces[i][4];
            int n3 = input_faces[i][5];

            new_Vertices.col(v1).copyTo(temp.col(0));
            cv::Point3d pt1 = convert_MattoPts(temp);
            new_Vertices.col(v2).copyTo(temp.col(0));
            cv::Point3d pt2 = convert_MattoPts(temp);
            new_Vertices.col(v3).copyTo(temp.col(0));
            cv::Point3d pt3 = convert_MattoPts(temp);

            new_Normals.col(n1).copyTo(temp.col(0));
            cv::Point3d vn1 = convert_MattoPts(temp);
            new_Normals.col(n2).copyTo(temp.col(0));
            cv::Point3d vn2 = convert_MattoPts(temp);
            new_Normals.col(n3).copyTo(temp.col(0));
            cv::Point3d vn3 = convert_MattoPts(temp);

            cv::Point3d fnormal = FindFaceNormal(pt1, pt2, pt3, vn1, vn2, vn3); //knowing the direction and normalized
            cv::Point3d face_point_i = pt1 + pt2 + pt3;
            face_point_i.x = face_point_i.x / 3;
            face_point_i.y = face_point_i.y / 3;
            face_point_i.z = face_point_i.z / 3;

            double isfront_i = dotProduct(fnormal, face_point_i);
            if(isfront_i < 0.000){
                for (int neighbor_count = 0; neighbor_count <
                                             neighbor_num; ++neighbor_count) {  //notice: cannot use J here, since the last j will not be counted
                    int j = 5 * neighbor_count;
                    int v1_ = input_faces[neighbor_faces[i][j]][0];
                    int v2_ = input_faces[neighbor_faces[i][j]][1];
                    int v3_ = input_faces[neighbor_faces[i][j]][2];

                    int n1_ = input_faces[neighbor_faces[i][j]][3];
                    int n2_ = input_faces[neighbor_faces[i][j]][4];
                    int n3_ = input_faces[neighbor_faces[i][j]][5];

                    new_Vertices.col(v1_).copyTo(temp.col(0));
                    cv::Point3d pt1_ = convert_MattoPts(temp);
                    new_Vertices.col(v2_).copyTo(temp.col(0));
                    cv::Point3d pt2_ = convert_MattoPts(temp);
                    new_Vertices.col(v3_).copyTo(temp.col(0));
                    cv::Point3d pt3_ = convert_MattoPts(temp);

                    new_Normals.col(n1_).copyTo(temp.col(0));
                    cv::Point3d vn1_ = convert_MattoPts(temp);
                    new_Normals.col(n2_).copyTo(temp.col(0));
                    cv::Point3d vn2_ = convert_MattoPts(temp);
                    new_Normals.col(n3_).copyTo(temp.col(0));
                    cv::Point3d vn3_ = convert_MattoPts(temp);

                    cv::Point3d fnormal_n = FindFaceNormal(pt1_, pt2_, pt3_, vn1_, vn2_, vn3_);

                    cv::Point3d face_point_j = pt1_ + pt2_ + pt3_;
                    face_point_j.x = face_point_j.x / 3;
                    face_point_j.y = face_point_j.y / 3;
                    face_point_j.z = face_point_j.z / 3;

                    double isfront_j = dotProduct(fnormal_n, face_point_j);

                    if (isfront_i * isfront_j <= 0.0) // one is front, another is back
                    {
                        /*finish finding, drawing the image*/
                        new_Vertices.col(neighbor_faces[i][j + 1]).copyTo(ept_1);  //under camera frames
                        new_Vertices.col(neighbor_faces[i][j + 3]).copyTo(ept_2);

                        cv::Point2d prjpt_1 = reproject(ept_1, P);
                        cv::Point2d prjpt_2 = reproject(ept_2, P);
                        if (prjpt_1.x <= 640 && prjpt_1.x >= -100 && prjpt_2.x < 640 && prjpt_2.x >= -100)
                        {
                            cv::line(image, prjpt_1, prjpt_2, cv::Scalar(0, 255, 0), 1, 8, 0);
                        }

                    }
                }
            }

        }
    }

};

cv::Point3d ToolModel::convert_MattoPts(cv::Mat &input_Mat) { //should be a 4 by 1 mat
    cv::Point3d output_point;
    output_point.x = input_Mat.at<double>(0, 0);
    output_point.y = input_Mat.at<double>(1, 0);
    output_point.z = input_Mat.at<double>(2, 0);

    return output_point;

};

void ToolModel::getFaceInfo(const std::vector<std::vector<int> > &input_faces,
                            const std::vector<cv::Point3d> &input_vertices,
                            const std::vector<cv::Point3d> &input_Vnormal, cv::Mat &face_normals,
                            cv::Mat &face_centroids) {

    int face_size = input_faces.size();
    face_normals = cv::Mat(4, face_size, CV_64FC1);
    face_centroids = cv::Mat(4, face_size, CV_64FC1);

    for (int i = 0; i < face_size; ++i) {
        int v1 = input_faces[i][0];
        int v2 = input_faces[i][1];
        int v3 = input_faces[i][2];
        int n1 = input_faces[i][3];
        int n2 = input_faces[i][4];
        int n3 = input_faces[i][5];

        cv::Point3d pt1 = input_vertices[v1];
        cv::Point3d pt2 = input_vertices[v2];
        cv::Point3d pt3 = input_vertices[v3];

        cv::Point3d normal1 = input_Vnormal[n1];
        cv::Point3d normal2 = input_Vnormal[n2];
        cv::Point3d normal3 = input_Vnormal[n3];

        cv::Point3d fnormal = FindFaceNormal(pt1, pt2, pt3, normal1, normal2,
                                             normal3); //knowing the direction and normalized

        face_normals.at<double>(0, i) = fnormal.x;
        face_normals.at<double>(1, i) = fnormal.y;
        face_normals.at<double>(2, i) = fnormal.z;
        face_normals.at<double>(3, i) = 0;

        cv::Point3d face_point = pt1 + pt2 + pt3;

        face_point.x = face_point.x / 3.000000;
        face_point.y = face_point.y / 3.000000;
        face_point.z = face_point.z / 3.000000;
        face_point = Normalize(face_point);

        face_centroids.at<double>(0, i) = face_point.x;
        face_centroids.at<double>(1, i) = face_point.y;
        face_centroids.at<double>(2, i) = face_point.z;
        face_centroids.at<double>(3, i) = 1;

    }

};

cv::Point3d ToolModel::crossProduct(cv::Point3d &vec1, cv::Point3d &vec2) {    //3d vector

    cv::Point3d res_vec;
    res_vec.x = vec1.y * vec2.z - vec1.z * vec2.y;
    res_vec.y = vec1.z * vec2.x - vec1.x * vec2.z;
    res_vec.z = vec1.x * vec2.y - vec1.y * vec2.x;

    return res_vec;
};

double ToolModel::dotProduct(cv::Point3d &vec1, cv::Point3d &vec2) {
    double dot_res;
    dot_res = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;

    return dot_res;
};

cv::Point3d ToolModel::Normalize(cv::Point3d &vec1) {
    cv::Point3d norm_res;

    double norm = vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z;
    if (norm == 0.000000) {
        //ROS_ERROR("Trying to normalize a zero vector!");
    } else {
        norm = pow(norm, 0.5);

        norm_res.x = vec1.x / norm;
        norm_res.y = vec1.y / norm;
        norm_res.z = vec1.z / norm;

        return norm_res;
    }

};

cv::Point3d ToolModel::FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                                      cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3) {
    cv::Point3d temp_v1;
    cv::Point3d temp_v2;

    temp_v1.x = input_v1.x - input_v2.x;    //let temp v1 be v1-v2
    temp_v1.y = input_v1.y - input_v2.y;
    temp_v1.z = input_v1.z - input_v2.z;

    temp_v2.x = input_v1.x - input_v3.x;    //let temp v1 be v1-v3
    temp_v2.y = input_v1.y - input_v3.y;
    temp_v2.z = input_v1.z - input_v3.z;

    cv::Point3d res = crossProduct(temp_v1, temp_v2);

    double outward_normal_1 = dotProduct(res, input_n1);
    double outward_normal_2 = dotProduct(res, input_n2);
    double outward_normal_3 = dotProduct(res, input_n3);
    if ((outward_normal_1 < 0) || (outward_normal_2 < 0) || (outward_normal_3 < 0)) {
        res = -res;
    }

    return res;  // knowing the direction

};


int ToolModel::Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2, std::vector<int> &match_vec) {
    int match_count = 0;
    if (vec1.size() != vec2.size())  ///face vectors
    {
        printf("Two vectors are not in the same size \n");
    } else {
        for (int j = 0; j < 3; ++j) {
            if (vec1[0] == vec2[j]) {
                match_count += 1;
                match_vec.push_back(vec1[0]);   //vertex
                match_vec.push_back(vec1[3]);  // corresponding vertex normal

            }

        }

        for (int j = 0; j < 3; ++j) {
            if (vec1[1] == vec2[j]) {
                match_count += 1;
                match_vec.push_back(vec1[1]);
                match_vec.push_back(vec1[4]);  // corresponding vertex normal

            }

        }

        for (int j = 0; j < 3; ++j) {
            if (vec1[2] == vec2[j]) {
                match_count += 1;
                match_vec.push_back(vec1[2]);
                match_vec.push_back(vec1[5]);  // corresponding vertex normal

            }

        }
    }

    return match_count;
};

/*******This function is to do transformations to the raw data from the loader, to offset each part*******/
void ToolModel::modify_model_(std::vector<glm::vec3> &input_vertices, std::vector<glm::vec3> &input_Vnormal,
                              std::vector<cv::Point3d> &input_Vpts, std::vector<cv::Point3d> &input_Npts,
                              double &offset, cv::Mat &input_Vmat, cv::Mat &input_Nmat) {

    Convert_glTocv_pts(input_vertices, input_Vpts);
    ConvertInchtoMeters(input_Vpts);

    int size = input_Vpts.size();
    for (int i = 0; i < size; ++i) {
        input_Vpts[i].y = input_Vpts[i].y - offset;
    }
    Convert_glTocv_pts(input_Vnormal, input_Npts); //not using homogeneous for v weight now
    ConvertInchtoMeters(input_Npts);

    input_Vmat = cv::Mat(4, size, CV_64FC1);

    for (int i = 0; i < size; ++i) {
        input_Vmat.at<double>(0, i) = input_Vpts[i].x;
        input_Vmat.at<double>(1, i) = input_Vpts[i].y;
        input_Vmat.at<double>(2, i) = input_Vpts[i].z;
        input_Vmat.at<double>(3, i) = 1;
    }

    int Nsize = input_Npts.size();
    input_Nmat = cv::Mat(4, Nsize, CV_64FC1);

    for (int i = 0; i < Nsize; ++i) {
        input_Nmat.at<double>(0, i) = input_Npts[i].x;
        input_Nmat.at<double>(1, i) = input_Npts[i].y;
        input_Nmat.at<double>(2, i) = input_Npts[i].z;
        input_Nmat.at<double>(3, i) = 0;
    }

};

/*using cylinder pose to compute rest pose*/
void ToolModel::computeEllipsePose(toolModel &inputModel, const double &theta_ellipse, const double &theta_grip_1,
                                   const double &theta_grip_2) {

    cv::Mat I = cv::Mat::eye(3, 3, CV_64FC1);
    /*********** computations for ellipse kinematics **********/
    ///take cylinder part as the origin
    cv::Mat q_temp(4, 1, CV_64FC1);

    cv::Mat q_ellipse_(4, 1, CV_64FC1);
    q_ellipse_.at<double>(0, 0) = 0;
    q_ellipse_.at<double>(1, 0) = 0.004;//0.011
    q_ellipse_.at<double>(2, 0) = 0;
    q_ellipse_.at<double>(3, 0) = 1;

    q_temp = transformPoints(q_ellipse_, cv::Mat(inputModel.rvec_cyl),
                             cv::Mat(inputModel.tvec_cyl)); //transform the ellipse coord according to cylinder pose

    inputModel.tvec_elp(0) = q_temp.at<double>(0, 0);
    inputModel.tvec_elp(1) = q_temp.at<double>(1, 0);
    inputModel.tvec_elp(2) = q_temp.at<double>(2, 0);

    cv::Mat rot_ellipse(3,3,CV_64FC1);
    cv::Rodrigues(inputModel.rvec_cyl, rot_ellipse);

    double cos_theta = cos(theta_ellipse);
    double sin_theta = sin(theta_ellipse);

    /*** this works in simulation ?? ***/
    cv::Mat g_ellipse = (cv::Mat_<double>(3,3) << cos_theta, -sin_theta, 0,
            sin_theta, cos_theta, 0,
            0,0, 1);

    cv::Mat rot_new =  rot_ellipse * g_ellipse;
    cv::Mat temp_vec(3,1,CV_64FC1);
    cv::Rodrigues(rot_new, inputModel.rvec_elp);

    /*********** computations for gripper kinematics **********/
    cv::Mat test_gripper(3, 1, CV_64FC1);
    test_gripper.at<double>(0, 0) = 0;
    test_gripper.at<double>(1, 0) = 0.001;
    test_gripper.at<double>(2, 0) = 0;

    cv::Mat rot_elp(3, 3, CV_64FC1);
    cv::Rodrigues(inputModel.rvec_elp, rot_elp);  // get rotation mat of the ellipse

    cv::Mat q_rot(3, 1, CV_64FC1);
    q_rot = rot_elp * test_gripper;

    inputModel.tvec_grip1(0) = q_rot.at<double>(0, 0) + inputModel.tvec_elp(0);
    inputModel.tvec_grip1(1) = q_rot.at<double>(1, 0) + inputModel.tvec_elp(1);
    inputModel.tvec_grip1(2) = q_rot.at<double>(2, 0) + inputModel.tvec_elp(2);

    double theta_orien_grip = -1.0 * theta_grip_1; //maybe the x being flipped
    double theta_grip_open = theta_grip_2;
    if(theta_grip_open < 0.0){
        theta_grip_open = 0.0;
    }else{
        theta_grip_open = theta_grip_open + 0.07;
    }

    double grip_2_delta = theta_orien_grip - theta_grip_open;
    double grip_1_delta = theta_orien_grip + theta_grip_open;

    cos_theta = cos(grip_1_delta);
    sin_theta = sin(grip_1_delta);

    cv::Mat gripper_1_ = (cv::Mat_<double>(3,3) << 1, 0, 0,
            0,cos_theta,-sin_theta,
            0, sin_theta, cos_theta);

    cv::Mat rot_grip_1 = rot_elp * gripper_1_ ;
    cv::Rodrigues(rot_grip_1, inputModel.rvec_grip1);

    /*gripper 2*/
    inputModel.tvec_grip2(0) = inputModel.tvec_grip1(0);
    inputModel.tvec_grip2(1) = inputModel.tvec_grip1(1);
    inputModel.tvec_grip2(2) = inputModel.tvec_grip1(2);

    cos_theta = cos(grip_2_delta);
    sin_theta = sin(grip_2_delta);

    cv::Mat gripper_2_ = (cv::Mat_<double>(3,3) << 1, 0, 0,
            0,cos_theta,-sin_theta,
            0, sin_theta, cos_theta);

    cv::Mat rot_grip_2 = rot_elp * gripper_2_;
    cv::Rodrigues(rot_grip_2, inputModel.rvec_grip2);
};

cv::Mat ToolModel::computeSkew(cv::Mat &w) {
    cv::Mat skew(3, 3, CV_64FC1);
    skew.at<double>(0, 0) = 0;
    skew.at<double>(1, 0) = w.at<double>(2, 0);
    skew.at<double>(2, 0) = -w.at<double>(1, 0);
    skew.at<double>(0, 1) = -w.at<double>(2, 0);
    skew.at<double>(1, 1) = 0;
    skew.at<double>(2, 1) = w.at<double>(0, 0);
    skew.at<double>(0, 2) = w.at<double>(1, 0);
    skew.at<double>(1, 2) = -w.at<double>(0, 0);
    skew.at<double>(2, 2) = 0;

    return skew;

};

void ToolModel::computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat){

    outputMat = cv::Mat::eye(4,4,CV_64F);

    cv::Mat R = inputMat.colRange(0,3).rowRange(0,3);
    cv::Mat p = inputMat.colRange(3,4).rowRange(0,3);

    /*debug: opencv......*/
    cv::Mat R_rat = R.clone();
    cv::Mat p_tra = p.clone();

    R_rat = R_rat.t();  // rotation of inverse
    p_tra = -1 * R_rat * p_tra; // translation of inverse

    R_rat.copyTo(outputMat.colRange(0,3).rowRange(0,3));
    p_tra.copyTo(outputMat.colRange(3,4).rowRange(0,3));

}

/****render a rectangle contains the tool model, TODO:*****/
void
ToolModel::renderTool(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P, cv::OutputArray jac) {

    /** approach 1: using Vertices mat and normal mat **/
    Compute_Silhouette(body_faces, body_neighbors, body_Vmat, body_Nmat, CamMat, image, cv::Mat(tool.rvec_cyl),
                       cv::Mat(tool.tvec_cyl), P, jac);

    Compute_Silhouette(ellipse_faces, ellipse_neighbors, ellipse_Vmat, ellipse_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_elp), cv::Mat(tool.tvec_elp), P, jac);

    Compute_Silhouette(griper1_faces, griper1_neighbors, gripper1_Vmat, gripper1_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_grip1), cv::Mat(tool.tvec_grip1), P, jac);

    Compute_Silhouette(griper2_faces, griper2_neighbors, gripper2_Vmat, gripper2_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_grip2), cv::Mat(tool.tvec_grip2), P, jac);

};

float ToolModel::calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage) {

    float matchingScore;

    /*** When ROI is an empty rec, the position of tool is simply just not match, return 0 matching score ***/
    cv::Mat ROI_toolImage = toolImage.clone(); //(ROI); //crop tool image
    cv::Mat segImageGrey = segmentedImage.clone(); //(ROI); //crop segmented image, notice the size of the segmented image

    segImageGrey.convertTo(segImageGrey, CV_32FC1);
    cv::Mat segImgBlur;
    cv::GaussianBlur(segImageGrey,segImgBlur, cv::Size(9,9),4,4);
    segImgBlur /= 255; //scale the blurred image

    cv::Mat toolImageGrey; //grey scale of toolImage since tool image has 3 channels
    cv::Mat toolImFloat; //Float data type of grey scale tool image

    cv::cvtColor(ROI_toolImage, toolImageGrey, CV_BGR2GRAY); //convert it to grey scale

    toolImageGrey.convertTo(toolImFloat, CV_32FC1); // convert grey scale to float
    cv::Mat result(1, 1, CV_32FC1);

    cv::matchTemplate(segImgBlur, toolImFloat, result, CV_TM_CCORR_NORMED); //seg, toolImg
    matchingScore = static_cast<float> (result.at<float>(0));

    return matchingScore;
}

/*** chamfer matching algorithm, using distance transform, generate measurement model for PF ***/
float ToolModel::calculateChamferScore(cv::Mat &toolImage, const cv::Mat &segmentedImage) {

    float output = 0;
    cv::Mat ROI_toolImage = toolImage.clone(); //CV_8UC3
    cv::Mat segImgGrey = segmentedImage.clone(); //CV_8UC1

    segImgGrey.convertTo(segImgGrey, CV_8UC1);

    /***tool image process**/
    cv::Mat toolImageGrey(ROI_toolImage.size(), CV_8UC1); //grey scale of toolImage since tool image has 3 channels
    cv::Mat toolImFloat(ROI_toolImage.size(), CV_32FC1); //Float data type of grey scale tool image
    cv::cvtColor(ROI_toolImage, toolImageGrey, CV_BGR2GRAY); //convert it to grey scale

    toolImageGrey.convertTo(toolImFloat, CV_32FC1); // get float img

    cv::Mat BinaryImg(toolImFloat.size(), toolImFloat.type());
    BinaryImg = toolImFloat * (1.0/255);

    if(countNonZero(BinaryImg) < 10){
        output = 1000;
    }else{
        /***segmented image process**/
        for (int i = 0; i < segImgGrey.rows; i++) {
            for (int j = 0; j < segImgGrey.cols; j++) {
                segImgGrey.at<uchar>(i,j) = 255 - segImgGrey.at<uchar>(i,j);

            }
        }

        cv::Mat normDIST;
        cv::Mat distance_img;
        cv::distanceTransform(segImgGrey, distance_img, CV_DIST_L2, 3);
        cv::normalize(distance_img, normDIST, 0.00, 1.00, cv::NORM_MINMAX);

//    cv::imshow("segImgGrey img", segImgGrey);
//    cv::imshow("Normalized img", normDIST);
////    cv::imshow("distance_img", distance_img);
//    cv::waitKey();

        /***multiplication process**/
        cv::Mat resultImg; //initialize
        cv::multiply(normDIST, BinaryImg, resultImg);

        for (int k = 0; k < resultImg.rows; ++k) {
            for (int i = 0; i < resultImg.cols; ++i) {

                double mul = resultImg.at<float>(k,i);
                if(mul > 0.0)
                    output += mul;
            }
        }
        /**
         * Didn't flip the score because in this pkg we need the errors instead of matching score
         */
//    //ROS_INFO_STREAM("OUTPUT: " << output);
//    output = exp(-1 * output/80);
    }

    return output;

};

/*********** reproject a single point under the camera onto a image, FOR THE BODY COORD TRANSFORMATION ***************/
cv::Point2d ToolModel::reproject(const cv::Mat &point, const cv::Mat &P) {
    cv::Mat results(3, 1, CV_64FC1);
    cv::Point2d output;

    cv::Mat ptMat(4, 1, CV_64FC1);
    ptMat.at<double>(0, 0) = point.at<double>(0, 0);
    ptMat.at<double>(1, 0) = point.at<double>(1, 0);
    ptMat.at<double>(2, 0) = point.at<double>(2, 0);
    ptMat.at<double>(3, 0) = 1.0;

    results = P * ptMat;
    output.x = results.at<double>(0, 0) / results.at<double>(2, 0);
    output.y = results.at<double>(1, 0) / results.at<double>(2, 0);

    return output;
};