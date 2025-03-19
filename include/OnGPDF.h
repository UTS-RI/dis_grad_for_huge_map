// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <functional>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

typedef Eigen::MatrixXd EMatrixX;
typedef Eigen::VectorXd EVectorX;
typedef Eigen::RowVectorXd ERowVectorX;

inline double kf_se(double r, double a){return exp(-r*r*a*0.5);} // se kernel
inline double kf_se1(double r, double dx,double a){return -dx*a*exp(-r*r*a*0.5);} // 1 derivative of se kernel
inline double kf_Se2(double r, double dx1, double dx2, double delta, double a){ // 2 derivative of se kernel
  return (dx1*dx2*a*a-delta*a)*exp(-r*r*a*0.5);}

inline double kf_ma(double r, double a) {return (1.0+a*r)*exp(-a*r);} // matern kernel
inline double kf_ma1(double r, double dx, double a) {return a*a*dx*exp(-a*r);} // 1 derivative of se kernel
inline double kf_ma2(float r, float dx1, float dx2, float delta, float a){ // 2 derivative of se kernel
  return a*a*(delta-a*dx1*dx2/r)*exp(-a*r);}

class OnGPDF{
public:
    OnGPDF(std::vector<Eigen::Vector3d> trainingPoints,
            int distance_method_type,
            double para_map_lambda_scale,
            double para_map_noise):
            distance_method(distance_method_type),
            map_lambda_scale(para_map_lambda_scale),
            map_noise(para_map_noise),
            trained(false),
            points(trainingPoints){}

    void reset();
    bool isTrained(){return trained;}
    std::vector<Eigen::Vector3d> getPoints(){return points;}

    void train(std::vector<Eigen::Vector3d> leafVoxels);
    void testSingleDistanceGradient(const EVectorX& xt, double& val, double grad[]);

    // covariances for x1 (input points) with different noise params for inputs
    EMatrixX se_kernel_3D(EMatrixX const& x1, double scale_param, double sigx);

    // covariances for x1 (input points) and x2 (test points)
    EMatrixX se_kernel_3D(EMatrixX const& x1, EMatrixX const& x2, double scale_param);

private:
    EMatrixX x;
    EMatrixX L;
    EMatrixX L_rgb;
    EVectorX alpha;
    EVectorX alphaR;
    EVectorX alphaG;
    EVectorX alphaB;
    
    int distance_method = 0;
    double map_lambda_scale = 0;
    double map_noise = 0;
    bool trained = false;
    std::vector<Eigen::Vector3d> points;
}; // OnGPDF