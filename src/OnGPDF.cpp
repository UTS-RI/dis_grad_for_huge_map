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

#include "OnGPDF.h"

void OnGPDF::reset(){
  trained = false;
  return;
}

void OnGPDF::train(std::vector<Eigen::Vector3d> leafVoxels){
  reset();

  int N = leafVoxels.size();
  int dim = 3;

  if (N > 0){
      x = EMatrixX::Zero(dim,N);
      EVectorX f = EVectorX::Zero(N);
      double sigx = map_noise;

      int k=0;
      for (auto it = leafVoxels.begin(); it!=leafVoxels.end(); it++, k++){
          x(0,k) = (*it).x();
          x(1,k) = (*it).y();
          x(2,k) = (*it).z();
          f(k) = 1; // refer to our paper
      }
      
      EVectorX y(N);
      y << f;
      EMatrixX K = se_kernel_3D(x, map_lambda_scale, sigx);

      L = K.llt().matrixL();
      alpha = y;
      L.template triangularView<Eigen::Lower>().solveInPlace(alpha);
      L.transpose().template triangularView<Eigen::Upper>().solveInPlace(alpha);

      trained = true;
  }
  return;
}

void OnGPDF::testSingleDistanceGradient(const EVectorX& xt, double& val, double grad[])
{
  if (!isTrained())
      return;

  if (x.rows() != xt.size())
      return;

  EMatrixX K = se_kernel_3D(x, xt, map_lambda_scale);
  EVectorX res = K.transpose()*alpha;
  val = res(0);

  int n = x.cols();
  int m = xt.cols();

  double a = map_lambda_scale;
  EMatrixX Grx = EMatrixX::Zero(n,m);
  EMatrixX Gry = EMatrixX::Zero(n,m);
  EMatrixX Grz = EMatrixX::Zero(n,m);

  for (int k=0;k<n;k++){
    for (int j=0;j<m;j++){
        double r = (x.col(k)-xt.col(j)).norm();
        if(distance_method == 0){
          Grx(k, j) = kf_se1(r,x(0, k) - xt(0, j),a);
          Gry(k, j) = kf_se1(r,x(1, k) - xt(1, j),a);
          Grz(k, j) = kf_se1(r,x(2, k) - xt(2, j),a);
        }else if(distance_method == 1){
          Grx(k, j) = kf_ma1(r,x(0, k) - xt(0, j),a);
          Gry(k, j) = kf_ma1(r,x(1, k) - xt(1, j),a);
          Grz(k, j) = kf_ma1(r,x(2, k) - xt(2, j),a);
        }
    }
  }

  EVectorX gradx = Grx.transpose()*alpha;
  EVectorX grady = Gry.transpose()*alpha;
  EVectorX gradz = Grz.transpose()*alpha;

  grad[0] = gradx(0);  
  grad[1] = grady(0);
  grad[2] = gradz(0);

  double gradLen = sqrt(pow(grad[0], 2) + pow(grad[1], 2) + pow(grad[2], 2)); 
  
  if(gradLen != 0){
      grad[0]/=gradLen;
      grad[1]/=gradLen;
      grad[2]/=gradLen;
  } else {
      grad[0]=0;
      grad[1]=0;
      grad[2]=0;
  }
  return;
}

// 3D train
EMatrixX OnGPDF::se_kernel_3D(EMatrixX const& x1, double scale_param, double sigx)
{
  int n = x1.cols();
  double a = scale_param;
  EMatrixX K = EMatrixX::Zero(n,n);

  for (int k=0;k<n;k++){
      for (int j=k;j<n;j++){
          if (k==j){
              K(k,k) = 1.0+sigx;
          }
          else{
              double r = (x1.col(k)-x1.col(j)).norm();
              if(distance_method == 0){
                K(k,j) = kf_se(r,a);
              }else if(distance_method == 1){
                K(k,j) = kf_ma(r,a);
              }
              K(j,k) = K(k,j);
          }
      }
   }
  return K;
}

// 3D test
EMatrixX OnGPDF::se_kernel_3D(EMatrixX const& x1, EMatrixX const& x2, double scale_param)
{
  int n = x1.cols();
  int m = x2.cols();
  double a = scale_param;
  EMatrixX K = EMatrixX::Zero(n,m);

   for (int k=0;k<n;k++){
      for (int j=0;j<m;j++){
          double r = (x1.col(k)-x2.col(j)).norm();
          if(distance_method == 0){
            K(k,j) = kf_se(r,a);
          }else if(distance_method == 1){
            K(k,j) = kf_ma(r,a);
          }
      }
   }
  return K;
}