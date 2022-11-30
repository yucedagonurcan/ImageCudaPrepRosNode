//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_UNDISTORT_CU_
#define ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_UNDISTORT_CU_

#include <ros/ros.h>
#include <ros/console.h>

#include "undistort.h"
#include "../ErrorCheck.h"

int iDivUp(int a, int b){

  return (a % b != 0) ? (a / b + 1) : (a / b);

}

// gpuIntrinsicWarp
template<typename T>
__global__ void gpuIntrinsicWarp( T* input, T* output, int width, int height,
                                  float2 focalLength, float2 principalPoint, float k1, float k2, float p1, float p2)
{
  const int2 uv_out = make_int2(blockDim.x * blockIdx.x + threadIdx.x,
                                 blockDim.y * blockIdx.y + threadIdx.y);

  if( uv_out.x >= width || uv_out.y >= height )
    return;

  const float u = uv_out.x;
  const float v = uv_out.y;

  const float _fx = 1.0f / focalLength.x;
  const float _fy = 1.0f / focalLength.y;

  const float y      = (v - principalPoint.y)*_fy;
  const float y2     = y*y;
  const float _2p1y  = 2.0*p1*y;
  const float _3p1y2 = 3.0*p1*y2;
  const float p2y2   = p2*y2;

  const float x  = (u - principalPoint.x)*_fx;
  const float x2 = x*x;
  const float r2 = x2 + y2;
  const float d  = 1.0 + (k1 + k2*r2)*r2;
  const float _u = focalLength.x*(x*(d + _2p1y) + p2y2 + (3.0*p2)*x2) + principalPoint.x;
  const float _v = focalLength.y*(y*(d + (2.0*p2)*x) + _3p1y2 + p1*x2) + principalPoint.y;

  const int2 uv_in = make_int2( _u, _v );

  if( uv_in.x >= width || uv_in.y >= height || uv_in.x < 0 || uv_in.y < 0 )
    return;

  output[uv_out.y * width + uv_out.x] = input[uv_in.y * width + uv_in.x];
}


namespace ips{

  Undistort::Undistort(const ImageInfo& src_info, const ImageInfo& dst_info) {

    src_info_ = src_info;
    dst_info_ = dst_info;

  }

  bool Undistort::validateProcessor(){
    if (  src_info_.encoding != "rgb8"){

      ROS_ERROR("[Undistort] %s is invalid encoding format! Not supported encoding type.", src_info_.encoding.c_str());
      return false;
    }
    return true;
  }

  bool Undistort::m_Run(void** srcImage, void** dstImage) {
    if(! validateProcessor()){
      ROS_ERROR("There is an error in validation of `Undistort` processor");
      exit(1);
    }
    if (!m_Processing(srcImage, dstImage)) return false;
    return true;
  }

  bool Undistort::m_Processing(void **src, void **dst) {

    if( !src || !dst )
      return cudaErrorInvalidDevicePointer;

    if( src_info_.width == 0 || src_info_.height == 0 )
      return cudaErrorInvalidValue;

    // launch kernel
    const dim3 blockDim(8, 8);
    const dim3 gridDim(iDivUp(src_info_.width,blockDim.x), iDivUp(src_info_.height,blockDim.y));

    gpuIntrinsicWarp<<<gridDim, blockDim>>>((uchar3*)*src, (uchar3*)*dst, src_info_.width, src_info_.height,
                                             src_info_.focal, src_info_.principal,
                                             src_info_.k1, src_info_.k2, src_info_.p1, src_info_.p2);

    return CheckCUDA(cudaGetLastError(), "gpuIntrinsicWarp") == 0;
  };

};
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_UNDISTORT_CU_
