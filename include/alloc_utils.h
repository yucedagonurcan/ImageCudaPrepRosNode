//
// Created by yucedagonurcan on 11/29/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_ALLOC_UTILS_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_ALLOC_UTILS_H_
#include <string>
#include <npp.h>
#include "ErrorCheck.h"

struct ImageInfo{
  int width;
  int height;
  int step;

  int num_channels;
  std::string encoding;

  // Intrinsic Parameters
  float k1;
  float k2;
  float p1;
  float p2;
  float2 focal;
  float2 principal;
};


namespace alloc_utils {

inline bool imageCudaAlloc( void **img, const ImageInfo img_info ) {

  int srcStep;

  if ( img_info.encoding == "mono8" ) {
    *img = nppiMalloc_8u_C1( img_info.width, img_info.height, &srcStep );
    return true;
  } else if ( img_info.encoding == "mono16" ) {
    *img = nppiMalloc_16u_C1( img_info.width, img_info.height, &srcStep );
    return true;
  } else if ( img_info.encoding == "rgb8" || img_info.encoding == "bgr8" ) {
    *img = nppiMalloc_8u_C3( img_info.width, img_info.height, &srcStep );
    return true;
  } else if ( img_info.encoding == "rgb16" || img_info.encoding == "bgr16" ) {
    *img = nppiMalloc_16u_C3( img_info.width, img_info.height, &srcStep );
    return true;
  } else if ( img_info.encoding == "rgba8" || img_info.encoding == "bgra8" ) {
    *img = nppiMalloc_8u_C4( img_info.width, img_info.height, &srcStep );
    return true;
  } else if ( img_info.encoding == "rgba16" || img_info.encoding == "bgra16" ) {
    *img = nppiMalloc_16u_C4( img_info.width, img_info.height, &srcStep );
    return true;
  } else if ( img_info.encoding == "bayer_rggb8" || img_info.encoding == "bayer_bggr8" ) {
    *img = nppiMalloc_8u_C1( img_info.width, img_info.height, &srcStep );
    return true;
  } else
    return false;
}

inline bool m_imageCudaAlloc( void **src, void **dst, const ImageInfo src_info, const ImageInfo dst_info ) {

  auto success_src = imageCudaAlloc( src, src_info );
  auto success_dst = imageCudaAlloc( dst, dst_info );

  return success_src && success_dst;
}

inline bool m_copyImageFromHostToDevice( void *device_ptr, const std::vector<uint8_t>& host_vec, const ImageInfo& img_info) {

  const auto num_channels_src = sensor_msgs::image_encodings::numChannels(img_info.encoding);

  if (ips::CheckCUDA(
           cudaMemcpy2D(device_ptr, img_info.step,
                         &host_vec[0], img_info.step,
                         img_info.width * num_channels_src, img_info.height, cudaMemcpyHostToDevice),
           "cudaMemcpy2D") != 0){
    ROS_ERROR("Something went wrong in m_copyImageFromHostToDevice process.");
    return false;
  }
  return true;
}

inline bool m_copyImageFromDeviceToHost( void *device_ptr, std::vector<uint8_t>& host_vec, const ImageInfo& img_info) {

  const auto num_channels_src = sensor_msgs::image_encodings::numChannels(img_info.encoding);

  if (ips::CheckCUDA(
           cudaMemcpy2D(&host_vec[0], img_info.step,
                         device_ptr, img_info.step,
                         img_info.width * num_channels_src, img_info.height, cudaMemcpyDeviceToHost),
           "cudaMemcpy2D") != 0){
    ROS_ERROR("Something went wrong in m_copyImageFromDeviceToHost process.");
    return false;
  }
  return true;
}

}
#endif//ROS_IMAGE_PROCESSING_INCLUDE_ALLOC_UTILS_H_
