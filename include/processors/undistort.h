//
// Created by yucedagonurcan on 11/30/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_UNDISTORT_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_UNDISTORT_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "processor.h"

template<typename T>
__global__ void gpuIntrinsicWarp( T* input, T* output, int width, int height,
                                  float2 focalLength, float2 principalPoint, float k1, float k2, float p1, float p2);


namespace ips {

class Undistort : public ImageProcessing {
 public:
  Undistort( const ImageInfo &src_info, const ImageInfo &dst_info );
  bool validateProcessor();
  bool m_Run( void **srcImage, void **dstImage );

 protected:
  bool m_Processing( void **src, void **dst );
};
}
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_UNDISTORT_H_
