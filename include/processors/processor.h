//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_PROCESSOR_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_PROCESSOR_H_
#include <sensor_msgs/Image.h>
#include "alloc_utils.h"

namespace ips{

class ImageProcessing{
 public:
  virtual ~ImageProcessing() {}
  bool m_Run();

 protected:
  virtual bool m_Processing(void **src, void **dst) = 0;
  virtual bool validateProcessor() = 0;
  
  ImageInfo src_info_;
  ImageInfo dst_info_;
};

}
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_PROCESSOR_H_
