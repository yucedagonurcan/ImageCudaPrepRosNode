//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_PROCESSOR_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_PROCESSOR_H_
#include <sensor_msgs/Image.h>

namespace ips{

class ImageProcessing{
 public:
  virtual ~ImageProcessing() {}
  bool m_Run();

 protected:
  virtual bool m_InitialMember(const sensor_msgs::Image &srcMsg) = 0;
  virtual bool m_CudaAllocation(void **src, void **dst) = 0;
  virtual bool m_Processing(void **src, void **dst) = 0;

  int m_srcW, m_srcH;
  int m_dstW, m_dstH;
  int m_srcStep, m_dstStep;

  std::string m_srcEncoding;
  std::string m_dstEncoding;

  uint32_t m_srcChannelNum;
  uint32_t m_dstChannelNum;
};

}
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_PROCESSOR_H_
