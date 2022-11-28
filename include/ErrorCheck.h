//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_ERRORCHECK_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_ERRORCHECK_H_

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// CUDA
#include <npp.h>
namespace ips {

inline cudaError_t CheckCUDA(cudaError_t status, const std::string func) {
  if (status != cudaSuccess)
    ROS_ERROR("[CUDA ERROR][%s] %d", func.c_str(), status);

  return status;
}

inline NppStatus CheckNPP(NppStatus status, const std::string func) {
  if (status != NPP_NO_ERROR)
    ROS_ERROR("[NPP ERROR][%s] %d", func.c_str(), status);

  return status;
}

} // namespace ips
#endif//ROS_IMAGE_PROCESSING_INCLUDE_ERRORCHECK_H_
