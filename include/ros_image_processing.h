//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef BOXFILTERNPP_BOXFILTERNPP_ROSIMAGEPROCESSING_H_
#define BOXFILTERNPP_BOXFILTERNPP_ROSIMAGEPROCESSING_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "processors/resize.h"
#include "processors/debayer.h"

#include <Exceptions.h>
#include <ImageIO.h>
#include <ImagesCPU.h>
#include <ImagesNPP.h>

#include "nppi_geometry_transforms.h"
#include "alloc_utils.h"


class ImageProcCUDA{
 public:
  ImageProcCUDA();

 private:

  std::shared_ptr<ips::Resizer> img_resizer_;
  std::shared_ptr<ips::Debayer> img_debayer_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber img_sub_;
  ros::Publisher img_pub_;

  ImageInfo src_info_;
  ImageInfo color_img_info_;
  ImageInfo resize_img_info_;

  void imageCallback(const sensor_msgs::Image::ConstPtr& img);
};


#endif//BOXFILTERNPP_BOXFILTERNPP_ROSIMAGEPROCESSING_H_
