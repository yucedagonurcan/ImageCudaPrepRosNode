//
// Created by yucedagonurcan on 11/28/22.
//
#include "ros_image_processing.h"

void ImageProcCUDA::imageCallback(const sensor_msgs::Image::ConstPtr& img){

  if(img->encoding == "bayer_rggb8" || img->encoding == "bayer_bggr8"){

    sensor_msgs::Image color_img;
    sensor_msgs::Image out_img;

    if( ! img_debayer_->m_Run(*img, color_img) ){
      ROS_ERROR("There is some problem in Debayer section.");
      exit(1);
    }

    if(img_resizer_->m_Run(color_img, out_img))
      img_pub_.publish(out_img);
  } else {

    sensor_msgs::Image out_img;

    if(img_resizer_->m_Run(*img, out_img))
      img_pub_.publish(out_img);
  }

}

ImageProcCUDA::ImageProcCUDA(): nh_(), pnh_("~") {

  img_sub_ = nh_.subscribe("/dash_right/image_raw", 1, &ImageProcCUDA::imageCallback, this);
  img_pub_ = pnh_.advertise<sensor_msgs::Image>("/cudacim", 1);

  img_resizer_.reset(new ips::ImageResizer(1440/2, 928/2));
  img_debayer_.reset(new ips::Debayer(1440, 928));
}