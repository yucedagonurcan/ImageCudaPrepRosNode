//
// Created by yucedagonurcan on 11/28/22.
//
#include "ros_image_processing.h"




void ImageProcCUDA::imageCallback(const sensor_msgs::Image::ConstPtr& img){

  if(img->encoding == "bayer_rggb8" || img->encoding == "bayer_bggr8"){


    int img_width = img->width;
    int img_height = img->height;
    int img_ch_num = sensor_msgs::image_encodings::numChannels(img->encoding);

    float scale = 0.5f;

    sensor_msgs::Image color_img;
    sensor_msgs::Image out_img;

    if( ! img_debayer_->m_Run(*img, color_img) ){
      ROS_ERROR("There is some problem in Debayer section.");
      exit(1);
    }


    if(img_resizer_->m_Run(color_img, out_img)){

      int srcStep = img_width * img_ch_num;
      int dstStep = img_width * scale * img_ch_num;

      void **src;
      void **dst;

      *src = nppiMalloc_8u_C1(img_width, img_height, reinterpret_cast<int *>( img_width * img_ch_num ) );
      *dst = nppiMalloc_8u_C3(img_width*scale, img_height*scale, reinterpret_cast<int *>( (int)(img_width * scale) * img_ch_num ) );

      std::vector<uint8_t> resizedImage(dstStep * img_height*scale, 0);

      if (CheckCUDA(
               cudaMemcpy2D(&resizedImage[0], dstStep * img_ch_num,
                             dst, m_dstStep,
                             m_dstW * m_srcChannelNum, m_dstH, cudaMemcpyDeviceToHost),
               "cudaMemcpy2D") != 0) return false;
    }



      img_pub_.publish(out_img);
    }
  } else {

    sensor_msgs::Image out_img;

    if(img_resizer_->m_Run(*img, out_img))
      img_pub_.publish(out_img);
  }

}

ImageProcCUDA::ImageProcCUDA(): nh_(), pnh_("~") {

  img_sub_ = nh_.subscribe("/dash_right/image_raw", 1, &ImageProcCUDA::imageCallback, this);
  img_pub_ = pnh_.advertise<sensor_msgs::Image>("/cudacim", 1);

  img_resizer_.reset(new ips::ImageResizer(1440, 928, 1440/2, 928/2));
  img_debayer_.reset(new ips::Debayer(1440, 928));
}