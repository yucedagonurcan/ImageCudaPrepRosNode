//
// Created by yucedagonurcan on 11/28/22.
//
#include "ros_image_processing.h"

void ImageProcCUDA::imageCallback(const sensor_msgs::Image::ConstPtr& img){

    void* src_ptr = nullptr;
    void* color_ptr = nullptr;
    void* resized_ptr = nullptr;

    const auto num_channels_src = sensor_msgs::image_encodings::numChannels(src_info_.encoding);
    const auto num_channels_color = sensor_msgs::image_encodings::numChannels(color_img_info_.encoding);
    const auto num_channels_resized = sensor_msgs::image_encodings::numChannels(resize_img_info_.encoding);

    auto alloc_success = alloc_utils::m_ImageCudaAlloc(&src_ptr, &color_ptr, src_info_, color_img_info_);
    if(!alloc_success){
      ROS_ERROR("Allocation is unsuccessfull, Source and Color images!");
      exit(1);
    }


    if (ips::CheckCUDA(
             cudaMemcpy2D(src_ptr, src_info_.step,
                           &img->data[0], src_info_.step,
                           src_info_.width * num_channels_src, src_info_.height, cudaMemcpyHostToDevice),
             "cudaMemcpy2D") != 0){
      ROS_ERROR("Something went wrong in Debayer process.");
      exit(1);
    }


    if( ! img_debayer_->m_Run(&src_ptr, &color_ptr) ){
      ROS_ERROR("There is some problem in Debayer section.");
      exit(1);
    }

    alloc_success = alloc_utils::imageCudaAlloc(&resized_ptr, resize_img_info_);
    if(!alloc_success){
      ROS_ERROR("Allocation is unsuccessfull, Resized image!");
      exit(1);
    }

    if( ! img_resizer_->m_Run(&color_ptr, &resized_ptr) ){
      ROS_ERROR("There is some problem in Resized section.");
      exit(1);
    }


    std::vector<uint8_t> resizedImage(resize_img_info_.width * resize_img_info_.height * num_channels_resized, 100);

    if (ips::CheckCUDA(
             cudaMemcpy2D(&resizedImage[0], resize_img_info_.step,
                           resized_ptr, resize_img_info_.step,
                           resize_img_info_.width * num_channels_resized, resize_img_info_.height,
                           cudaMemcpyDeviceToHost),
             "cudaMemcpy2D") != 0){
      ROS_ERROR("Something went wrong in Resize process.");
      exit(1);
    }

    sensor_msgs::Image resize_img;
    {
      resize_img.header.frame_id = img->header.frame_id;
      resize_img.width = resize_img_info_.width;
      resize_img.height = resize_img_info_.height;
      resize_img.step = resize_img_info_.width * num_channels_color;
      resize_img.encoding = "rgb8";
      resize_img.is_bigendian = img->is_bigendian;

      resize_img.data = std::move(resizedImage);
    }
    img_pub_.publish(resize_img);
}

ImageProcCUDA::ImageProcCUDA(): nh_(), pnh_("~") {

  img_sub_ = nh_.subscribe("/dash_right/image_raw", 1, &ImageProcCUDA::imageCallback, this);
  img_pub_ = pnh_.advertise<sensor_msgs::Image>("/cudacim", 1);

  src_info_.width = 1440;
  src_info_.height = 928;
  src_info_.encoding = "bayer_rggb8";
  const auto num_channels_src = sensor_msgs::image_encodings::numChannels(src_info_.encoding);
  src_info_.step = src_info_.width * num_channels_src;

  color_img_info_.width = src_info_.width;
  color_img_info_.height = src_info_.height;
  color_img_info_.encoding = "rgb8";
  const auto num_channels_color = sensor_msgs::image_encodings::numChannels(color_img_info_.encoding);
  color_img_info_.step = color_img_info_.width * num_channels_color;

  img_debayer_.reset(new ips::Debayer(src_info_, color_img_info_));

  resize_img_info_.width = color_img_info_.width/3;
  resize_img_info_.height = color_img_info_.height;
  resize_img_info_.encoding = "rgb8";
  const auto num_channels_resize = sensor_msgs::image_encodings::numChannels(resize_img_info_.encoding);
  resize_img_info_.step = resize_img_info_.width * num_channels_resize;

  img_resizer_.reset(new ips::Resizer(color_img_info_, resize_img_info_));


}