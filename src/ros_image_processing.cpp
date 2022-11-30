//
// Created by yucedagonurcan on 11/28/22.
//
#include "ros_image_processing.h"

void ImageProcCUDA::imageCallback( const sensor_msgs::Image::ConstPtr &img ) {

  if ( !collected_image_info ) {

    src_info_.num_channels = sensor_msgs::image_encodings::numChannels( src_info_.encoding );
    color_img_info_.num_channels = sensor_msgs::image_encodings::numChannels( color_img_info_.encoding );
    undistort_img_info_.num_channels = sensor_msgs::image_encodings::numChannels( undistort_img_info_.encoding );
    resize_img_info_.num_channels = sensor_msgs::image_encodings::numChannels( resize_img_info_.encoding );

    auto alloc_success = alloc_utils::m_imageCudaAlloc( &src_ptr_, &color_ptr_, src_info_, color_img_info_ );
    if ( !alloc_success ) {
      ROS_ERROR( "Allocation is unsuccessfull, Source and Color images!" );
      exit( 1 );
    }

    alloc_success = alloc_utils::imageCudaAlloc( &undistort_ptr_, undistort_img_info_ );
    if ( !alloc_success ) {
      ROS_ERROR( "Allocation is unsuccessfull, Undistort image!" );
      exit( 1 );
    }

    alloc_success = alloc_utils::imageCudaAlloc( &resized_ptr_, resize_img_info_ );
    if ( !alloc_success ) {
      ROS_ERROR( "Allocation is unsuccessfull, Resized image!" );
      exit( 1 );
    }

    output_img_.data.resize( resize_img_info_.width * resize_img_info_.height * resize_img_info_.num_channels );
    collected_image_info = true;
  }

  if ( !alloc_utils::m_copyImageFromHostToDevice( src_ptr_, img->data, src_info_ ) ) {
    ROS_ERROR( "Something went wrong in Debayer-HostToDeviceCopy process." );
    exit( 1 );
  }

  // SEC DEBAYER
  if ( !img_debayer_->m_Run( &src_ptr_, &color_ptr_ ) ) {
    ROS_ERROR( "There is some problem in Debayer section." );
    exit( 1 );
  }

  // SEC UNDISTORT

  if ( !img_undistort_->m_Run( &color_ptr_, &undistort_ptr_ ) ) {
    ROS_ERROR( "There is some problem in Undistortion section." );
    exit( 1 );
  }

  // SEC RESIZE
  if ( !img_resizer_->m_Run( &undistort_ptr_, &resized_ptr_ ) ) {
    ROS_ERROR( "There is some problem in Resized section." );
    exit( 1 );
  }

  // SEC Generate Output

  if ( !alloc_utils::m_copyImageFromDeviceToHost( resized_ptr_, output_img_.data, resize_img_info_ ) ) {
    ROS_ERROR( "Something went wrong in m_copyImageFromDeviceToHost process." );
    exit( 1 );
  }

  output_img_.header.frame_id = img->header.frame_id;
  output_img_.width = resize_img_info_.width;
  output_img_.height = resize_img_info_.height;
  output_img_.step = resize_img_info_.width * resize_img_info_.num_channels;
  output_img_.encoding = "rgb8";
  output_img_.is_bigendian = img->is_bigendian;

  img_pub_.publish( output_img_ );
}

ImageProcCUDA::ImageProcCUDA() : nh_(), pnh_( "~" ) {

  img_sub_ = nh_.subscribe( "/dash_center/image_raw", 1, &ImageProcCUDA::imageCallback, this );
  img_pub_ = pnh_.advertise<sensor_msgs::Image>( "/dash_center/image_proc", 1 );

  src_info_.width = 1440;
  src_info_.height = 928;
  src_info_.encoding = "bayer_rggb8";
  const auto num_channels_src = sensor_msgs::image_encodings::numChannels( src_info_.encoding );
  src_info_.step = src_info_.width * num_channels_src;

  color_img_info_.width = src_info_.width;
  color_img_info_.height = src_info_.height;
  color_img_info_.encoding = "rgb8";
  const auto num_channels_color = sensor_msgs::image_encodings::numChannels( color_img_info_.encoding );
  color_img_info_.step = color_img_info_.width * num_channels_color;

  img_debayer_.reset( new ips::Debayer( src_info_, color_img_info_ ) );

  undistort_img_info_ = color_img_info_;
  undistort_img_info_.focal.x = 1.0160135140574507e+03;
  undistort_img_info_.focal.y = 1.0228831191736832e+03;

  undistort_img_info_.principal.x = 7.1448986307993209e+02;
  undistort_img_info_.principal.y = 4.4984198523108546e+02;

  undistort_img_info_.k1 = -0.1770247952838815;
  undistort_img_info_.k2 = 0.10158718377719865;
  undistort_img_info_.p1 = 0.00011105436831281622;
  undistort_img_info_.p2 = -0.00013079579757543979;
  //  undistort_img_info_.k3 = -0.03281243814970146;
  img_undistort_.reset( new ips::Undistort( undistort_img_info_, undistort_img_info_ ) );

  resize_img_info_.width = color_img_info_.width / 3;
  resize_img_info_.height = color_img_info_.height / 3;
  resize_img_info_.encoding = "rgb8";
  const auto num_channels_resize = sensor_msgs::image_encodings::numChannels( resize_img_info_.encoding );
  resize_img_info_.step = resize_img_info_.width * num_channels_resize;

  img_resizer_.reset( new ips::Resizer( color_img_info_, resize_img_info_ ) );
}