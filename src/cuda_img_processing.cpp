//
// Created by yucedagonurcan on 11/28/22.
//
#include "cuda_img_processing.h"

void ImageProcCUDA::populateCameraInfo(ImageInfo& in_info){

  in_info.focal.x = 1.0160135140574507e+03;
  in_info.focal.y = 1.0228831191736832e+03;

  in_info.principal.x = 7.1448986307993209e+02;
  in_info.principal.y = 4.4984198523108546e+02;

  in_info.k1 = -0.1770247952838815;
  in_info.k2 = 0.10158718377719865;
  in_info.p1 = 0.00011105436831281622;
  in_info.p2 = -0.00013079579757543979;
}


void ImageProcCUDA::imageCallback( const sensor_msgs::Image::ConstPtr &img ) {

  if ( !collected_image_info ) {

    // SEC SOURCE CONFIG
    src_info_.width = img->width;
    src_info_.height = img->height;
    src_info_.encoding = img->encoding;
    src_info_.num_channels = sensor_msgs::image_encodings::numChannels( src_info_.encoding );
    src_info_.step = src_info_.width * src_info_.num_channels;
    if(! ips::CheckCudaMalloc(alloc_utils::imageCudaAlloc( &src_ptr_, src_info_ ), "Source")){
      exit( 1 );
    }


    // SEC DEBAYER CONFIG
    color_img_info_.width = src_info_.width;
    color_img_info_.height = src_info_.height;
    color_img_info_.encoding = "rgb8";
    color_img_info_.num_channels = sensor_msgs::image_encodings::numChannels( color_img_info_.encoding );
    color_img_info_.step = color_img_info_.width * color_img_info_.num_channels;
    img_debayer_.reset( new ips::Debayer( src_info_, color_img_info_ ) );
    if(! ips::CheckCudaMalloc(alloc_utils::imageCudaAlloc( &color_ptr_, color_img_info_ ), "Debayer")){
      exit( 1 );
    }

    // SEC UNDISTORT CONFIG
    undistort_img_info_ = color_img_info_;
    populateCameraInfo(undistort_img_info_);
    undistort_img_info_.num_channels = sensor_msgs::image_encodings::numChannels( undistort_img_info_.encoding );
    img_undistort_.reset( new ips::Undistort( undistort_img_info_, undistort_img_info_ ) );

    if(! ips::CheckCudaMalloc(alloc_utils::imageCudaAlloc( &undistort_ptr_, undistort_img_info_ ), "Undistort")){
      exit( 1 );
    }

    // SEC RESIZE CONFIG
    resize_img_info_.width = static_cast<int>(color_img_info_.width * resize_ratio_);
    resize_img_info_.height = static_cast<int>(color_img_info_.height * resize_ratio_);
    resize_img_info_.encoding = "rgb8";
    resize_img_info_.num_channels = sensor_msgs::image_encodings::numChannels( resize_img_info_.encoding );
    resize_img_info_.step = resize_img_info_.width * resize_img_info_.num_channels;
    img_resizer_.reset( new ips::Resizer( color_img_info_, resize_img_info_ ) );

    if(! ips::CheckCudaMalloc(alloc_utils::imageCudaAlloc( &resized_ptr_, resize_img_info_ ), "Resize")){
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

  output_img_.header = img->header;
  output_img_.width = resize_img_info_.width;
  output_img_.height = resize_img_info_.height;
  output_img_.step = resize_img_info_.width * resize_img_info_.num_channels;
  output_img_.encoding = "rgb8";
  output_img_.is_bigendian = img->is_bigendian;

  img_pub_.publish( output_img_ );
}

ImageProcCUDA::ImageProcCUDA() : nh_(), pnh_( "~" ) {

  img_sub_ = nh_.subscribe( "input/image_raw", 1, &ImageProcCUDA::imageCallback, this );
  img_pub_ = nh_.advertise<sensor_msgs::Image>( "output/image_raw", 1 );

  pnh_.param<bool>("enable/debayer", enable_debayer, true);
  pnh_.param<bool>("enable/undistort", enable_undistort, true);
  pnh_.param<bool>("enable/resize", enable_resize, true);

  pnh_.param<float>("resize_ratio", resize_ratio_, 0.3f);

}