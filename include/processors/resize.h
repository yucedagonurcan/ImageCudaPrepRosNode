//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_RESIZE_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_RESIZE_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "processor.h"
#include "../ErrorCheck.h"

#include <npp.h>
#include "alloc_utils.h"

namespace ips{

  class Resizer: public ImageProcessing{

   public:
    Resizer(const ImageInfo& src_info, const ImageInfo& dst_info) {

      src_info_ = src_info;
      dst_info_ = dst_info;

      m_srcROI = {0, 0, src_info_.width, src_info_.height};
      m_srcSize = { src_info_.width, src_info_.height };

      m_dstSize = { dst_info_.width, dst_info_.height };
      m_dstROI = {0, 0, dst_info_.width, dst_info_.height};

    }


    bool validateProcessor() override{

      if(src_info_.encoding != dst_info_.encoding){
        ROS_ERROR("[Resizer] IN[%s] and OUT[%s] encoding is not compatible for `Resizer`.", src_info_.encoding.c_str(), dst_info_.encoding.c_str());
        return false;
      }

      if ( src_info_.encoding != "rgb8"   &&
           src_info_.encoding != "rgb16"  &&
           src_info_.encoding != "rgba8"  &&
           src_info_.encoding != "rgba16" &&
           src_info_.encoding != "bgr8"   &&
           src_info_.encoding != "bgr16"  &&
           src_info_.encoding != "bgra8"  &&
           src_info_.encoding != "bgra16" &&
           src_info_.encoding != "mono8"  &&
           src_info_.encoding != "mono16"){

        ROS_ERROR("[Resizer] %s is invalid encoding format! Not supported encoding type.", src_info_.encoding.c_str());
        return false;
      }
    }

    bool m_Run(void** srcImage, void** dstImage) {
      if(validateProcessor()){
        ROS_ERROR("There is an error in validation of `Resizer` processor");
        exit(1);
      }
      if (!m_Processing(srcImage, dstImage)) return false;
      return true;
    }

   protected:

    virtual bool m_Processing(void **src, void **dst) {
      if (src_info_.encoding == "mono8"){
        if (CheckNPP(
                 nppiResize_8u_C1R((Npp8u *)*src, src_info_.step, m_srcSize, m_srcROI,
                                    (Npp8u *)*dst, dst_info_.step, m_dstSize, m_dstROI, NPPI_INTER_LINEAR),
                 "nppiResize_8u_C1R") != 0) return false;

        return true;
      }
      else if (src_info_.encoding == "mono16"){
        if (CheckNPP(
                 nppiResize_16u_C1R((Npp16u *)*src, src_info_.step, m_srcSize, m_srcROI,
                                     (Npp16u *)*dst, dst_info_.step, m_dstSize, m_dstROI, NPPI_INTER_LINEAR),
                 "nppiResize_16u_C1R") != 0) return false;

        return true;
      }
      else if (src_info_.encoding == "rgb8" || src_info_.encoding == "bgr8"){
        if (CheckNPP(
                 nppiResize_8u_C3R((Npp8u *)*src, src_info_.step, m_srcSize, m_srcROI,
                                    (Npp8u *)*dst, dst_info_.step, m_dstSize, m_dstROI, NPPI_INTER_LINEAR),
                 "nppiResize_8u_C3R") != 0) return false;

        return true;
      }
      else if (src_info_.encoding == "rgb16" || src_info_.encoding == "bgr16"){
        if (CheckNPP(
                 nppiResize_16u_C3R((Npp16u *)*src, src_info_.step, m_srcSize, m_srcROI,
                                     (Npp16u *)*dst, dst_info_.step, m_dstSize, m_dstROI, NPPI_INTER_LINEAR),
                 "nppiResize_16u_C3R") != 0) return false;

        return true;
      }
      else if (src_info_.encoding == "rgba8" || src_info_.encoding == "bgra8"){
        if (CheckNPP(
                 nppiResize_8u_C4R((Npp8u *)*src, src_info_.step, m_srcSize, m_srcROI,
                                    (Npp8u *)*dst, dst_info_.step, m_dstSize, m_dstROI, NPPI_INTER_LINEAR),
                 "nppiResize_8u_C4R") != 0) return false;

        return true;
      }
      else if (src_info_.encoding == "rgba16" || src_info_.encoding == "bgra16"){
        if (CheckNPP(
                 nppiResize_16u_C4R((Npp16u *)*src, src_info_.step, m_srcSize, m_srcROI,
                                     (Npp16u *)*dst, dst_info_.step, m_dstSize, m_dstROI, NPPI_INTER_LINEAR),
                 "nppiResize_16u_C4R") != 0) return false;

        return true;
      }
      else
        return false;
    };

   private:
    NppiSize m_srcSize{}, m_dstSize{};
    NppiRect m_srcROI{}, m_dstROI{};

  };
}
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_RESIZE_H_
