//
// Created by yucedagonurcan on 11/28/22.
//

#ifndef ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_DEBAYER_H_
#define ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_DEBAYER_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "processor.h"
#include "../ErrorCheck.h"

#include <npp.h>

namespace ips{

  class Debayer: public ImageProcessing{

   public:
    Debayer(const ImageInfo& src_info, const ImageInfo& dst_info) {

      src_info_ = src_info;
      dst_info_ = dst_info;

      m_srcROI = {0, 0, src_info_.width, src_info_.height};
      m_srcSize = { src_info_.width, src_info_.height };
    }

    bool validateProcessor() override{
      if (  src_info_.encoding != "bayer_rggb8"   &&
            src_info_.encoding != "bayer_bggr8"){

        ROS_ERROR("[Debayer] %s is invalid encoding format! Not supported encoding type.", src_info_.encoding.c_str());
        return false;
      }
    }

    bool m_Run(void** srcImage, void** dstImage) {
      if(validateProcessor()){
        ROS_ERROR("There is an error in validation of `Debayer` processor");
        exit(1);
      }
      if (!m_Processing(srcImage, dstImage)) return false;
      return true;
    }

   protected:
    bool m_Processing(void **src, void **dst) override {

      if (src_info_.encoding == "bayer_bggr8"){
        if (CheckNPP(
                 nppiCFAToRGB_8u_C1C3R((Npp8u *)*src, src_info_.step,
                                        m_srcSize,m_srcROI,
                                        (Npp8u *)*dst, dst_info_.step, NPPI_BAYER_BGGR, NPPI_INTER_UNDEFINED),
                 "nppiCFAToRGB_8u_C1C3R") != 0) return false;
        return true;
      }
      else if (src_info_.encoding == "bayer_rggb8"){
        if (CheckNPP(
                 nppiCFAToRGB_8u_C1C3R((Npp8u *)*src, src_info_.step,
                                        m_srcSize,m_srcROI,
                                    (Npp8u *)*dst, dst_info_.step, NPPI_BAYER_RGGB, NPPI_INTER_UNDEFINED),
                 "nppiCFAToRGB_8u_C1C3R") != 0) return false;
        return true;
       }
      else
        return false;
    };

   private:
    NppiRect m_srcROI{};
    NppiSize m_srcSize{};

  };
}
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_DEBAYER_H_
