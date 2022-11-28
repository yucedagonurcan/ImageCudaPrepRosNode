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
    Debayer(const int dstW, const int dstH) {
      m_dstW = dstW;
      m_dstH = dstH;

      m_dstSize.width = m_dstW;
      m_dstSize.height = m_dstH;

      m_dstROI = {0, 0, m_dstW, m_dstH};
    }

    bool m_Run(const sensor_msgs::Image &srcMsg, sensor_msgs::Image &dstMsg) {
      if (!m_InitialMember(srcMsg))
        return false;

      std::vector<uint8_t> color_img(m_dstStep * m_dstH, 0);

      {
        void *srcImage = nullptr, *dstImage = nullptr;

        if (!m_CudaAllocation(&srcImage, &dstImage)) return false;

        if (CheckCUDA(
                 cudaMemcpy2D(srcImage, m_srcStep,
                               &srcMsg.data[0], m_srcStep,
                               m_srcW * m_srcChannelNum, m_srcH, cudaMemcpyHostToDevice),
                 "cudaMemcpy2D") != 0) return false;

        if (!m_Processing(&srcImage, &dstImage)) return false;

        if (CheckCUDA(
                 cudaMemcpy2D(&color_img[0], m_dstStep,
                               dstImage, m_dstStep,
                               m_dstStep, m_dstH, cudaMemcpyDeviceToHost),
                 "cudaMemcpy2D") != 0) return false;

        nppiFree(srcImage);
        nppiFree(dstImage);
      }

      {
        dstMsg.header.frame_id = srcMsg.header.frame_id;
        dstMsg.width = m_dstW;
        dstMsg.height = m_dstH;
        dstMsg.step = m_dstStep;
        dstMsg.encoding = "rgb8";
        dstMsg.is_bigendian = srcMsg.is_bigendian;

        dstMsg.data = std::move(color_img);
      }

      return true;
    };

   protected:
    virtual bool m_InitialMember(const sensor_msgs::Image &srcMsg) {
      if ((srcMsg.width < 0) || (srcMsg.height < 0)){
        ROS_ERROR("[Debayer] Unvalid image size. check your image.");
        return false;
      }

      if ( srcMsg.encoding != "bayer_bggr8" &&
           srcMsg.encoding != "bayer_rggb8"){

        ROS_ERROR("[Debayer] %s is invalid encording format! Not supportted encording type.", srcMsg.encoding.c_str());
        return false;
      }

      m_srcW = srcMsg.width;
      m_srcH = srcMsg.height;
      m_srcSize.width = srcMsg.width;
      m_srcSize.height = srcMsg.height;
      m_srcROI = {0, 0, m_srcW, m_srcH};

      m_srcEncoding = srcMsg.encoding;
      m_srcChannelNum = sensor_msgs::image_encodings::numChannels(m_srcEncoding);

      m_srcStep = m_srcW * m_srcChannelNum;
      m_dstStep = m_dstW * m_srcChannelNum * 3;

      return true;
    };

    virtual bool m_CudaAllocation(void **src, void **dst) {
      int srcStep, dstStep;
      if (m_srcEncoding == "bayer_rggb8" || m_srcEncoding == "bayer_bggr8"){
        *src = nppiMalloc_8u_C1(m_srcW, m_srcH, &srcStep);
        *dst = nppiMalloc_8u_C3(m_dstW, m_dstH, &dstStep);
        return true;
      }
      else
        return false;
    };

    virtual bool m_Processing(void **src, void **dst) {

      if (m_srcEncoding == "bayer_bggr8"){
        if (CheckNPP(
                 nppiCFAToRGB_8u_C1C3R((Npp8u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                        (Npp8u *)*dst, m_dstStep, NPPI_BAYER_BGGR, NPPI_INTER_UNDEFINED),
                 "nppiCFAToRGB_8u_C1C3R") != 0) return false;

        return true;
      }
      else if (m_srcEncoding == "bayer_rggb8"){
        if (CheckNPP(
                 nppiCFAToRGB_8u_C1C3R((Npp8u *)*src, m_srcStep, m_srcSize, m_srcROI,
                                    (Npp8u *)*dst, m_dstStep, NPPI_BAYER_RGGB, NPPI_INTER_UNDEFINED),
                 "nppiCFAToRGB_8u_C1C3R") != 0) return false;
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
#endif//ROS_IMAGE_PROCESSING_INCLUDE_PROCESSORS_DEBAYER_H_
