/* Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros_image_processing.h"

#include <fstream>
#include <iostream>

#include <cuda_runtime.h>
#include <npp.h>

#include <helper_cuda.h>
#include <helper_string.h>


bool printfNPPinfo(int argc, char *argv[])
{
  const NppLibraryVersion *libVer = nppGetLibVersion();

  printf("NPP Library Version %d.%d.%d\n", libVer->major, libVer->minor,
         libVer->build);

  int driverVersion, runtimeVersion;
  cudaDriverGetVersion(&driverVersion);
  cudaRuntimeGetVersion(&runtimeVersion);

  printf("  CUDA Driver  Version: %d.%d\n", driverVersion / 1000,
         (driverVersion % 100) / 10);
  printf("  CUDA Runtime Version: %d.%d\n", runtimeVersion / 1000,
         (runtimeVersion % 100) / 10);

  // Min spec is SM 1.0 devices
  bool bVal = checkCudaCapabilities(1, 0);
  return bVal;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "my_npp");

  findCudaDevice(argc, (const char **)argv);
  if ( !printfNPPinfo( argc, argv ) )
    exit(EXIT_SUCCESS);


  printf("%s Starting...\n\n", argv[0]);
  ImageProcCUDA img_proc_cuda;

  ros::spin();

  try
  {
    std::string sFilename;
    char *filePath;




    if (checkCmdLineFlag(argc, (const char **)argv, "input"))
    {
      getCmdLineArgumentString(argc, (const char **)argv, "input", &filePath);
    }
    else
    {
      filePath = sdkFindFilePath("Lena.pgm", argv[0]);
    }

    if (filePath)
    {
      sFilename = filePath;
    }
    else
    {
      sFilename = "Lena.pgm";
    }

    // if we specify the filename at the command line, then we only test
    // sFilename[0].
    int file_errors = 0;
    std::ifstream infile(sFilename.data(), std::ifstream::in);

    if (infile.good())
    {
      std::cout << "boxFilterNPP opened: <" << sFilename.data()
                << "> successfully!" << std::endl;
      file_errors = 0;
      infile.close();
    }
    else
    {
      std::cout << "boxFilterNPP unable to open: <" << sFilename.data() << ">"
                << std::endl;
      file_errors++;
      infile.close();
    }

    if (file_errors > 0)
    {
      exit(EXIT_FAILURE);
    }

    std::string sResultFilename = sFilename;

    std::string::size_type dot = sResultFilename.rfind('.');

    if (dot != std::string::npos)
    {
      sResultFilename = sResultFilename.substr(0, dot);
    }

    sResultFilename += "_boxFilter.pgm";

    if (checkCmdLineFlag(argc, (const char **)argv, "output"))
    {
      char *outputFilePath;
      getCmdLineArgumentString(argc, (const char **)argv, "output",
                               &outputFilePath);
      sResultFilename = outputFilePath;
    }

    // declare a host image object for an 8-bit grayscale image
    npp::ImageCPU_8u_C1 oHostSrc;
    // load gray-scale image from disk
    npp::loadImage(sFilename, oHostSrc);
    // declare a device image and copy construct from the host image,
    // i.e. upload host to device
    npp::ImageNPP_8u_C1 oDeviceSrc(oHostSrc);

    // create struct with box-filter mask size
    NppiSize oSrcSize = {(int)oDeviceSrc.width(), (int)oDeviceSrc.height()};
    NppiPoint oSrcOffset = {0, 0};

    // create struct with ROI size
    NppiSize oSizeROI = {(int)400, (int)400};
    // allocate device image of appropriately reduced size
    npp::ImageNPP_8u_C1 oDeviceDst(oSizeROI.width, oSizeROI.height);
    // set anchor point inside the mask to (oMaskSize.width / 2,
    // oMaskSize.height / 2) It should round down when odd

    for(int i=0; i<3520; i+=1){    // run box filter

      NppiSize oMaskSize = {i%30, i%30};
      NppiPoint oAnchor = {oMaskSize.width / 2, oMaskSize.height / 2};


      NPP_CHECK_NPP(nppiFilterBoxBorder_8u_C1R(
          oDeviceSrc.data() + oDeviceSrc.pitch() * (i%350) + (i%350), oDeviceSrc.pitch(), oSrcSize, oSrcOffset,
          oDeviceDst.data(), oDeviceDst.pitch(), oSizeROI, oMaskSize, oAnchor,
          NPP_BORDER_REPLICATE));


      npp::ImageNPP_8u_C1 oDeviceRotDst(oSizeROI.width, oSizeROI.height);

      NppiRect rotRect = {0, 0, (int)oDeviceSrc.width(), (int)oDeviceSrc.height()};
      NPP_CHECK_NPP(nppiRotate_8u_C1R(
          oDeviceDst.data(), oSrcSize, oDeviceDst.pitch(), rotRect,
          oDeviceRotDst.data(), oDeviceRotDst.pitch(), rotRect, 45 - i%90, 0, 0, 1));

      // declare a host image for the result
      npp::ImageCPU_8u_C1 oHostDst(oDeviceRotDst.size());
      // and copy the device result data into it
      oDeviceRotDst.copyTo(oHostDst.data(), oHostDst.pitch());

      saveImage(sResultFilename, oHostDst);
      std::cout << "Saved image: " << sResultFilename << " " << i << std::endl;
      // usleep(10000);
    }

    nppiFree(oDeviceSrc.data());
    nppiFree(oDeviceDst.data());

    exit(EXIT_SUCCESS);
  }
  catch (npp::Exception &rException)
  {
    std::cerr << "Program error! The following exception occurred: \n";
    std::cerr << rException << std::endl;
    std::cerr << "Aborting." << std::endl;

    exit(EXIT_FAILURE);
  }
  catch (...)
  {
    std::cerr << "Program error! An unknow type of exception occurred. \n";
    std::cerr << "Aborting." << std::endl;

    exit(EXIT_FAILURE);
    return -1;
  }

  return 0;
}
