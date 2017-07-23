/* Copyright (c) 2017 Gnomons Vietnam Co., Ltd., Gnomons Co., Ltd.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CAMERA_IF_HPP_
#define CAMERA_IF_HPP_

#include "mbed.h"
#include "DisplayBace.h"
#include "opencv.hpp"
#include "EasyAttach_CameraAndLCD.h"

/* Video input and LCD layer 0 output */
#define VIDEO_FORMAT           (DisplayBase::VIDEO_FORMAT_YCBCR422)
#define GRAPHICS_FORMAT        (DisplayBase::GRAPHICS_FORMAT_YCBCR422)
#define WR_RD_WRSWA            (DisplayBase::WR_RD_WRSWA_32_16BIT)
#define DATA_SIZE_PER_PIC      (2u)

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#if MBED_CONF_APP_LCD
  #define VIDEO_PIXEL_HW       LCD_PIXEL_WIDTH
  #define VIDEO_PIXEL_VW       LCD_PIXEL_HEIGHT
  #if (MBED_CONF_APP_LCD_TYPE == GR_PEACH_4_3INCH_SHIELD) || (MBED_CONF_APP_LCD_TYPE == GR_LYCHEE_TF043HV001A0)
  #define ASPECT_RATIO_16_9    (1)
  #endif
#else
  #define VIDEO_PIXEL_HW       (640u)  /* QVGA */
  #define VIDEO_PIXEL_VW       (480u)  /* QVGA */
#endif

#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)


/**
* @brief	Starts the camera
* @param	None
* @return	None
*/
void camera_start(void);

/**
* @brief	Create jpeg from yuv image
* @param	None
* @return	jpeg size
*/
size_t create_jpeg();

/**
* @brief	Return jpeg addresse
* @param	None
* @return	jpeg address
*/
uint8_t* get_jpeg_adr();

/**
* @brief	Takes a video frame (in grayscale)
* @param	img_gray	Grayscale video frame
* @return	None
*/
void create_gray(cv::Mat &img_gray);

/**
* @brief	Save jpeg to storage
* @param	file_name	name of file
* @return	None
*/
void save_image_jpg(const char* file_name);

#if MBED_CONF_APP_LCD
void ClearSquare(void);
void DrawSquare(int x, int y, int w, int h, uint32_t const colour);
#endif

#endif
