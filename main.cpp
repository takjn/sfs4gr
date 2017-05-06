#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "SdUsbConnect.h"
#include "JPEG_Converter.h"
#include "dcache-control.h"
#include "opencv2/opencv.hpp"
#include "DisplayApp.h"

#define MOUNT_NAME             "storage"

#define DBG_PCMONITOR (0)
#if (DBG_PCMONITOR == 1)
/* For viewing image on PC */
static DisplayApp  display_app;
#endif


/* Video input and LCD layer 0 output */
#define VIDEO_FORMAT           (DisplayBase::VIDEO_FORMAT_YCBCR422)
#define GRAPHICS_FORMAT        (DisplayBase::GRAPHICS_FORMAT_YCBCR422)
#define WR_RD_WRSWA            (DisplayBase::WR_RD_WRSWA_32_16BIT)
#define DATA_SIZE_PER_PIC      (2u)

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
#define VIDEO_PIXEL_HW         (640u)  /* VGA */
#define VIDEO_PIXEL_VW         (480u)  /* VGA */

#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)

#if defined(__ICCARM__)
#pragma data_alignment=32
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]@ ".mirrorram";
#pragma data_alignment=4
#else
static uint8_t user_frame_buffer0[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((section("NC_BSS"),aligned(32)));
#endif
static int file_name_index = 1;
/* jpeg convert */
static JPEG_Converter Jcu;
#if defined(__ICCARM__)
#pragma data_alignment=32
static uint8_t JpegBuffer[1024 * 63];
#else
static uint8_t JpegBuffer[1024 * 63]__attribute((aligned(32)));
#endif

DisplayBase Display;
DigitalIn   button_shutter(D10);
DigitalIn   button0(USER_BUTTON0);
DigitalIn   button1(USER_BUTTON1);
DigitalOut  led1(LED1);
DigitalOut  led2(LED2);
DigitalOut  led3(LED3);
DigitalOut  led4(LED4);

static cv::Mat intrinsic, distortion;
  
static void save_image_bmp(void) {
    // Transform buffer into OpenCV Mat
    cv::Mat img_yuv(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC2, user_frame_buffer0);

    // Convert from YUV422 to grayscale
    cv::Mat img_gray;
    cv::cvtColor(img_yuv, img_gray, CV_YUV2GRAY_YUY2);

    cv::Mat img_gray_dst;
    cv::undistort(img_gray, img_gray_dst, intrinsic, distortion);

    char file_name[32];
    sprintf(file_name, "/"MOUNT_NAME"/img_%d.bmp", file_name_index);
    cv::imwrite(file_name, img_gray);
    printf("Saved file %s\r\n", file_name);

    sprintf(file_name, "/"MOUNT_NAME"/dst_%d.bmp", file_name_index);
    cv::imwrite(file_name, img_gray_dst);
    printf("Saved file %s\r\n", file_name);
}

static void save_image_jpg(void) {
    size_t jcu_encode_size;
    JPEG_Converter::bitmap_buff_info_t bitmap_buff_info;
    JPEG_Converter::encode_options_t   encode_options;

    bitmap_buff_info.width              = VIDEO_PIXEL_HW;
    bitmap_buff_info.height             = VIDEO_PIXEL_VW;
    bitmap_buff_info.format             = JPEG_Converter::WR_RD_YCbCr422;
    bitmap_buff_info.buffer_address     = (void *)user_frame_buffer0;

    encode_options.encode_buff_size     = sizeof(JpegBuffer);
    encode_options.p_EncodeCallBackFunc = NULL;
    encode_options.input_swapsetting    = JPEG_Converter::WR_RD_WRSWA_32_16_8BIT;

    jcu_encode_size = 0;
    dcache_invalid(JpegBuffer, sizeof(JpegBuffer));
    if (Jcu.encode(&bitmap_buff_info, JpegBuffer, &jcu_encode_size, &encode_options) != JPEG_Converter::JPEG_CONV_OK) {
        jcu_encode_size = 0;
    }

    char file_name[32];
    sprintf(file_name, "/"MOUNT_NAME"/img_%d.jpg", file_name_index);
    FILE * fp = fopen(file_name, "w");
    fwrite(JpegBuffer, sizeof(char), (int)jcu_encode_size, fp);
    fclose(fp);
    printf("Saved file %s\r\n", file_name);
}

static void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)user_frame_buffer0,
        FRAME_BUFFER_STRIDE,
        VIDEO_FORMAT,
        WR_RD_WRSWA,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}


size_t encode_jpeg(uint8_t* buf, int len, int width, int height, uint8_t* inbuf) {
    size_t encode_size;
    JPEG_Converter::bitmap_buff_info_t bitmap_buff_info;
    JPEG_Converter::encode_options_t encode_options;
    bitmap_buff_info.width = width;
    bitmap_buff_info.height = height;
    bitmap_buff_info.format = JPEG_Converter::WR_RD_YCbCr422;
    bitmap_buff_info.buffer_address = (void *) inbuf;
    encode_options.encode_buff_size = len;
    encode_options.p_EncodeCallBackFunc = NULL;
    encode_options.input_swapsetting = JPEG_Converter::WR_RD_WRSWA_32_16_8BIT;

    encode_size = 0;
    dcache_invalid(buf, len);
    if (Jcu.encode(&bitmap_buff_info, buf, &encode_size, &encode_options)
            != JPEG_Converter::JPEG_CONV_OK) {
        encode_size = 0;
    }

    return encode_size;
}

size_t create_jpeg(){
    return encode_jpeg(JpegBuffer, sizeof(JpegBuffer), VIDEO_PIXEL_HW, VIDEO_PIXEL_VW, user_frame_buffer0);
}

uint8_t* get_jpeg_adr(){
    return JpegBuffer;
}


// harf step (400 steps)
#define STEPPER_WAIT 0.004

DigitalOut A4988STEP(D8);
DigitalOut A4988DIR(D9);

int setup() {
    A4988DIR = 0;
    A4988STEP = 0;

    // cv::FileStorage fs("/storage/camera.xml", cv::FileStorage::READ);
    // if (!fs.isOpened()){
    //     cout << "camera.xml open error" << endl;
    //     return -1;
    // }

    // fs["intrinsic"] >> intrinsic;
    // fs["distortion"] >> distortion;
    // fs.release();

    // cout << "camera matrix: " << intrinsic << endl
    //      << "distortion coeffs: " << distortion << endl;
    intrinsic = (cv::Mat_<double>(3,3) << 367.879585, 0.000000, 314.035869, 0.000000, 367.582735, 234.664545, 0.000000, 0.000000, 1.000000);
    distortion = (cv::Mat_<double>(1,4) << -0.333848, 0.165991, 0.000608, -0.001805 );

    return 0;
}

void rotate(int steps) {
    A4988DIR = 1;
    if (steps < 0) {
        A4988DIR = 0;
        steps = -steps;
    }

    for (int i=0;i<steps;i++) {
        led2=1;
        A4988STEP = 1;
        wait(STEPPER_WAIT);
        led2=0;
        A4988STEP = 0;
        wait(STEPPER_WAIT);
    }
}

int main() {
    // Initialize the background to black
    for (int i = 0; i < sizeof(user_frame_buffer0); i += 2) {
        user_frame_buffer0[i + 0] = 0x10;
        user_frame_buffer0[i + 1] = 0x80;
    }

    // Camera
    EasyAttach_Init(Display);
    Start_Video_Camera();
    led4 = 1;

    // SD & USB
    SdUsbConnect storage(MOUNT_NAME);

    storage.wait_connect();
    setup();
    led3 = 1;

    button_shutter.mode(PullUp);

    while (1) {
        storage.wait_connect();
        // if (button_shutter == 0) {
        //     led1 = 1;
        //     save_image_bmp(); // save as bitmap
        //     save_image_jpg(); // save as jpeg
        //     led1 = 0;
        // }

        if (button0 == 0) {
            led1 = 1;
            save_image_bmp(); // save as bitmap
            save_image_jpg(); // save as jpeg
            led1 = 0;
            file_name_index++;
        }
        if (button1 == 0) {
            for (int i=0;i<80;i++) {
                led1 = 1;
                save_image_bmp(); // save as bitmap
                save_image_jpg(); // save as jpeg
                led1 = 0;
                rotate(10);
                file_name_index++;
            }
        }

#if (DBG_PCMONITOR == 1)
        size_t jpeg_size = create_jpeg();
        display_app.SendJpeg(get_jpeg_adr(), jpeg_size);
#endif

    }
}