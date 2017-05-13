#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "SdUsbConnect.h"
#include "JPEG_Converter.h"
#include "dcache-control.h"
#include "opencv2/opencv.hpp"
#include "DisplayApp.h"

// 筐体に依存するパラメーター
#define CAMERA_DISTANCE 110     // 原点(ステッピングモーター回転軸)からカメラの距離(mm)

// ステッピングモーターの出力ピン(ステッピングモータードライバとしてA4988を利用)
DigitalOut a4988_step(D8);
DigitalOut a4988_dir(D9);

// ステッピングモーター関連のパラメーター
#define STEPPER_WAIT    0.004   // wait
#define STEPPER_STEPS   800     // 1周に必要なステップ数（Quarter step）
#define STEPPER_STEP    20      // 1回のステップ数

// カメラ内部パラメーター（OpenCVのカメラキャリブレーションが必要）
#define CAMERA_CENTER_U 314     // 画像中心(横方向)
#define CAMERA_CENTER_V 234     // 画像中心（縦方向）
#define CAMERA_FX 367.879585    // カメラ焦点距離(fx)
#define CAMERA_FY 367.879585    // カメラ焦点距離(fy)

// 復元関連のパラメーター
#define SILHOUETTE_NOISE_THRESHOLD 3    // 欠損ノイズとみなすしきい値
#define PCD_POINTS 100                  // 復元する空間範囲(mm)

// 復元関連のデータ
static unsigned char point_cloud_data[PCD_POINTS][PCD_POINTS][PCD_POINTS]; // 仮想物体（復元する点群）
static cv::Mat img_silhouette;      // 輪郭画像
static cv::Mat img_background;      // 背景画像
static bool has_background = false; // 背景画像を取得済かどうかを管理するフラグ
static int reconst_count = 1;       // 復元結果ファイルのインデックス
static char file_name[32];          // 出力ファイル名

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
DigitalIn   button0(USER_BUTTON0);
DigitalIn   button1(USER_BUTTON1);
DigitalOut  led1(LED1);
DigitalOut  led2(LED2);
DigitalOut  led3(LED3);
DigitalOut  led4(LED4);

// 背景画像の取得
void get_background_image(void) {
    // Transform buffer into OpenCV Mat
    cv::Mat img_yuv(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC2, user_frame_buffer0);

    // Convert from YUV422 to grayscale
    cv::cvtColor(img_yuv, img_background, CV_YUV2GRAY_YUY2);

    // set flag
    has_background = true;
    led3 = 1;
}

// 3次元座標から2次元座標への変換
int projection(double rad, double Xw, double Yw,double Zw, int &u, int &v)
{
    // （仮想物体を）ワールド座標の原点を中心にY軸周りに回転
    double Xc= cos(rad)*Xw + sin(rad)*Zw;
    double Yc= Yw;
    double Zc=-sin(rad)*Xw + cos(rad)*Zw;

    // ワールド座標からカメラ座標への変換（Z軸方向の移動のみ）
    Zc-=CAMERA_DISTANCE;
  
    // 画像座標へ変換
    u= CAMERA_CENTER_U - (int)((Xc/Zc)*(CAMERA_FX));
    v= CAMERA_CENTER_V - (int)((Yc/Zc)*(CAMERA_FY));

    return (u<0 || u>VIDEO_PIXEL_HW || v<0 || v>VIDEO_PIXEL_VW);
}

// 輪郭画像からの立体形状復元
// 3d reconstruction from silhouette
void reconst(double rad) {
    // Transform buffer into OpenCV Mat
    cv::Mat img_yuv(VIDEO_PIXEL_VW, VIDEO_PIXEL_HW, CV_8UC2, user_frame_buffer0);

    // Convert from YUV422 to grayscale
    cv::cvtColor(img_yuv, img_silhouette, CV_YUV2GRAY_YUY2);

    // 背景画像の除去と輪郭画像の取得
    // Remove background and get silhouette
    cv::Mat temp;
    cv::absdiff(img_silhouette, img_background, img_silhouette);
    cv::threshold(img_silhouette, img_silhouette, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // レンズ歪みの除去 （GR-LYCHEEのメモリ不足により実行不可）
    // Undistort (could not undistort because of out of memory on GR-LYCHEE)
    // cv::Mat intrinsic, distortion;
    // intrinsic = (cv::Mat_<double>(3,3) << 367.879585, 0.000000, 314.035869, 0.000000, 367.582735, 234.664545, 0.000000, 0.000000, 1.000000);
    // distortion = (cv::Mat_<double>(1,4) << -0.333848, 0.165991, 0.000608, -0.001805);
    // cv::undistort(temp, img_silhouette, intrinsic, distortion);

    // 輪郭画像の出力（デバッグ用だが、region RAM overflowed with Heapが発生するためコメントアウト）
    // sprintf(file_name, "/"MOUNT_NAME"/img_%d.bmp", file_name_index);
    // cv::imwrite(file_name, img_silhouette);
    // printf("Saved file %s\r\n", file_name);

    // 輪郭画像による仮想物体の型抜き - Shape from silhouette
    // このプログラムでは、仮想物体の形状を点群(point cloud data)で表現している。
    // point_cloud_data[x][y][z] = 0の場合、そこには物体がないことを意味する。
    // point_cloud_data[x][y][z] > 0の場合、そこには物体がある（可能性がある）ことを意味する。数字が大きいほど、可能性が高い。
    // 型抜きとは、仮想物体の復元対象点ごとに、輪郭画像内外を判定し、輪郭画像外であれば除去、輪郭画像内であれば保持を繰り返すこと。
    // このプログラムでは、原点を中心にxyzそれぞれ-50mm〜50mmの範囲を1mm単位で復元する。
    int xx,yy,zz;   // 復元対象の点の座標値(x,y,z)
    int u,v;        // 復元対象の点の、カメラ画像内での座標値(x,y)
    for (int z=0; z<PCD_POINTS; z++) {
        for (int y=0; y<PCD_POINTS; y++) {
            for (int x=0; x<PCD_POINTS; x++) {
                if (point_cloud_data[x][y][z] > 0) {
                    // 復元する点ごとに、輪郭画像内外を判定する
                    // 復元対象の点の座標値の計算
                    xx = (x - PCD_POINTS / 2);
                    yy = (y - PCD_POINTS / 2);
                    zz = (z - PCD_POINTS / 2);

                    // 復元対象の点がカメラ画像内ではどこにあるかを計算する
                    if (!projection(rad, xx, yy, zz, u, v)) {
                        // カメラ画像内のため、輪郭画像と比較する
                        if (!img_silhouette.at<unsigned char>(v, u)) {
                            // 復元対象の点は、輪郭画像外（黒色）のため、除去
                            point_cloud_data[x][y][z]--;
                        }
                    } else {
                        // カメラ画像外のためクリアする
                        point_cloud_data[x][y][z]=0;
                    }
                }
            }
        }
    }
}

// 点群データの初期化
void clear_point_cloud_data() {
    for (int z=0;z<PCD_POINTS;z++) {
        for (int y=0;y<PCD_POINTS;y++) {
            for (int x=0;x<PCD_POINTS;x++) {
                point_cloud_data[x][y][z]=SILHOUETTE_NOISE_THRESHOLD;
            }
        }
    }
}

// ステッピングモーターの回転(ステッピングモータードライバとしてA4988を利用)
void rotate(int steps) {
    a4988_dir = 1;
    for (int i=0;i<steps;i++) {
        led2=1;
        a4988_step = 1;
        wait(STEPPER_WAIT);
        led2=0;
        a4988_step = 0;
        wait(STEPPER_WAIT);
    }
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

    // Stepping motor
    a4988_dir = 0;
    a4988_step = 0;

    // Point cloud data
    clear_point_cloud_data();

    while (1) {
        storage.wait_connect();

        if (button0 == 0) {
            // 背景画像の取得
            led1 = 1;
            get_background_image(); // get background image
            led1 = 0;
        }
        if (button1 == 0 && has_background) {
            // Shape from silhouette アルゴリズムによる立体形状復元
            // テーブルを回転させながら輪郭画像の取得と立体形状復元を繰り返す
            for (int i=0;i<(STEPPER_STEPS/STEPPER_STEP);i++) {
                // 輪郭画像の取得と立体形状復元を繰り返す
                led1 = 1;
                double rad = (double)(2*3.14)*((double)i/(STEPPER_STEPS/STEPPER_STEP));
                reconst(rad);
                save_image_jpg(); // save as jpeg
                led1 = 0;
                file_name_index++;

                // テーブルの回転
                rotate(STEPPER_STEP);
            }

            // 復元した立体形状データ(Point Cloud Data)の出力
            cout << "writting..." << endl;
            led1 = 1;
            sprintf(file_name, "/"MOUNT_NAME"/result_%d.xyz", reconst_count++);
            FILE * fp = fopen(file_name, "w");
            for (int z=1; z<PCD_POINTS-1; z++) {
                for (int y=1; y<PCD_POINTS-1; y++) {
                    for (int x=1; x<PCD_POINTS-1; x++) {
                        if (point_cloud_data[x][y][z] > 0) {

                            // 物体内部の点は出力しない
                            int count = 0;
                            if (point_cloud_data[x-1][y-1][z-1]==0) count++;
                            if (point_cloud_data[x-1][y  ][z-1]==0) count++;
                            if (point_cloud_data[x-1][y+1][z-1]==0) count++;
                            if (point_cloud_data[x  ][y-1][z-1]==0) count++;
                            if (point_cloud_data[x  ][y  ][z-1]==0) count++;
                            if (point_cloud_data[x+1][y+1][z-1]==0) count++;
                            if (point_cloud_data[x+1][y-1][z-1]==0) count++;
                            if (point_cloud_data[x+1][y  ][z-1]==0) count++;
                            if (point_cloud_data[x+1][y+1][z-1]==0) count++;

                            if (point_cloud_data[x-1][y-1][z  ]==0) count++;
                            if (point_cloud_data[x-1][y  ][z  ]==0) count++;
                            if (point_cloud_data[x-1][y+1][z  ]==0) count++;
                            if (point_cloud_data[x  ][y-1][z  ]==0) count++;
                            // if (point_cloud_data[x  ][y  ][z  ]==0) count++;
                            if (point_cloud_data[x+1][y+1][z  ]==0) count++;
                            if (point_cloud_data[x+1][y-1][z  ]==0) count++;
                            if (point_cloud_data[x+1][y  ][z  ]==0) count++;
                            if (point_cloud_data[x+1][y+1][z  ]==0) count++;

                            if (point_cloud_data[x-1][y-1][z+1]==0) count++;
                            if (point_cloud_data[x-1][y  ][z+1]==0) count++;
                            if (point_cloud_data[x-1][y+1][z+1]==0) count++;
                            if (point_cloud_data[x  ][y-1][z+1]==0) count++;
                            if (point_cloud_data[x  ][y  ][z+1]==0) count++;
                            if (point_cloud_data[x+1][y+1][z+1]==0) count++;
                            if (point_cloud_data[x+1][y-1][z+1]==0) count++;
                            if (point_cloud_data[x+1][y  ][z+1]==0) count++;
                            if (point_cloud_data[x+1][y+1][z+1]==0) count++;

                            if (count>1) fprintf(fp,"%d -%d %d\n", x, y, z);
                        }
                    }
                }
            }
            fclose(fp);
            led1 = 0;
            cout << "finish" << endl;
            clear_point_cloud_data();
        }

#if (DBG_PCMONITOR == 1)
        size_t jpeg_size = create_jpeg();
        display_app.SendJpeg(get_jpeg_adr(), jpeg_size);
#endif

    }
}