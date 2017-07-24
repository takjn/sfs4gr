/*
** tiny Point Cloud Library
**
** Copyright (c) 2017 Jun Takeda
**
** Permission is hereby granted, free of charge, to any person obtaining
** a copy of this software and associated documentation files (the
** "Software"), to deal in the Software without restriction, including
** without limitation the rights to use, copy, modify, merge, publish,
** distribute, sublicense, and/or sell copies of the Software, and to
** permit persons to whom the Software is furnished to do so, subject to
** the following conditions:
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
** CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
** TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
** SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
** [ MIT license: http://www.opensource.org/licenses/mit-license.php ]
*/

#include <bitset>
#include "mbed.h"
#include "SdUsbConnect.h"
#include "DisplayApp.h"
#include "tinypcl.hpp"
#include "camera_if.hpp"

// Extrinsic parameters of the camera (Depends on your enclosure design)
#define CAMERA_DISTANCE 115     // Distance from the origin to the camera (mm)
#define CAMERA_OFFSET  3        // Height offset of the camera relative to the origin (mm)

// Intrinsic parameters of the camera (cf. OpenCV's Camera Calibration)
#define CAMERA_CENTER_U 321     // Optical centers (cx)
#define CAMERA_CENTER_V 244     // Optical centers (cy)
#define CAMERA_FX 365.202395    // Focal length(fx)
#define CAMERA_FY 365.519979    // Focal length(fy)

// Stepper Motor Parameters (Depends on your stepper motor)
#define STEPPER_DIRECTION   1       // Direction (0 or 1)
#define STEPPER_WAIT        0.004   // Pulse duration
#define STEPPER_STEP_COUNTS 200     // A 200 step motor is the same as a 1.8 degrees motor 
#define STEPPER_STEP_RESOLUTIONS 4  // full-step = 1, half-step = 2, quarter-step = 4

// 復元関連のパラメーター
#define SILHOUETTE_COUNTS           40  // 復元で利用する画像の枚数 
#define SILHOUETTE_THRESH_BINARY    30  // 二値化する際のしきい値

// Defines pins numbers (Depends on your circuit design)
DigitalOut  a4988_step(D8);     // Connect the pin to A4988 step
DigitalOut  a4988_dir(D9);      // Connect the pin to A4988 dir
DigitalIn   button0(D4);        // Connect the pin to SW1
DigitalIn   button1(D6);        // Connect the pin to SW2
DigitalOut  led_ready(D5);      // Connect the pin to LED1 (ready)
DigitalOut  led_working(D7);    // Connect the pin to LED2 (working)
DigitalOut  led1(LED1);         // Use onboard LED for debugging purposes

// 復元関連のデータ
PointCloud point_cloud;      // 仮想物体（復元する点群）
cv::Mat img_background;      // 背景画像

int reconst_count = 1;       // 復元結果ファイルのインデックス
char file_name[32];          // 出力ファイル名
int file_name_index = 1;     // 出力ファイル名のインデックス

/* For viewing image on PC */
static DisplayApp  display_app;

// 3次元座標から2次元座標への変換
int projection(double rad, double Xw, double Yw,double Zw, int &u, int &v)
{
    // （仮想物体を）ワールド座標の原点を中心にY軸周りに回転
    double Xc= cos(rad)*Xw + sin(rad)*Zw;
    double Yc= Yw;
    double Zc=-sin(rad)*Xw + cos(rad)*Zw;

    // ワールド座標からカメラ座標への変換（Z軸方向の移動のみ）
    Yc+=CAMERA_OFFSET;
    Zc-=CAMERA_DISTANCE;
  
    // 画像座標へ変換
    u= CAMERA_CENTER_U - (int)((Xc/Zc)*(CAMERA_FX));
    v= CAMERA_CENTER_V - (int)((Yc/Zc)*(CAMERA_FY));

    return (u>0 && u<VIDEO_PIXEL_HW && v>0 && v<VIDEO_PIXEL_VW);
}

// 輪郭画像からの立体形状復元
// 3d reconstruction from silhouette
void reconst(double rad) {
    // Takes a video frame in grayscale (cf. camera_if.cpp)
    cv::Mat img_silhouette;
    create_gray(img_silhouette);

    // Background subtraction
    cv::absdiff(img_silhouette, img_background, img_silhouette);

    // Get a silhouette
    cv::threshold(img_silhouette, img_silhouette, SILHOUETTE_THRESH_BINARY, 255, cv::THRESH_BINARY);

    // Saves a silhouette image for dubugging purposes
    // sprintf(file_name, "/storage/img_%d.bmp", file_name_index);
    // cv::imwrite(file_name, img_silhouette);
    // printf("Saved file %s\r\n", file_name);

    // 輪郭画像による仮想物体の型抜き - Shape from silhouette
    // このプログラムでは、仮想物体の形状を点群(point cloud data)で表現している。
    // point_cloud_data(x,y,z) = 0の場合、そこには物体がないことを意味する。
    // point_cloud_data(x,y,z) = 1の場合、そこには物体がある（可能性がある）ことを意味する。
    // 型抜きとは、仮想物体の復元対象点ごとに、輪郭画像内外を判定し、輪郭画像外であれば除去、輪郭画像内であれば保持を繰り返すこと。
    // このプログラムでは、原点を中心にxyzそれぞれ-50mm ~ +50mmの範囲を1mm単位で復元する。
    double xx,yy,zz;    // 復元対象の点の座標値(x,y,z)
    int u,v;            // 復元対象の点の、カメラ画像内での座標値(x,y)
    int pcd_index=0;    // 仮想物体の復元対象点

    zz = (-point_cloud.POINTS / 2) * point_cloud.SCALE;
    for (int z=0; z<point_cloud.POINTS; z++, zz += point_cloud.SCALE) {

        yy = (-point_cloud.POINTS / 2) * point_cloud.SCALE;
        for (int y=0; y<point_cloud.POINTS; y++, yy += point_cloud.SCALE) {

            xx = (-point_cloud.POINTS / 2) * point_cloud.SCALE;
            for (int x=0; x<point_cloud.POINTS; x++, xx += point_cloud.SCALE, pcd_index++) {
                if (point_cloud.get(pcd_index) == 1) {
                    
                    // 復元対象の点がカメラ画像内ではどこにあるかを計算する
                    if (projection(rad, xx, yy, zz, u, v)) {
                        // カメラ画像内のため、輪郭画像と比較する
                        if (img_silhouette.at<unsigned char>(v, u)) {
                            // 復元対象の点は、輪郭画像内（白色）のため、そのまま
                        }
                        else {
                            // 復元対象の点は、輪郭画像外（黒色）のため、除去
                            point_cloud.set(pcd_index, 0);
                        }
                    } else {
                        // カメラ画像外のためクリアする
                        point_cloud.set(pcd_index, 0);
                    }
                }
            }
        }
    }
}

// Rotates a stepper motor with a A4988 stepper motor driver
void rotate(int steps) {
    a4988_dir = STEPPER_DIRECTION;
    for (int i=0;i<steps;i++) {
        a4988_step = 1;
        wait(STEPPER_WAIT);
        a4988_step = 0;
        wait(STEPPER_WAIT);
    }
}

int main() {
    // Starts camera
    camera_start();
    led1 = 1;

    // Connects SD & USB
    SdUsbConnect storage("storage");

    // Resets stepper motor
    a4988_dir = STEPPER_DIRECTION;
    a4988_step = 0;

    while (1) {
        storage.wait_connect();

        if (button0 == 0) {
            // Takes a video frame in grayscale and set as a background image
            led_working = 1;
            create_gray(img_background);

            // Saves a background image to a storage
            sprintf(file_name, "/storage/img_%d.jpg", file_name_index++);
            save_image_jpg(file_name); // save as jpeg
            printf("Saved file %s\r\n", file_name);

            led_working = 0;
            led_ready = 1;

            wait_ms(100);
        }
        if (button1 == 0 && !img_background.empty()) {
            // Shape from silhouette アルゴリズムによる立体形状復元
            // テーブルを回転させながら輪郭画像の取得と立体形状復元を繰り返す
            for (int i = 0; i < SILHOUETTE_COUNTS; i++) {
                // Send a preview image to PC
                size_t jpeg_size = create_jpeg();
                display_app.SendJpeg(get_jpeg_adr(), jpeg_size);

                // 輪郭画像の取得と立体形状復元
                led_working = 1;
                double rad = (double)(2 * 3.14159265258979)*((double)i / SILHOUETTE_COUNTS);
                reconst(rad);

                // Saves a preview image for dubugging purposes
                sprintf(file_name, "/storage/img_%d.jpg", file_name_index++);
                save_image_jpg(file_name); // save as jpeg
                printf("Saved file %s\r\n", file_name);

                led_working = 0;

                // Rotates the turn table
                rotate(STEPPER_STEP_COUNTS * STEPPER_STEP_RESOLUTIONS / SILHOUETTE_COUNTS);
            }

            // Clean the result
            point_cloud.remove_edge();

            // Save the result
            cout << "writting..." << endl;
            led_working = 1;

            sprintf(file_name, "/storage/result_%d.xyz", reconst_count);
            point_cloud.save_as_xyz(file_name);
            sprintf(file_name, "/storage/result_%d.stl", reconst_count);
            point_cloud.save_as_stl(file_name);
            // sprintf(file_name, "/storage/result_%d.ply", reconst_count);
            // point_cloud.save_as_ply(file_name);

            reconst_count++;

            led_working = 0;
            cout << "finish" << endl;
            point_cloud.clear();
        }

        // Send a preview image to PC
        size_t jpeg_size = create_jpeg();
        display_app.SendJpeg(get_jpeg_adr(), jpeg_size);
    }
}
