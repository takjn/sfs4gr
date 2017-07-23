#include <bitset>
#include "mbed.h"
#include "SdUsbConnect.h"
#include "DisplayApp.h"
#include "tinypcl.hpp"
#include "camera_if.hpp"

// 筐体に依存するパラメーター
#define CAMERA_DISTANCE 115     // 原点(ステッピングモーター回転軸)からカメラの距離(mm)
#define CAMERA_OFFSET  3        // カメラ高さの調整(mm)

// ステッピングモーターの出力ピン(ステッピングモータードライバとしてA4988を利用)
DigitalOut a4988_step(D8);
DigitalOut a4988_dir(D9);

// ボタンの入力ピン
DigitalIn   button0(D4);
DigitalIn   button1(D6);

// 状態表示LEDの出力ピン
DigitalOut  led_working(D7);    // 処理中
DigitalOut  led_ready(D5);      // 背景画像取得完了

// デバッグ用（カメラが準備完了になった時にLED1を点灯）
DigitalOut  led1(LED1);

// ステッピングモーター関連のパラメーター
#define STEPPER_DIRECTION 1     // ステッピングモーターの回転方向(0 または 1)
#define STEPPER_WAIT    0.004   // wait
#define STEPPER_STEPS   800     // 1周に必要なステップ数（Quarter step）
#define STEPPER_STEP    20      // 1回のステップ数

// カメラ内部パラメーター（OpenCVのカメラキャリブレーションが必要）
#define CAMERA_CENTER_U 321     // 画像中心(横方向)
#define CAMERA_CENTER_V 244     // 画像中心（縦方向）
#define CAMERA_FX 365.202395    // カメラ焦点距離(fx)
#define CAMERA_FY 365.519979    // カメラ焦点距離(fy)

// 復元関連のパラメーター
#define SILHOUETTE_THRESH_BINARY 30     // 二値化する際のしきい値

// 復元関連のデータ
PointCloud point_cloud;      // 仮想物体（復元する点群）

cv::Mat img_background;      // 背景画像
bool has_background = false; // 背景画像を取得済かどうかを管理するフラグ
int reconst_count = 1;       // 復元結果ファイルのインデックス
char file_name[32];          // 出力ファイル名
int file_name_index = 1;     // 出力ファイル名のインデックス

#define MOUNT_NAME             "storage"

#define DBG_PCMONITOR (1)
#if (DBG_PCMONITOR == 1)
/* For viewing image on PC */
static DisplayApp  display_app;
#endif

// 背景画像の取得
void get_background_image(void) {
    // Takes a video frame in grayscale(see camera_if.cpp)
    create_gray(img_background);

    // set flag
    has_background = true;
    led_ready = 1;
}

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
    // Takes a video frame in grayscale(see camera_if.cpp)
    cv::Mat img_silhouette;
    create_gray(img_silhouette);

    // 背景画像の除去と輪郭画像の取得
    // Background subtraction and get silhouette
    cv::absdiff(img_silhouette, img_background, img_silhouette);
    cv::threshold(img_silhouette, img_silhouette, SILHOUETTE_THRESH_BINARY, 255, cv::THRESH_BINARY);

    // 輪郭画像の出力（デバッグ用）
    // sprintf(file_name, "/"MOUNT_NAME"/img_%d.bmp", file_name_index);
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

// ステッピングモーターの回転(ステッピングモータードライバとしてA4988を利用)
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
    // Camera
    camera_start();
    led1 = 1;

    // SD & USB
    SdUsbConnect storage(MOUNT_NAME);

    // Stepping motor
    a4988_dir = 0;
    a4988_step = 0;

    // clear Point cloud data
    point_cloud.clear();

    while (1) {
        storage.wait_connect();

        if (button0 == 0) {
            // 背景画像の取得
            led_working = 1;
            get_background_image(); // get background image

            // 取得した背景画像の保存
            sprintf(file_name, "/"MOUNT_NAME"/img_%d.jpg", file_name_index++);
            save_image_jpg(file_name); // save as jpeg
            printf("Saved file %s\r\n", file_name);

            led_working = 0;

            wait_ms(100);
        }
        if (button1 == 0 && has_background) {
            // Shape from silhouette アルゴリズムによる立体形状復元
            // テーブルを回転させながら輪郭画像の取得と立体形状復元を繰り返す
            for (int i=0;i<(STEPPER_STEPS/STEPPER_STEP);i++) {
#if (DBG_PCMONITOR == 1)
                // プレビュー画像の送信
                size_t jpeg_size = create_jpeg();
                display_app.SendJpeg(get_jpeg_adr(), jpeg_size);
#endif

                // 輪郭画像の取得と立体形状復元
                led_working = 1;
                double rad = (double)(2*3.14)*((double)i/(STEPPER_STEPS/STEPPER_STEP));
                reconst(rad);

                // 取得した画像の保存
                sprintf(file_name, "/"MOUNT_NAME"/img_%d.jpg", file_name_index++);
                save_image_jpg(file_name); // save as jpeg
                printf("Saved file %s\r\n", file_name);

                led_working = 0;

                // テーブルの回転
                rotate(STEPPER_STEP);
            }

            // 復元した立体形状データの修正
            point_cloud.remove_edge();

            // 復元した立体形状データの出力
            cout << "writting..." << endl;
            led_working = 1;

            sprintf(file_name, "/"MOUNT_NAME"/result_%d.xyz", reconst_count);
            point_cloud.save_as_xyz(file_name);
            sprintf(file_name, "/"MOUNT_NAME"/result_%d.stl", reconst_count);
            point_cloud.save_as_stl(file_name);
            // sprintf(file_name, "/"MOUNT_NAME"/result_%d.ply", reconst_count);
            // point_cloud.save_as_ply(file_name);
            reconst_count++;

            led_working = 0;
            cout << "finish" << endl;
            point_cloud.clear();
        }

#if (DBG_PCMONITOR == 1)
        size_t jpeg_size = create_jpeg();
        display_app.SendJpeg(get_jpeg_adr(), jpeg_size);
#endif

    }
}
