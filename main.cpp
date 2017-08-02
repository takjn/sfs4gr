/*
** DIY GR-LYCHEE/GR-PEACH 3D Scanner
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

// 3D reconstruction Parameters
#define SILHOUETTE_COUNTS   40  // number of silhouette to use

// Stepper motor parameters (Depends on your stepper motor)
#define STEPPER_DIRECTION   1       // Direction (0 or 1)
#define STEPPER_WAIT        0.004   // Pulse duration
#define STEPPER_STEP_COUNTS 200     // A 200 step motor is the same as a 1.8 degrees motor 

// Stepper motor driver parameters (Depends on your circuit design)
#define STEPPER_STEP_RESOLUTIONS 4  // full-step = 1, half-step = 2, quarter-step = 4

// Defines pins numbers (Depends on your circuit design)
DigitalOut  a4988_step(D8);     // Connect the pin to A4988 step
DigitalOut  a4988_dir(D9);      // Connect the pin to A4988 dir
DigitalIn   button0(D6);        // Connect the pin to SW1
DigitalOut  led_working(D7);    // Connect the pin to LED1 (working)
DigitalOut  led1(LED1);         // Use onboard LED for debugging purposes

// Global variable for 3D reconstruction
PointCloud point_cloud;      // Point cloud (3D reconstruction result)

int reconst_index = 1;
int file_name_index = 1;
char file_name[32];

/* For viewing image on PC */
static DisplayApp  display_app;

// Projects a 3D point into camera coordinates
int projection(double rad, double Xw, double Yw,double Zw, int &u, int &v)
{
    // Pitch rotations around the Y axis
    double Xc= cos(rad)*Xw + sin(rad)*Zw;
    double Yc= Yw;
    double Zc=-sin(rad)*Xw + cos(rad)*Zw;

    // Perspective projection
    Yc+=CAMERA_OFFSET;
    Zc-=CAMERA_DISTANCE;
  
    u= CAMERA_CENTER_U - (int)((Xc/Zc)*(CAMERA_FX));
    v= CAMERA_CENTER_V - (int)((Yc/Zc)*(CAMERA_FY));

    return (u>0 && u<VIDEO_PIXEL_HW && v>0 && v<VIDEO_PIXEL_VW);
}

// Voxel based "Shape from silhouette"
// Only voxels that lie inside all silhouette volumes remain part of the final shape.
void shape_from_silhouette(double rad) {

    // Take a silhouette
    cv::Mat img_silhouette = get_silhouette();

    // Saves a silhouette image for dubugging purposes
    // sprintf(file_name, "/storage/img_%d.bmp", file_name_index);
    // cv::imwrite(file_name, img_silhouette);
    // printf("Saved file %s\r\n", file_name);

    // Check each voxels
    double xx,yy,zz;    // 3D point(x,y,z)
    int u,v;            // camera coordinates(x,y)
    int pcd_index=0;

    zz = (-point_cloud.SIZE / 2) * point_cloud.SCALE;
    for (int z=0; z<point_cloud.SIZE; z++, zz += point_cloud.SCALE) {

        yy = (-point_cloud.SIZE / 2) * point_cloud.SCALE;
        for (int y=0; y<point_cloud.SIZE; y++, yy += point_cloud.SCALE) {

            xx = (-point_cloud.SIZE / 2) * point_cloud.SCALE;
            for (int x=0; x<point_cloud.SIZE; x++, xx += point_cloud.SCALE, pcd_index++) {
                if (point_cloud.get(pcd_index) == 1) {
                    
                    // Project a 3D point into camera coordinates
                    if (projection(rad, xx, yy, zz, u, v)) {
                        if (img_silhouette.at<unsigned char>(v, u)) {
                            // Keep the point because it is inside the shilhouette
                        }
                        else {
                            // Delete the point because it is outside the shilhouette
                            point_cloud.set(pcd_index, 0);
                        }
                    } else {
                        // Delete the point because it is outside the camera image
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
    // Start camera
    camera_start();
    led1 = 1;

    // Connect SD & USB
    SdUsbConnect storage("storage");

    // Reset stepper motor
    a4988_dir = STEPPER_DIRECTION;
    a4988_step = 0;

    while (1) {
        storage.wait_connect();

        if (button0 == 0) {
            // Scan 3D object with camera
            // Repeat taking a image and 3D reconstruction while rotating the turntable.
            for (int i = 0; i < SILHOUETTE_COUNTS; i++) {
                // Send a preview image to PC
                size_t jpeg_size = create_jpeg();
                display_app.SendJpeg(get_jpeg_adr(), jpeg_size);

                // Shape from silhouette
                led_working = 1;
                double rad = (double)(2 * 3.14159265258979)*((double)i / SILHOUETTE_COUNTS);
                shape_from_silhouette(rad);

                // Save a preview image for dubugging purposes
                sprintf(file_name, "/storage/img_%d.jpg", file_name_index++);
                save_image_jpg(file_name); // save as jpeg
                printf("Saved file %s\r\n", file_name);

                led_working = 0;

                // Rotate the turntable
                rotate(STEPPER_STEP_COUNTS * STEPPER_STEP_RESOLUTIONS / SILHOUETTE_COUNTS);
            }

            // Save the result
            cout << "writting..." << endl;
            led_working = 1;

            // Finalize the result
            point_cloud.finalize();

            sprintf(file_name, "/storage/result_%d.xyz", reconst_index);
            point_cloud.save_as_xyz(file_name);
            sprintf(file_name, "/storage/result_%d.stl", reconst_index);
            point_cloud.save_as_stl(file_name);
            // sprintf(file_name, "/storage/result_%d.ply", reconst_index);
            // point_cloud.save_as_ply(file_name);

            reconst_index++;

            led_working = 0;
            cout << "finish" << endl;
            point_cloud.clear();
        }

        // Send a preview image to PC
        size_t jpeg_size = create_jpeg();
        display_app.SendJpeg(get_jpeg_adr(), jpeg_size);
    }
}
