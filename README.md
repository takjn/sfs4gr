# GR-Boads_Camera_sample
GR-PEACH、および、GR-LYCHEEで動作するサンプルプログラムです。  
GR-LYCHEEの開発環境については、[GR-LYCHEE用オフライン開発環境の手順](https://developer.mbed.org/users/dkato/notebook/offline-development-lychee-langja/)を参照ください。


## 概要
カメラ画像をUSB、または、SDに保存するサンプルです。  
USBとSDが両方挿入されている場合は、先に検出した方のデバイスに接続します。  
``USER_BUTTON0``を押すとbitmap形式で保存、``USER_BUTTON1``を押すとJPEG形式で保存します。  
bitmap形式での保存には [OpenCV](https://github.com/d-kato/opencv-lib) 、JPEG形式での保存には [JCU](https://developer.mbed.org/teams/Renesas/code/GraphicsFramework/) を使用します。  

カメラの指定を行う場合(GR-PEACHのみ)は``mbed_app.json``に``camera-type``を追加してください。
```json
{
    "config": {
        "camera":{
            "help": "0:disable 1:enable",
            "value": "1"
        },
        "camera-type":{
            "help": "Options are CAMERA_CVBS, CAMERA_MT9V111, CAMERA_OV7725",
            "value": "CAMERA_CVBS"
        },
        "lcd":{
            "help": "0:disable 1:enable",
            "value": "0"
        },
        "usb-host-ch":{
            "help": "(for GR-PEACH) 0:ch0 1:ch1",
            "value": "1"
        },
        "audio-camera-shield":{
            "help": "(for GR-PEACH) 0:use 1:not use",
            "value": "1"
        }
    },
    "target_overrides": {
        "*": {
            "target.macros_add": ["HAVE_OPENCV_IMGCODECS"]
        }
    }
}
```
camera-typeを指定しない場合は以下の設定となります。  
* GR-PEACH、カメラ：CAMERA_MT9V111  
* GR-LYCHEE、カメラ：CAMERA_OV7725  

カメラ画像をLCDやWindows用PCアプリで表示する場合は [GR-Boads_Camera_LCD_sample](https://github.com/d-kato/GR-Boads_Camera_LCD_sample) を参照ください。  
