# t42_img_control
Yale OpenHand Project model T-42を画像FB制御するパッケージ

## 実験条件
OS: Ubuntu 20.04

劣駆動ハンド: Yale OpenHand project ModelT-42(https://www.eng.yale.edu/grablab/openhand/model_t42.html)

カメラ: BASLER acA1300-200uc(https://www.baslerweb.com/jp/products/cameras/area-scan-cameras/ace/aca1300-200uc/)

## 使用方法
劣駆動ハンドとカメラを接続した状態で下記を実行
```
roslaunch t42_img_control img_fb_controller.launch
```

またパラメータについてはconfig/param.yamlに書いてある