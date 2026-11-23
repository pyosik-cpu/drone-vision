# SG2002 CVITEK Runtime 模型信息工具

第一阶段仅加载 CVI 模型、读取并打印输入输出张量信息，不执行推理。

`sg2002_armor_detect` 在单张 JPEG 上执行装甲板模型推理，打印四角点、颜色类别、数字类别和置信度。

## 构建

```bash
cmake --preset sg2002-release
cmake --build --preset sg2002-release -j
```

## 部署与运行

```bash
scp build/sg2002-release/sg2002_model_info root@192.168.42.1:/root/
ssh -t root@192.168.42.1 \
  'LD_LIBRARY_PATH=/mnt/system/lib:/mnt/system/usr/lib:/mnt/system/usr/lib/3rd /root/sg2002_model_info /mnt/cvimodel/scrfd_768_432_int8_1x.cvimodel'
```

装甲板单图测试：

```bash
scp build/sg2002-release/sg2002_armor_detect root@192.168.42.1:/root/
ssh -t root@192.168.42.1 \
  'LD_LIBRARY_PATH=/mnt/system/lib:/mnt/system/usr/lib:/mnt/system/usr/lib/3rd /root/sg2002_armor_detect /root/yolov5_armor_320_bf16.cvimodel /root/armor_test.jpg 0.5 0.45 /root/armor_result.jpg'
```

程序静态链接 MaixCDK 中的 CVITEK TPU Runtime 库，但仍需在板端验证其与模型、驱动及固件是否兼容。若模型注册失败，应改用 SG2002 板端固件对应版本的开发包重新链接，不应覆盖板端已有动态库。

也可以在 Ubuntu 项目目录中运行一键测试脚本，结果会自动下载到 `results/`：

```bash
./scripts/run_armor_test.sh /home/pyosik/桌面/rm.jpg
```

连续执行 100 次推理并测量不含模型加载、JPEG 解码的 TPU FPS：

```bash
/root/sg2002_armor_detect /root/yolov5_armor_320_bf16.cvimodel \
  /root/armor_test.jpg 0.5 0.45 /root/armor_result.jpg 100
```

## 实时识别与PnP

`sg2002_armor_camera` 的可选第六个参数是相机标定配置文件。PnP使用项目内置的
平面单应性初值和迭代重投影优化，不依赖板端缺失的 `libopencv_calib3d`。

```bash
scp build/sg2002-release/sg2002_armor_camera root@192.168.42.1:/root/
scp calibration/camera_calibration.ini root@192.168.42.1:/root/

# 模型、置信度、NMS、帧数、敌方颜色、标定文件
/root/sg2002_armor_camera /root/yolov5_armor_320_bf16.cvimodel \
  0.5 0.45 0 any /root/camera_calibration.ini
```

标定配置的分辨率必须与传给PnP的原图坐标一致。当前检测角点映射回传感器
1920x1080坐标，因此标定脚本会把RTSP 1280x720内参同比例换算到1920x1080。

## 串口发送检测结果

串口默认关闭。确认开发板上未被系统控制台占用的串口设备后，可追加设备和
波特率参数：

```bash
/root/sg2002_armor_camera /root/yolov5_armor_320_bf16.cvimodel \
  0.5 0.45 0 any /root/camera_calibration.ini /dev/ttyS3 115200
```

如果暂时不传标定文件但需要测试中心坐标串口发送，第六个参数使用 `-`：

```bash
/root/sg2002_armor_camera /root/yolov5_armor_320_bf16.cvimodel \
  0.5 0.45 0 any - /dev/ttyS3 115200
```

程序在确认主目标后发送一行纯文本：
`x_m,y_m,z_m,center_x_px,center_y_px`。详细定义见
[`docs/serial_protocol.md`](docs/serial_protocol.md)。PnP关闭或重投影误差超限时
XYZ为0，中心像素坐标仍然有效。

## Buildroot开机服务

板端使用BusyBox init。服务脚本位于 `packaging/S99armor-camera`，安装到
`/etc/init.d/` 后会在开机时配置GP4为UART3_TX，并持续运行摄像头识别、
RTSP推流和串口发送。服务错误日志位于 `/tmp/armor-camera.err`。

```bash
scp packaging/S99armor-camera root@192.168.42.1:/etc/init.d/
ssh root@192.168.42.1 'chmod +x /etc/init.d/S99armor-camera && /etc/init.d/S99armor-camera start'
```

手动管理命令为 `start`、`stop`、`restart` 和 `status`。
