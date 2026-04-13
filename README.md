[README.md](https://github.com/user-attachments/files/26670911/README.md)
# RadarReceiverQt

基于 Qt6 的 60GHz 毫米波雷达数据采集与处理上位机，适配 RC6011 硬件平台。

电子科技大学 科研项目

---

## 功能概述

- **雷达数据采集**：通过 USB3.0 高速通道（CyAPI）接收雷达原始数据，按帧分割存储为 `.bin` 文件
- **串口通信**：通过串口发送 AT 指令控制雷达启停（`AT+START` / `AT+RESET`）
- **实时信号处理**：FFT 频谱实时显示，基于 CMSIS-DSP 库实现呼吸率与心率估计
- **摄像头采集**：同步采集摄像头画面，支持录像保存
- **OCR 数字识别**：识别监护仪等仪器屏幕上的数字（心率、呼吸率），与雷达采集同步，结果保存为 CSV
- **状态监控**：实时显示采集时长与帧数统计

---

## 版本记录

| 版本 | 说明 |
|------|------|
| v1.0 | 基础采集版本，USB 原始数据采集与存储 |
| v1.1 | 添加摄像头 OCR 识别、状态栏显示、路径管理优化 |

---

## 环境依赖

| 依赖 | 版本 |
|------|------|
| Qt | 6.8.3 |
| 编译器 | MSVC 2022 64bit |
| 构建工具 | CMake 3.19+ |
| CMSIS-DSP | 包含在项目中 |
| Tesseract OCR | 5.x（本地安装） |
| CyAPI | Cypress USB 驱动库 |

---

## 项目结构

```
RadarDataReceiver/
├── mainwindow.cpp / .h     # 主窗口逻辑
├── mainwindow.ui           # UI 布局
├── UsbReceiver.cpp / .h    # USB 高速数据接收线程（CyAPI）
├── CameraLabel.cpp / .h    # 支持框选的摄像头显示控件
├── OcrWrapper.cpp / .h     # OCR 识别封装（调用 Tesseract CLI）
├── radar_processor.cpp / .h # 雷达数据处理
├── vital_signs_detector.cpp / .h # 呼吸心率估计
├── CMSIS_DSP/              # DSP 算法库
└── usb3p0/                 # CyAPI 库文件
```

---

## 使用流程

**雷达数据采集：**
1. 连接 RC6011 硬件，选择对应串口与 USB 设备
2. 点击**打开外设**，输入每帧数据量（字节数）
3. 点击**设置保存路径**，原始数据自动保存至 `路径/Data/` 目录
4. 点击**开始采集**，程序自动发送复位与启动指令
5. 点击**停止采集**，查看采集统计

**OCR 识别（可选，与雷达采集同步）：**
1. 连接摄像头，框选心率与呼吸率数字区域
2. 在采样率输入框填写识别频率（建议 0.5~1 Hz）
3. 点击**打开数字识别**
4. 点击**开始采集**，OCR 与雷达同步启动
5. 停止后点击**保存标签数据**，CSV 保存至采集根目录

---

## 构建说明

```bash
git clone https://github.com/Chgeneral/RadarReceiverQt.git
cd RadarReceiverQt
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build .
```

需提前安装：
- Qt 6.8.3 MSVC 版本
- Tesseract OCR（并配置 `TESSDATA_PREFIX` 环境变量）
- Cypress CyAPI 驱动

---

## 数据格式

- **原始雷达数据**：`frame_XXXXXX.bin`，每帧固定字节数，16通道 × 2Chirps × 512点 × 2字节
- **标签数据**：`label_data.csv`，字段为 `时间戳, 心率(HR), 呼吸率(RR)`

---

## License

仅供学术研究使用，未经许可不得用于商业目的。
