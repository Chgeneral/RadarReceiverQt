#ifndef USBRECEIVER_H
#define USBRECEIVER_H

#include <QThread>
#include <QObject>
#include <QByteArray>

// 引入CyAPI头文件，确保包含路径正确
#include<Windows.h>
#include "CyAPI.h"

class UsbReceiver : public QThread
{
    Q_OBJECT
public:
    explicit UsbReceiver(QObject* parent = nullptr);
    ~UsbReceiver();

    // 线程主函数
    void run() override;

    // 停止线程读取
    void stop();

    // 设置每帧数据量（字节数），由Data_lineEdut传入
    void setFrameSize(int frameSize) {m_frameSize = frameSize;}

    void setSaveDir(const QString& dir){ m_saveDir = dir ; }

signals:
    void usbDataReceived(int frameIndex);
    void usbError(const QString& err);

private:
    bool initUsbDevice();
    void closeUsbDevice();



private:
    volatile bool m_running = false;

    CCyUSBDevice* m_usbDevice = nullptr;
    CCyBulkEndPoint* m_bulkInEp = nullptr;

    int m_frameSize = 0; //每帧字节数
    int m_frameIndex = 0;//当前帧序号
    QByteArray m_rxBuffer;//接收缓冲
    QByteArray m_readBuffer;   // 添加这一行，复用读取缓冲

    // 缓冲区大小示例，依据设备最大包大小确定
    static constexpr int BUFFER_SIZE = 1024 * 1024;

    QString m_saveDir;
};

#endif // USBRECEIVER_H
