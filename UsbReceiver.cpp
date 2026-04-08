#include "UsbReceiver.h"
#include <QDebug>
#include <QFile>

#pragma comment(lib,"SetupAPI.lib")
#pragma comment(lib,"User32.lib")
#pragma comment(lib,"legacy_stdio_definitions.lib")


UsbReceiver::UsbReceiver(QObject* parent)
    : QThread(parent), m_running(false), m_usbDevice(nullptr), m_bulkInEp(nullptr)
{
}

UsbReceiver::~UsbReceiver()
{
    stop();
    wait(3000);
    closeUsbDevice();
}

bool UsbReceiver::initUsbDevice()
{
    // 打开默认设备索引0，或按需修改
    emit usbError("开始初始化USB设备...");
    try {
        m_usbDevice = new CCyUSBDevice();
        emit usbError("CCyUSBDevice 创建成功");

        if (m_usbDevice->DeviceCount() && !m_usbDevice->Open(0)) {
            m_usbDevice->Reset();
            m_usbDevice->Open(0);
            emit usbError("Reset后重新打开设备");
        }

        if (!m_usbDevice->IsOpen()) {
            emit usbError("无法打开USB设备");
            delete m_usbDevice;
            m_usbDevice = nullptr;
            return false;
        }
        emit usbError("USB设备打开成功");

        // 打印所有端点信息
        int epCount = m_usbDevice->EndPointCount();
        emit usbError(QString("设备端点数量: %1").arg(epCount));

        // m_bulkInEp = nullptr;
        // for (int i = 0; i < epCount; i++) {
        //     CCyUSBEndPoint* ep = m_usbDevice->EndPoints[i];
        //     bool isBulk = (ep->Attributes == CY_U3P_USB_EP_BULK);        // 2 = Bulk传输
        //     bool isIn   = (ep->Address & 0x80) != 0;   // 最高位为1 = IN方向

        //     emit usbError(QString("端点[%1]: 地址=0x%2 MaxPktSize=%3 类型=%4")
        //                       .arg(i)
        //                       .arg(ep->Address, 2, 16, QChar('0'))
        //                       .arg(ep->MaxPktSize)
        //                       .arg(ep->Attributes));

        //     if (isBulk && isIn && m_bulkInEp == nullptr) {
        //         // 找到第一个Bulk IN端点
        //         m_bulkInEp = (CCyBulkEndPoint*)ep;
        //         emit usbError(QString("已选择端点[%1]作为数据接收端点").arg(i));
        //     }

        // }

        m_bulkInEp = m_usbDevice->BulkInEndPt;  // 直接用CyAPI自动识别的端点

        if (!m_bulkInEp) {
            emit usbError("未找到任何Bulk IN端点");
            delete m_usbDevice;
            m_usbDevice = nullptr;
            return false;
        }

        m_readBuffer.resize(m_frameSize);

    } catch (...) {
        emit usbError("异常: 打开USB设备失败");
        return false;
    }
    return true;
}

void UsbReceiver::closeUsbDevice()
{
    if (m_usbDevice) {
        delete m_usbDevice;
        m_usbDevice = nullptr;
        m_bulkInEp = nullptr;
    }
}

void UsbReceiver::run()
{
    if (m_frameSize<=0)
    {
        emit usbError("帧大小未设置或无效");
        return;
    }

    if (!initUsbDevice())
    {
        return;
    }

    m_running   = true;
    m_frameIndex = 0;
    m_rxBuffer.clear();

    emit usbError("USB线程开始接收数据");

    UCHAR* inBuf = reinterpret_cast<UCHAR*>(m_readBuffer.data());
    int bufSize = m_readBuffer.size(); //固定缓冲区大小

    while (m_running) {
        OVERLAPPED inOvLap = {0};
        inOvLap.hEvent = CreateEvent(NULL, false, false, NULL);

        long inLen = 0;

        // 完全参照参考代码
        UCHAR* inContext = m_bulkInEp->BeginDataXfer(inBuf, bufSize, &inOvLap);
        m_bulkInEp->WaitForXfer(&inOvLap, 2000);
        m_bulkInEp->FinishDataXfer(inBuf, inLen, &inOvLap, inContext);

        CloseHandle(inOvLap.hEvent);

        if (!m_running) break;

        if (inLen == 0) continue;

        //emit usbError(QString("inLen=%1").arg(inLen));

        m_rxBuffer.append(reinterpret_cast<const char*>(inBuf), inLen);

        // const int MAX_BUFFER = m_frameSize * 10;
        // if (m_rxBuffer.size() > MAX_BUFFER) {
        //     emit usbError("缓冲区溢出，丢弃数据");
        //     m_rxBuffer = m_rxBuffer.right(m_frameSize);
        // }

        while (m_rxBuffer.size() >= m_frameSize) {
            QByteArray frame = m_rxBuffer.left(m_frameSize);
            m_rxBuffer.remove(0, m_frameSize);

            // 验证帧大小是否正确
            // emit usbError(QString("第%1帧大小: %2 字节，期望: %3 字节")
            //                   .arg(m_frameIndex)
            //                   .arg(frame.size())
            //                   .arg(m_frameSize));

            // 直接在子线程写文件
            QString fileName = QString("%1/frame_%2.bin")
                                   .arg(m_saveDir)
                                   .arg(m_frameIndex, 6, 10, QChar('0'));
            QFile file(fileName);
            if (file.open(QIODevice::WriteOnly)) {
                file.write(frame);
                file.close();
            } else {
                emit usbError(QString("第%1帧写文件失败").arg(m_frameIndex));
            }

            // 每10帧通知更新日志
            if (m_frameIndex % 10 == 0) {
                emit usbDataReceived(m_frameIndex);
            }
            m_frameIndex++;

        }
    }

    closeUsbDevice();
}

void UsbReceiver::stop()
{
    m_running = false;
    if (m_bulkInEp)
    {
        m_bulkInEp->Abort();
    }
}
