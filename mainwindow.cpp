#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "radar_processor.h"
#include "vital_signs_detector.h"
#include <QSerialPortInfo>
#include <QMessageBox>
#include <QFileDialog>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>
#include <QRadioButton>
#include <QTimer>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include <QLCDNumber>
#include <QLabel>
#include <cmath>
#include <QDebug>


#define _USE_MATH_DEFINES


MainWindow::MainWindow(QWidget *parent)
    // 初始化
    : QMainWindow(parent)
    , ui(std::make_unique<Ui::MainWindow>())
    , serialPort(std::make_unique<QSerialPort>(this))
    , displayTimer(std::make_unique<QTimer>(this))
    , timerElapsed(std::make_unique<QTimer>(this))
    , currentMode(HexMode)
    , totalBytesReceived(0)
    , totalBytesSent(0)
    , totalFrameReceived(0)
    , elapsedSeconds(0)
    , chartFFT(std::make_unique<QChart>())
    , chartPhase(std::make_unique<QChart>())
    //, currentRadar(nullptr)
    , detector(std::make_unique<VitalSignsDetector>())
    , breathingLCD(nullptr)
    , heartrateLCD(nullptr)
    , serialPortUSB(std::make_unique<QSerialPort>(this))

{
    ui->setupUi(this);
    setWindowTitle("信号接收与处理工具 - Qt6");

    // // 设置定时器
    // displayTimer->setInterval(500);//1000对应1s


    displayTimer->setInterval(500);

    setupConnections();
    updatePortList();
    setupCharts();
    setupVitalSignUI();

    updatePortListUSB();
}

MainWindow::~MainWindow() = default;

static inline quint32 readBE32(const uchar* p) {
    return (quint32(p[0]) << 24) | (quint32(p[1]) << 16) | (quint32(p[2]) << 8) | quint32(p[3]);
}


void MainWindow::setupConnections()
{
    /*以下是关于串口接收模块的基本连接*/
    // 基本连接
    connect(ui->btnRefresh, &QPushButton::clicked, this, &MainWindow::onRefreshPorts);
    connect(ui->btnOpen, &QPushButton::clicked, this, &MainWindow::onOpenPort);
    connect(ui->btnClose, &QPushButton::clicked, this, &MainWindow::onClosePort);
    connect(ui->btnSend, &QPushButton::clicked, this, &MainWindow::onSendData);
    connect(ui->btnClear, &QPushButton::clicked, this, &MainWindow::onClearData);
    connect(ui->btnSave, &QPushButton::clicked, this, &MainWindow::onSaveData);
    connect(ui->btnProcess,&QPushButton::clicked,this,&MainWindow::onProcessAndSave);
    connect(ui->btnOpenDat, &QPushButton::clicked, this, &MainWindow::onOpenDatFile);

    // 显示模式切换
    connect(ui->rbtnText, &QRadioButton::toggled, this, &MainWindow::onDisplayModeChanged);
    //connect(ui->rbtnHex, &QRadioButton::toggled, this,&MainWindow::onDisplayModeChanged);


    // 串口接收与错误处理
    connect(serialPort.get(),&QSerialPort::readyRead,this,&MainWindow::onReadData);
    connect(serialPort.get(), QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
            this, [this](QSerialPort::SerialPortError error) {
                if (error != QSerialPort::NoError) {  // 只处理非 NoError 的情况
                    appendLog("串口错误: " + serialPort->errorString());
                }
            });

    //计时功能
    connect(timerElapsed.get(),&QTimer::timeout,this,[this](){
        if (serialPort->isOpen()){
            int elapsedSeconds = portOpenTime.secsTo(QDateTime::currentDateTime());
            ui->lblStatus->setText(QString("接收时长: %1 秒 | 接收帧数: %2 帧")
                                    .arg(elapsedSeconds)
                                    .arg(totalFrameReceived));
        }
    });


    ui->btnClose->setEnabled(false);
    ui->btnSend->setEnabled(false);
    ui->btnProcess->setEnabled(false);

    //默认选择16进制模式
    ui->rbtnHex->setChecked(true);

    /*以下是关于USB接收模块的连接*/
    ui->BauldRateUSB->setText("2000000");
    ui->BauldRateUSB->setEnabled(false); // 固定，不允许更改
    //基本连接
    connect(ui->btnOpenDeviceUSB, &QPushButton::clicked, this, &MainWindow::onOpenPortUSB);
    connect(ui->btnCloseDeviceUSB, &QPushButton::clicked, this, &MainWindow::onClosePortUSB);
    connect(ui->btnRefreshUSB, &QPushButton::clicked, this, &MainWindow::onRefreshPortsUSB);
    connect(ui->btnATStart, &QPushButton::clicked, this, &MainWindow::onATStart);
    connect(ui->btnATReset, &QPushButton::clicked, this, &MainWindow::onATReset);
    connect(ui->btnSaveDataUSB, &QPushButton::clicked, this, &MainWindow::onSaveDataUSB);
    //禁用关闭按钮
    ui->btnClearDataUSB->setEnabled(false);
    ui->btnATStart->setEnabled(false);
    ui->btnATReset->setEnabled(false);

    //接收连接
    connect(serialPortUSB.get(),&QSerialPort::readyRead,this,&MainWindow::onReadDataUSB);


}

void MainWindow::updatePortList()
{
    ui->cmbPort->clear();
    const auto infos = QSerialPortInfo::availablePorts();

    for (const auto & info : infos)
    {
        ui->cmbPort->addItem(info.portName());
    }
    if (ui->cmbPort->count() == 0)
    {
        ui->cmbPort->addItem("无可用串口");
    }
}

void MainWindow::updatePortListUSB()
{
    ui->cmbPortUSB->clear();
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : ports)
    {
        ui->cmbPortUSB->addItem(port.portName());
    }
    if (ui->cmbPortUSB->count() == 0)
    {
        ui->cmbPortUSB->addItem("无可用串口");
    }
}

void MainWindow::onRefreshPorts()
{
    updatePortList();
    appendLog("已刷新串口列表");
}

void MainWindow::onRefreshPortsUSB()
{
    updatePortListUSB();
    appendLogUSB("已刷新USB串口列表");
}

void MainWindow::onOpenPort()
{
    if (ui->cmbPort->currentText() == "无可用串口")
    {
        QMessageBox::warning(this,"错误","没有可用串口");
        return;
    }

    serialPort->setPortName(ui->cmbPort->currentText());
    serialPort->setBaudRate(ui->cmbBaudRate->currentText().toInt());
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (serialPort->open(QIODevice::ReadWrite))
    {
        portOpenTime = QDateTime::currentDateTime();
        timerElapsed->start(1000); //每秒更新

        ui->btnOpen->setEnabled(false);
        ui->btnClose->setEnabled(true);
        ui->btnSend->setEnabled(true);
        ui->cmbPort->setEnabled(false);
        ui->cmbBaudRate->setEnabled(false);
        ui->btnProcess->setEnabled(true);

        totalBytesReceived=0;
        totalBytesSent=0;
        totalFrameReceived=0;
        receivedBuffer.clear();

        serialPort->clear(QSerialPort::AllDirections);
        serialPort->readAll();   // 丢掉打开瞬间堆积的半帧
        rxBuffer.clear();        // 关键：重新同步

        appendLog(QString("串口 %1 已打开 (波特率: %2)")
                      .arg(serialPort->portName())
                      .arg(serialPort->baudRate()));
    } else {
        QMessageBox::critical(this, "错误", "无法打开串口: " + serialPort->errorString());
    }
}

void MainWindow::onOpenPortUSB()
{
    QString portName = ui->cmbPortUSB->currentText();
    if (portName == "无可用串口") {
        QMessageBox::warning(this, "错误", "没有可用USB串口");
        return;
    }

    bool ok;
    int dataSize = ui->Data_lineEdit->text().toInt(&ok);
    if (!ok || dataSize <= 0) {
        QMessageBox::warning(this, "错误", "请输入有效的数据量");
        return;
    }

    serialPortUSB->setPortName(portName);
    serialPortUSB->setBaudRate(2000000); // 固定波特率
    serialPortUSB->setDataBits(QSerialPort::Data8);
    serialPortUSB->setParity(QSerialPort::NoParity);
    serialPortUSB->setStopBits(QSerialPort::OneStop);
    serialPortUSB->setFlowControl(QSerialPort::NoFlowControl);

    if (serialPortUSB->open(QIODevice::ReadWrite)) {
        appendLogUSB(QString("USB串口 %1 已打开，波特率 2000000").arg(portName));

        ui->btnOpenDeviceUSB->setEnabled(false);
        ui->btnCloseDeviceUSB->setEnabled(true);
        ui->cmbPortUSB->setEnabled(false);
        ui->Data_lineEdit->setEnabled(false);
        ui->BauldRateUSB->setEnabled(false);
        ui->btnATStart->setEnabled(true);
        ui->btnATReset->setEnabled(false);

        // 可以用timerUSB或信号槽读取数据，根据需要实现读取逻辑

    } else {
        QMessageBox::critical(this, "错误", "打开USB串口失败: " + serialPortUSB->errorString());
    }
}

void MainWindow::onClosePort()
{
    if (serialPort->isOpen())
    {
        timerElapsed->stop();

        QDateTime portCloseTime = QDateTime::currentDateTime();
        int elapsedSeconds = portOpenTime.secsTo(portCloseTime);

        serialPort->clear(QSerialPort::AllDirections);
        serialPort->readAll();
        rxBuffer.clear();

        serialPort->close();
        ui->btnOpen->setEnabled(true);
        ui->btnClose->setEnabled(false);
        ui->btnSend->setEnabled(false);
        ui->cmbPort->setEnabled(true);
        ui->cmbBaudRate->setEnabled(true);

        appendLog(QString("串口已关闭 (接收时长: %1 秒)").arg(elapsedSeconds));
        ui->lblStatus->setText(QString("接收时长: %1 秒 | 接收帧数: %2 帧")
                                   .arg(elapsedSeconds)
                                   .arg(totalFrameReceived));
    }
}

void MainWindow::onClosePortUSB()
{
    // 关闭串口前先停止线程
    if (m_usbReceiver) {
        m_usbReceiver->stop();
        m_usbReceiver->wait();
        m_usbReceiver->deleteLater();
        m_usbReceiver = nullptr;
        appendLogUSB("USB接收线程已停止");
    }

    if (serialPortUSB->isOpen())
    {
        serialPortUSB->close();
        appendLogUSB("USB串口已关闭");

        ui->btnOpenDeviceUSB->setEnabled(true);
        ui->btnCloseDeviceUSB->setEnabled(false);
        ui->cmbPortUSB->setEnabled(true);
        ui->Data_lineEdit->setEnabled(true);
        ui->BauldRateUSB->setEnabled(false);
        ui->btnATStart->setEnabled(false);
        ui->btnATReset->setEnabled(false);
    }
}

void MainWindow::onATStart()
{
    if (!serialPortUSB || !serialPortUSB->isOpen()) {
        QMessageBox::warning(this, "错误", "USB串口未打开，请先打开外设");
        return;
    }

    // 获取帧大小
    bool ok;
    const int frameSize = ui->Data_lineEdit->text().toInt(&ok);
    if (!ok || frameSize <= 0) {
        QMessageBox::warning(this, "错误", "请输入有效的数据量");
        return;
    }

    //检查保存目录
    if (m_saveDir.isEmpty()) {
        QMessageBox::StandardButton reply = QMessageBox::question(
            this, "未设置保存路径",
            "当前未设置保存路径，是否继续？\n（数据将不会被保存）",
            QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::No) return;
    }


    m_savedFrameCount = 0;

    //先发送复位指令，确保正常启动
    QString resetCmd = "AT+RESET\n";
    serialPortUSB->write(resetCmd.toLatin1());
    serialPortUSB->flush();
    appendLogUSB("已发送复位指令: AT+RESET");

    //延迟等待复位
    QEventLoop loop;
    QTimer::singleShot(500, &loop, &QEventLoop::quit);
    loop.exec();

    QString cmd = "AT+START\n";
    qint64 written = serialPortUSB->write(cmd.toLatin1());
    serialPortUSB->flush();
    if (written == cmd.size()) {
        appendLogUSB("已发送指令: AT+START");
        m_collectStartTime = QDateTime::currentDateTime(); //记录当前采集开始时间
    } else {
        appendLogUSB("AT+START 指令发送失败");
    }

    ui->btnATStart->setEnabled(false);
    ui->btnATReset->setEnabled(true);

    // 启动USB接收线程
    if (m_usbReceiver)
    {
        m_usbReceiver->stop();
        m_usbReceiver->wait();
        m_usbReceiver->deleteLater();
        m_usbReceiver = nullptr;
    }

    m_usbReceiver = new UsbReceiver(this);
    m_usbReceiver->setSaveDir(m_saveDir);
    m_usbReceiver->setFrameSize(frameSize);

    connect(m_usbReceiver, &UsbReceiver::usbDataReceived,
            this, [this](int frameIndex) {

        m_savedFrameCount++;
        appendLogUSB(QString("已保存第 %1 帧").arg(frameIndex));

            });

    connect(m_usbReceiver, &UsbReceiver::usbError,
            this, [this](const QString& err) {

        appendLogUSB("[日志] " + err);

            });

    m_usbReceiver->start();
    appendLogUSB(QString("USB接收线程已启动，数据保存至: %1").arg(m_saveDir));
}

void MainWindow::onATReset()
{
    if (!serialPortUSB || !serialPortUSB->isOpen()) {
        QMessageBox::warning(this, "错误", "USB串口未打开，请先打开外设");
        return;
    }

    // 先停止接收线程
    if (m_usbReceiver) {
        m_usbReceiver->stop();
        m_usbReceiver->wait();
        m_usbReceiver->deleteLater();
        m_usbReceiver = nullptr;
        appendLogUSB("USB接收线程已停止");
    }

    int elapsedSecs = m_collectStartTime.secsTo(QDateTime::currentDateTime());

    appendLogUSB(QString("采集完成：共 %1 帧，历时 %2 秒")
                     .arg(m_savedFrameCount)
                     .arg(elapsedSecs));

    //发送复位指令
    QString cmd = "AT+RESET\n";
    qint64 written = serialPortUSB->write(cmd.toLatin1());
    if (written == cmd.size()) {
        appendLogUSB("已发送指令: AT+RESET");
    } else {
        appendLogUSB("AT+RESET 指令发送失败");
    }

    ui->btnATStart->setEnabled(true);
    ui->btnATReset->setEnabled(false);

    QMessageBox::information(this, "采集完成",
                             QString("本次采集已完成\n\n"
                                     "保存帧数：%1 帧\n"
                                     "采集时长：%2 秒\n"
                                     "保存路径：%3")
                                 .arg(m_savedFrameCount)
                                 .arg(elapsedSecs)
                                 .arg(m_saveDir.isEmpty() ? "未保存" : m_saveDir));
}

bool MainWindow::tryTakeOneFrame60(QByteArray& frame)
{
    frame.clear();
    const int FRAME_HEADER = 0x66BB;
    const int FRAME_TAIL = 0xBB66;
    const int FFT_DATA_SIZE = 80 * 2 * sizeof(float);  // 640字节
    const int TOTAL_FRAME_SIZE = 2 + FFT_DATA_SIZE + 2;  // 644字节

    while (rxBuffer.size() >= TOTAL_FRAME_SIZE) {
        int pos = -1;
        for (int i = 0; i <= rxBuffer.size() - 2; ++i) {
            uint16_t header = (uchar(rxBuffer[i]) << 8) | uchar(rxBuffer[i+1]);
            if (header == FRAME_HEADER) {
                pos = i;
                break;
            }
        }

        if (pos < 0) {
            if (rxBuffer.size() > 256) rxBuffer = rxBuffer.right(256);
            return false;
        }

        int tailPos = pos + 2 + FFT_DATA_SIZE;
        if (tailPos + 2 > rxBuffer.size()) {
            return false;
        }

        uint16_t tail = (uchar(rxBuffer[tailPos]) << 8) | uchar(rxBuffer[tailPos+1]);
        if (tail == FRAME_TAIL) {
            frame = rxBuffer.mid(pos, TOTAL_FRAME_SIZE);
            rxBuffer.remove(0, pos + TOTAL_FRAME_SIZE);
            return true;
        }

        rxBuffer.remove(0, pos + 1);
    }

    return false;
}

void MainWindow::handleOneFrame(const QByteArray& frame)
{
    RadarData* radarPtr = process_fft_data_stream(
        reinterpret_cast<const uint8_t*>(frame.constData()),
        size_t(frame.size())
        );
    currentRadar.reset(radarPtr);

    if (currentRadar && currentRadar->data_count > 0) {
        totalFrameReceived++;        // 关键：按“抽到的完整帧”计数
        updateCharts();
        processVitalSigns();
    }
}

void MainWindow::onReadData()
{
    const QByteArray data = serialPort->readAll();
    if (!data.isEmpty()) {
        totalBytesReceived += data.length();

        //开辟缓存保存接收数据
        receivedBuffer.append(data);

        //显示接收数据
        QString displayText;
        if (currentMode == TextMode) {
            displayText = QString::fromUtf8(data);
        } else {
            displayText = byteArrayToHex(data);
        }

        QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString logEntry = QString("[%1] [接收] %2").arg(timestamp, displayText);
        ui->txtReceive->append(logEntry);

        //获取运行时间
        QDateTime portTime = QDateTime::currentDateTime();
        int elapsedSeconds = portOpenTime.secsTo(portTime);

        // //获取实时的雷达数据
        // RadarData *radarPtr = process_radar_data((const uint8_t *)data.constData(), data.size());
        // currentRadar.reset(radarPtr);

        // // 直接调用更新
        // if (currentRadar && currentRadar->data_count > 0) {
        //     totalFrameReceived++;
        //     updateCharts();
        //     processVitalSigns();
        // }

        // 1) 追加到同步缓冲
        rxBuffer.append(data);

        // 2) 防止同步失败导致内存无限增长
        if (rxBuffer.size() > 4096) {
            rxBuffer = rxBuffer.right(2048);
        }

        // 3) 循环抽帧并处理
        QByteArray frame;
        while (tryTakeOneFrame60(frame)) {
            handleOneFrame(frame);
        }


        // 更新统计信息
        ui->lblStatus->setText(QString("接收时长: %1 秒 | 接收帧数: %2 帧")
                                  .arg(elapsedSeconds)
                                  .arg(totalFrameReceived));
    }
}

void MainWindow::onReadDataUSB()
{
    const QByteArray data = serialPortUSB->readAll();
    if (!data.isEmpty())
    {
        // 开辟缓存保存接收数据
        rxBufferUSB.append(data);

        //显示接收数据
        QString displayText;
        displayText =QString::fromUtf8(data);

        QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
        QString logEntry = QString("[%1] [接收] %2").arg(timestamp, displayText);
        //ui->textReceiverUSB->append(logEntry);
    }

}

void MainWindow::onSendData()
{
    const QString text = ui->txtSend->toPlainText();
    if (text.isEmpty()) {
        QMessageBox::warning(this, "提示", "发送内容不能为空");
        return;
    }

    if (serialPort->isOpen()) {
        QByteArray dataToSend;

        // 根据当前模式确定发送数据
        if (currentMode == TextMode)
        {
            dataToSend = text.toLatin1();
        }else
        {
            //16进制模式：去除空格并转换
            QString hexText = text;
            hexText = hexText.remove(" ");
            hexText = hexText.remove("\n");
            for (int i = 0; i < hexText.length();i+=2)
            {
                bool ok;
                uchar byte = hexText.mid(i,2).toUShort(&ok,16);
                if(ok)
                {
                    dataToSend.append(byte);
                }
            }

        }
        if (!dataToSend.isEmpty())
        {
            serialPort->write(dataToSend);
            totalBytesSent +=dataToSend.length();

            QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
            QString displayText = (currentMode == TextMode)? text : byteArrayToHex(dataToSend);
            QString logEntry = QString("[%1] [发送] %2").arg(timestamp, displayText);
            ui->txtReceive->append(logEntry);

            //获取时间
            QDateTime portTime = QDateTime::currentDateTime();
            int elapsedSeconds = portOpenTime.secsTo(portTime);


            //更新统计信息
            ui->lblStatus->setText(QString("接收时长: %1 秒 | 接收帧数: %2 帧")
                                       .arg(elapsedSeconds)
                                       .arg(totalFrameReceived));

            ui->txtSend->clear();

        }else
        {
            QMessageBox::warning(this, "错误", "16进制数据格式不正确");
        }
    }
}

//图像相关处理函数
void MainWindow::updateCharts()
{
    if (!currentRadar || currentRadar->data_count == 0) return;

    plotFFTSpectrum(currentRadar->complex_data, currentRadar->fft_points);
}

//生命体征处理
void MainWindow::processVitalSigns()
{
    if (!currentRadar || currentRadar->data_count == 0) return;

    // 转换复数数据 默认第一个chirps
    std::vector<std::complex<float>> fftData;
    for (int i = 0; i < currentRadar->fft_points; ++i) {
        float real = currentRadar->complex_data[i * 2];
        float imag = currentRadar->complex_data[i * 2 + 1];
        fftData.push_back(std::complex<float>(real, imag));
    }

    // 处理数据
    int SubjectIdx = 4;
    detector->processFFTData(fftData, SubjectIdx);

    // 更新LCD显示
    if (breathingLCD && heartrateLCD) {
        breathingLCD->display(static_cast<int>(detector->getBreathingRate()));
        heartrateLCD->display(static_cast<int>(detector->getHeartRate()));
    }
}

void MainWindow::plotFFTSpectrum(const int16_t *complex_data, int fft_points)
{
    if (!chartFFT || !complex_data || fft_points <= 0) return;

    chartFFT->setAnimationOptions(QChart::NoAnimation);  // 禁用动画
    chartFFT->removeAllSeries();

    auto series = new QLineSeries();
    series->setName("FFT");

    //chirps数选择
    int chirpIdx = 1;
    int idxmove=fft_points*(chirpIdx-1);

    for (int i = 0+idxmove; i < idxmove+fft_points; ++i) {
        double fftdata=sqrt(complex_data[i * 2]*complex_data[i * 2]+complex_data[i * 2 + 1]*complex_data[i * 2 + 1]);
        //double fftdata_db=10.0*log10(fftdata+1);
        series->append(i, fftdata);
    }

    //获取其他过程图像并显示
    // std::vector<float> Freq = detector->getrrFreq();
    // for (int i = 0; i < Freq.size(); ++i) {
    //     double data=Freq[i];
    //     //double fftdata_db=10.0*log10(fftdata+1);
    //     series->append(i, data);
    // }

    chartFFT->addSeries(series);
    chartFFT->createDefaultAxes();

    // 设置X轴为整数
    QValueAxis *axisX = qobject_cast<QValueAxis*>(chartFFT->axes(Qt::Horizontal).at(0));
    if (axisX) {
        axisX->setLabelFormat("%i");  // 整数格式
        //axisX->setTickCount(fft_points / 10 + 1);  // 调整刻度数量
    }
}

void MainWindow::onClearData()
{
    ui->txtReceive->clear();
    receivedBuffer.clear();//清楚文件缓存
    chartFFT->removeAllSeries();
    //detector->clearAllBuffers();
    rxBuffer.clear();
    totalBytesReceived = 0;
    totalBytesSent = 0;
    totalFrameReceived = 0;
    ui->lblStatus->setText("接收时长: 0 秒 | 接收帧数: 0 帧");
    if (breathingLCD) breathingLCD->display(0);
    if (heartrateLCD) heartrateLCD->display(0);
}

void MainWindow::onSaveData()
{
    const QString fileName = QFileDialog::getSaveFileName(this,
                                                          "保存数据","","文本文件 (*.dat);;所有文件 (*)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (file.open(QIODevice::WriteOnly))
    {
        QByteArray dataToSave = receivedBuffer;

        file.write(dataToSave);
        file.close();

        QMessageBox::information(this,"成功",
                                 QString("数据已保存到:%1\n 总大小：%2 字节").arg(fileName).arg(dataToSave.length()));
    }else
    {
        QMessageBox::critical(this, "错误", "无法保存文件");
    }
}

void MainWindow::onSaveDataUSB()
{
    QString dir = QFileDialog::getExistingDirectory(this, "选择数据保存目录");
    if (dir.isEmpty()) return;

    m_saveDir = dir;
    appendLogUSB(QString("保存路径已设置: %1").arg(m_saveDir));
}

void MainWindow::onProcessAndSave()
{
    const QString outputPath = QFileDialog::getSaveFileName(this,
                                                            "保存处理后的数据", "", "CSV文件 (*.csv);;所有文件 (*)");
    if (outputPath.isEmpty()) return;

    //获取需要处理的数据
    QByteArray receivedData=receivedBuffer;
    // 调用 C 处理函数
    RadarData *radar = process_radar_stream(
        reinterpret_cast<const uint8_t*>(receivedData.constData()),
        size_t(receivedData.size())
        );

    if (!radar || radar->data_count <= 0) {
        QMessageBox::critical(this, "错误", "数据处理失败/没有解析到有效帧");
        if (radar) free_radar_data(radar);
        return;
    }


    if (!radar) {
        QMessageBox::critical(this, "错误", "数据处理失败");
        return;
    }

    // 保存处理后的数据
    if (save_radar_csv(radar, outputPath.toStdString().c_str()) == 0) {
        appendLog(QString("[处理完成] 数据已保存: %1").arg(outputPath));
        QMessageBox::information(this, "成功",
                                 QString("处理完成！\nFFT点数: %1\n数据点数: %2\n已保存到: %3")
                                     .arg(radar->fft_points)
                                     .arg(radar->data_count)
                                     .arg(outputPath));
    } else {
        QMessageBox::critical(this, "错误", "保存文件失败");
    }

    free_radar_data(radar);

}

void MainWindow::onOpenDatFile()
{
    const QString fileName = QFileDialog::getOpenFileName(this,
                                                          "打开.dat文件", "", "DAT文件 (*.dat);;所有文件 (*)");

    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "错误", "无法打开文件");
        return;
    }

    QByteArray fileData = file.readAll();
    file.close();

    if (fileData.isEmpty()) {
        QMessageBox::warning(this, "提示", "文件为空");
        return;
    }

    // 调用处理函数
    RadarData *radar = process_radar_data(
        reinterpret_cast<const uint8_t*>(fileData.constData()),
        fileData.size()
        );

    if (!radar) {
        QMessageBox::critical(this, "错误", "数据处理失败");
        return;
    }

    // 保存为CSV
    const QString outputPath = QFileDialog::getSaveFileName(this,
                                                            "保存CSV文件", "", "CSV文件 (*.csv);;所有文件 (*)");

    if (!outputPath.isEmpty()) {
        if (save_radar_csv(radar, outputPath.toStdString().c_str()) == 0) {
            appendLog(QString("从文件读取并处理完成: %1").arg(fileName));
            QMessageBox::information(this, "成功",
                                     QString("处理完成！\nFFT点数: %1\n数据点数: %2\n已保存到: %3")
                                         .arg(radar->fft_points)
                                         .arg(radar->data_count)
                                         .arg(outputPath));
        } else {
            QMessageBox::critical(this, "错误", "保存CSV失败");
        }
    }


    free_radar_data(radar);
}

//创建图表相关函数
void MainWindow::setupCharts()
{
    // FFT图表
    chartFFT = std::make_unique<QChart>();
    chartFFT->setTitle("FFT频谱");
    chartFFT->setAnimationOptions(QChart::SeriesAnimations);
    chartFFT->legend()->hide();  // 隐藏FFT图表图例

    // chartViewFFT = std::make_unique<QChartView>(chartFFT.get(), this);
    // chartViewFFT->setRenderHint(QPainter::Antialiasing);
    // chartViewFFT->resize(800, 400);  // 设置大小
    // //chartViewFFT->move(10, 10);      // 设置位置
    // chartViewFFT->show();

    // 如果UI中有 chartViewFFT 控件，设置图表
    if (ui->chartViewFFT) {
        ui->chartViewFFT->setChart(chartFFT.get());
        ui->chartViewFFT->setRenderHint(QPainter::Antialiasing);
    }

    // 相位谱图表
    // chartPhase = std::make_unique<QChart>();
    // chartPhase->setTitle("相位谱");
    // chartPhase->setAnimationOptions(QChart::SeriesAnimations);

    // auto chartViewPhase = new QChartView(chartPhase.get(), this);
    // chartViewPhase->setRenderHint(QPainter::Antialiasing);
}

void MainWindow::setupVitalSignUI()
{
    //创建LCD
    breathingLCD = ui->breathingLCD;
    breathingLCD->setDigitCount(3);
    breathingLCD->setSegmentStyle(QLCDNumber::Flat);
    breathingLCD->display(0);

    heartrateLCD = ui->heartrateLCD;
    heartrateLCD->setDigitCount(3);
    heartrateLCD->setSegmentStyle(QLCDNumber::Flat);
    heartrateLCD->display(0);

    //创建连接
    //connect(detector, &VitalSignsDetector::)

}

void MainWindow::onDisplayModeChanged()
{
    if (ui->rbtnText->isChecked()) {
        currentMode = TextMode;
    } else {
        currentMode = HexMode;
    }
    appendLog(QString("切换到%1模式").arg(currentMode == TextMode ? "文本" : "16进制"));
}

QString MainWindow::byteArrayToHex(const QByteArray &data)
{
    QString hexString;
    for (int i = 0; i < data.length(); ++i) {
        hexString += QString("%1 ").arg((uchar)data[i], 2, 16, QChar('0'));
    }
    return hexString.trimmed();
}

void MainWindow::appendLog(const QString &text, bool isReceived)
{
    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    const QString prefix = isReceived ? "[接收]" : "[发送]";
    ui->txtReceive->append(QString("%1 %2 %3").arg(timestamp, prefix, text));
}

void MainWindow::appendLogUSB(const QString &text)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->textReceiverUSB->append(QString("[%1][USB] %2").arg(timestamp, text));
}
