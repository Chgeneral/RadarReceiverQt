#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDateTime>
#include <memory>
#include <QtCharts/QChartView>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>
#include "radar_processor.h"
#include "vital_signs_detector.h"
#include <QLCDNumber>
#include <QTimer>
#include "UsbReceiver.h"



QT_BEGIN_NAMESPACE
namespace Ui {
    class MainWindow;
}
QT_END_NAMESPACE

class QComboBox;
class QPushButton;
class QTextEdit;
class QLabel;
class QCheckBox;
class QSpinBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onRefreshPorts();
    void onOpenPort();
    void onClosePort();
    void onReadData();
    void onSendData();
    void onClearData();
    void onSaveData();
    void onProcessAndSave();
    void onDisplayModeChanged();
    void onOpenDatFile();
    // void onDisplayTimerTimeout();
    //void onPortError(QSerialPort::SerialPortError error);

    /*USB处理函数*/
    void onRefreshPortsUSB();
    void onOpenPortUSB();
    void onClosePortUSB();
    void onATStart();
    void onATReset();
    void onReadDataUSB();
    void onSaveDataUSB();

private:
    void setupConnections();
    void updatePortList();
    void appendLog(const QString &text,bool isReceived = true);
    QString byteArrayToHex(const QByteArray &data);
    QByteArray extractDataFromLog(const QString &logText);
    QByteArray rxBuffer;//帧缓冲

    //显示图像相关
    void setupCharts();
    void updateCharts();
    void plotFFTSpectrum(const int16_t *complex_data, int fft_points);
    void plotPhaseSpectrum(const int16_t *complex_data, int data_count);

    //呼吸心跳处理
    void setupVitalSignUI();
    void processVitalSigns();

    // 2) 抽帧函数
    bool tryTakeOneFrame(QByteArray& frame);
    void handleOneFrame(const QByteArray& frame);

    static constexpr int kMaxRxBufferBytes = 2 * 1024 * 1024;

    enum DisplayMode
    {
        TextMode,
        HexMode
    };


    /*USB处理函数*/
    void updatePortListUSB();
    void appendLogUSB(const QString &text);
    bool tryTakeOneFrame60(QByteArray& frame);

private:
    std::unique_ptr<Ui::MainWindow> ui;
    std::unique_ptr<QSerialPort> serialPort;
    std::unique_ptr<QTimer> displayTimer;
    std::unique_ptr<QTimer> timerElapsed;

    std::unique_ptr<QChart> chartFFT;
    std::unique_ptr<QChart> chartTime;
    std::unique_ptr<QChart> chartPhase;

    //std::unique_ptr<RadarData, RadarDataDeleter> currentRadar;
    using RadarPtr = std::unique_ptr<RadarData, decltype(&free_radar_data)>;
    //std::unique_ptr<RadarDataProcessor::RadarData> currentRadar;
    RadarPtr currentRadar{nullptr, &free_radar_data};

    std::unique_ptr<QChartView> chartViewFFT;
    std::unique_ptr<QChartView> chartViewPhase;

    QByteArray receivedBuffer;      //将数据读取到缓存区
    DisplayMode currentMode;
    qint64 totalBytesReceived;
    qint64 totalBytesSent;
    qint64 totalFrameReceived;
    QDateTime portOpenTime;
    int elapsedSeconds;

    std::unique_ptr<VitalSignsDetector> detector;
    QLCDNumber *breathingLCD;
    QLCDNumber *heartrateLCD;

    /*USB处理*/
    std::unique_ptr<QSerialPort> serialPortUSB;
    QTimer timerUSB;
    // 成员
    QByteArray rxBufferUSB;
    int totalFrameReceivedUSB = 0;
    //QList<QByteArray> m_frameBuffer;
    UsbReceiver* m_usbReceiver = nullptr;
    QString m_saveDir;       // 保存目录，onATStart时确定
    int     m_savedFrameCount = 0;  // 已保存帧数
    QDateTime m_collectStartTime;   // 记录采集开始时间

};

// class RadarDataDeleter {
// public:
//     void operator()(RadarDataProcessor::RadarData *radar) const {
//         if (radar) dataProcessor->free_radar_data(radar);
//     }
// };

#endif // MAINWINDOW_H
