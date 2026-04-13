#ifndef VITAL_SIGNS_DETECTOR_H
#define VITAL_SIGNS_DETECTOR_H

#ifdef complex
#undef complex
#endif

#include <vector>
#include <complex>
#include <QObject>
#include <deque>
#include "arm_math.h"
#include "arm_const_structs.h"

class VitalSignsDetector
{
public:
    VitalSignsDetector();
    ~VitalSignsDetector();

    void processFFTData(const std::vector<std::complex<float>>& fftData, int SubjectIdx);//处理主函数

    float getBreathingRate() const {return breathingRate;}
    float getHeartRate() const {return heartRate;}
    std::vector<float> getphaseBuf() const {return phaseBuf;}
    std::vector<float> getrrFreq() const {return rrFreq;}
    std::vector<float> gethrFreq() const {return hrFreq;}
    void clearAllBuffers();


private:
    //信号处理常数
    static constexpr float Fs = 20;
    static constexpr int FFTNum = 512;
    static constexpr int WindSize = 300;
    static constexpr float k_1 = Fs/FFTNum*60;
    static constexpr float SamplingRate = Fs;
    static constexpr int NumStages = 2; //级联双二阶
    static constexpr int BUFFER_SIZE = 25;   // 移动平均窗口大小

    std::vector<float> phaseBuf;
    std::vector<float> rrFreq;
    std::vector<float> hrFreq;
    int count = 0;
    float breathingRate = 0.0f;
    float heartRate = 0.0f;
    float rawBreathingRate=0.0f;
    float rawHeartRate=0.0f;

    // 相位函数
    std::vector<float> unwrap(const std::vector<float>& phase);
    std::vector<float> diff(const std::vector<float>& signal);

    //CMSIS_DSP库 FFT实例
    arm_cfft_instance_f32 fftInstance;
    float32_t* fftInput = nullptr;

    /*CMSIS_DSP库 巴特沃斯滤波器实例*/
    // 呼吸滤波器 (0.1-0.5 Hz)
    arm_biquad_cascade_df2T_instance_f64 rrFilterInstance;
    float64_t* rrFilterState = nullptr;
    float64_t rrFilterCoeffs[10];

    // 心率滤波器 (0.9-1.9 Hz)
    arm_biquad_cascade_df2T_instance_f64 hrFilterInstance;
    float64_t* hrFilterState = nullptr;
    float64_t hrFilterCoeffs[20];

    // DSP 滤波器设计
    void designButterworthBandpass(float fcLow, float fcHigh, float32_t* coeffs);
    std::vector<float> applyDSPFilter(const std::vector<float>& signal,
                                      arm_biquad_cascade_df2T_instance_f64* filterInstance);

    // DSP 库相关函数
    std::vector<float> DSPapplyWindow(const std::vector<float>& signal);
    std::vector<float> DSPcomputeFFT(const std::vector<float>& signal);

    /*自设计IIR滤波器*/
    // IIR 滤波器结构体
    struct IIRFilter {
        std::vector<double> b;      // 分子系数
        std::vector<double> a;      // 分母系数
        std::vector<double> w;      // 状态缓冲 (Direct Form II)
        int order;                  // 滤波器阶数
    };

    struct MultiNotchFilter {
        std::vector<IIRFilter> filters;  // 每个IIRFilter对应一个陷波频率
    };


    IIRFilter rrFilter;  // 呼吸滤波器 (4阶)
    IIRFilter hrFilter;  // 心率滤波器 (8阶)

    // IIR 滤波
    void initIIRFilter(IIRFilter& filter, const double* b, const double* a, int order);
    void resetIIRFilter(IIRFilter& filter);
    std::vector<float> applyIIRFilter(const std::vector<float>& signal, IIRFilter& filter);

    //平滑函数
    float movingAverage(std::deque<float>& buffer, float newValue);
    bool isOutlier(float value, float mean, float threshold = 2.0f);

    // 平滑相关成员
    std::deque<float> heartRateBuffer;      // 心率缓冲
    std::deque<float> breathingRateBuffer;  // 呼吸率缓冲
    float smoothedHeartRate;
    float smoothedBreathingRate;

    // 陷波滤波器设计
    void designNotchCoeffs(double fs, double f0, double bw, double* b, double* a);
    void initMultiNotchFilter(MultiNotchFilter& multiFilter,
                              const std::vector<double>& notchFreqs,
                              double fs, double bw);
    std::vector<float> applyMultiNotchFilter(const std::vector<float>& signal,
                                             MultiNotchFilter& multiFilter);

};

#endif // VITAL_SIGNS_DETECTOR_H
