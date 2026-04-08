#define _USE_MATH_DEFINES
#include "vital_signs_detector.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <deque>

/*滤波器系数*/
static const double breathB[5] = {
    0.00362168151492858,
    0,
    -0.00724336302985716,
    0,
    0.00362168151492858
};

static const double breathA[5] = {
    1,
    -3.81325425602585,
    5.46451668975380,
    -3.48842176300383,
    0.837181651256022
};

static const double heartB[7] = {
    0.00289819463372144,
    0,
    -0.00869458390116431,
    0,
    0.00869458390116431,
    0,
    -0.00289819463372144
};

static const double heartA[7] = {
    1,
    -4.92323955519578,
    10.4959735661824,
    -12.3714278233939,
    8.49987016324314,
    -3.22980603668868,
    0.532075368312090
};




VitalSignsDetector::VitalSignsDetector()
{
    phaseBuf.resize(WindSize,0.0f); //初始化

    // 初始化FFT
    fftInput = new float32_t[FFTNum * 2];
    std::memset(fftInput, 0, FFTNum * 2 * sizeof(float32_t));
    arm_cfft_init_f32(&fftInstance, FFTNum);

    // // 初始化呼吸滤波器状态 (0.1-0.5 Hz)
    // rrFilterState = new float64_t[4 * 2];
    // std::memset(rrFilterState, 0, 4 * 2 * sizeof(float64_t));
    // //designButterworthBandpass(0.1f, 0.5f, rrFilterCoeffs);

    // // 分解为2个2阶级联
    // // 第一级 (b0, b1, b2, a1, a2)
    // rrFilterCoeffs[0] = 0.067455273889071853954391144725377671421;
    // rrFilterCoeffs[1] = -0.134910547778143707908782289450755342841;
    // rrFilterCoeffs[2] = 0.067455273889071853954391144725377671421;
    // rrFilterCoeffs[3] = -2.990204825208073735609559662407264113426;
    // rrFilterCoeffs[4] = 3.456335732017893569434363598702475428581;

    // // 第二级 (b0, b1, b2, a1, a2)
    // rrFilterCoeffs[5] = 0.067455273889071853954391144725377671421;
    // rrFilterCoeffs[6] = -0.134910547778143707908782289450755342841;
    // rrFilterCoeffs[7] = 0.067455273889071853954391144725377671421;
    // rrFilterCoeffs[8] = -1.87289372938147824498855698038823902607;
    // rrFilterCoeffs[9] = 0.412801598096190269782113091423525474966;

    // arm_biquad_cascade_df2T_init_f64(&rrFilterInstance, 2,
    //                                  rrFilterCoeffs, rrFilterState);

    // // 初始化心率滤波器状态 (0.9-1.9 Hz)
    // hrFilterState = new float64_t[4 * 4];
    // std::memset(hrFilterState, 0, 4 * 4 * sizeof(float64_t));
    // //designButterworthBandpass(0.9f, 1.9f, hrFilterCoeffs);
    // // 分解为4个2阶级联
    // // 第一级
    // hrFilterCoeffs[0] = 0.082672620462993381962313321764668216929;
    // hrFilterCoeffs[1] = -0.330690481851973527849253287058672867715;
    // hrFilterCoeffs[2] = 0.496035722777960319529455546216922812164;
    // hrFilterCoeffs[3] = 1.068353525316692920199557192972861230373;
    // hrFilterCoeffs[4] = 0.616247052563477892839216565334936603904;

    // // 第二级
    // hrFilterCoeffs[5] = 0.082672620462993381962313321764668216929;
    // hrFilterCoeffs[6] = -0.330690481851973527849253287058672867715;
    // hrFilterCoeffs[7] = 0.496035722777960319529455546216922812164;
    // hrFilterCoeffs[8] = 0.467522986836443554814479739434318616986;
    // hrFilterCoeffs[9] = 0.669152144730347231416089925915002822876;

    // // 第三级
    // hrFilterCoeffs[10] = 0.082672620462993381962313321764668216929;
    // hrFilterCoeffs[11] = -0.330690481851973527849253287058672867715;
    // hrFilterCoeffs[12] = 0.496035722777960319529455546216922812164;
    // hrFilterCoeffs[13] = 0.301543408042114380318565736160962842405;
    // hrFilterCoeffs[14] = 0.079244497522087828378367646564583992586;

    // // 第四级
    // hrFilterCoeffs[15] = 0.082672620462993381962313321764668216929;
    // hrFilterCoeffs[16] = -0.330690481851973527849253287058672867715;
    // hrFilterCoeffs[17] = 0.496035722777960319529455546216922812164;
    // hrFilterCoeffs[18] = 0.027107332720088535282787489677502890117;
    // hrFilterCoeffs[19] = 0.018137077030450785702919702657709422056;


    // arm_biquad_cascade_df2T_init_f64(&hrFilterInstance, 4,
    //                                  hrFilterCoeffs, hrFilterState);

    // 初始化呼吸滤波器 ，直接使用原始系数
    initIIRFilter(rrFilter, breathB, breathA, 4);

    // 初始化心率滤波器 ，直接使用原始系数
    initIIRFilter(hrFilter, heartB, heartA, 6);
}

VitalSignsDetector::~VitalSignsDetector()
{
    if (fftInput) delete[] fftInput;
    if (rrFilterState) delete[] rrFilterState;
    if (hrFilterState) delete[] hrFilterState;
}

void VitalSignsDetector::processFFTData(const std::vector<std::complex<float>>& fftData, int SubjectIdx)
{
    if (fftData.empty()) return;

    count++; //滑窗计数

    // 重构相位信号函数 FIFO方式
    float phase = std::arg(fftData[SubjectIdx]);
    if (phaseBuf.size() < WindSize) {
        phaseBuf.push_back(phase);
    } else {
        // 缓冲区已满，删除最早的，添加最新的
        phaseBuf.erase(phaseBuf.begin());
        phaseBuf.push_back(phase);
    }

    if (count < WindSize) return;

    // 解缠绕和相位差分
    std::vector<float> unwrap_phase=unwrap(phaseBuf);
    std::vector<float> diff_phase=diff(unwrap_phase);
    //std::vector<float> windowed_phase = DSPapplyHannWindow(diff_phase);

    //呼吸滤波
    //std::vector<float> rrPhase = applyDSPFilter(diff_phase, &rrFilterInstance);
    std::vector<float> rrPhase = applyIIRFilter(diff_phase, rrFilter);
    rrFreq = DSPcomputeFFT(DSPapplyWindow(rrPhase));

    //查找峰值时，在规定范围内
    int rrStart = static_cast<int>(0.1f * FFTNum / SamplingRate);
    int rrEnd = static_cast<int>(0.5f * FFTNum / SamplingRate);
    rrStart = std::max(1, rrStart);
    rrEnd = std::min((int)rrFreq.size() - 1, rrEnd);

    auto maxRRIt = std::max_element(rrFreq.begin() + rrStart, rrFreq.begin() + rrEnd);
    int maxIdxRR = std::distance(rrFreq.begin(), maxRRIt);


    rawBreathingRate = maxIdxRR * k_1;
    if (rawBreathingRate > 0) {
        smoothedBreathingRate = movingAverage(breathingRateBuffer, rawBreathingRate);
    }
    breathingRate=smoothedBreathingRate;


    //心跳滤波
    //std::vector<float> hrPhase = applyDSPFilter(diff_phase, &hrFilterInstance);
    std::vector<float> hrPhase = applyIIRFilter(diff_phase, hrFilter);
    //陷波滤波器
    // MultiNotchFilter multiNotchFilter;
    // double rrRate=rawBreathingRate/60;
    // std::vector<double> notchFrequencies = {rrRate * 2, rrRate * 3 , rrRate * 4};
    // double notchBandwidth = 0.005;  // 可调整带宽
    // initMultiNotchFilter(multiNotchFilter, notchFrequencies, SamplingRate, notchBandwidth);
    // auto phaseFiltered = applyMultiNotchFilter(hrPhase, multiNotchFilter);

    hrFreq = DSPcomputeFFT(DSPapplyWindow(hrPhase));

    int hrStart = static_cast<int>(0.9f * FFTNum / SamplingRate);
    int hrEnd = static_cast<int>(1.9f * FFTNum / SamplingRate);
    hrStart = std::max(1, hrStart);
    hrEnd = std::min((int)hrFreq.size() - 1, hrEnd);

    auto maxHRIt = std::max_element(hrFreq.begin() + hrStart, hrFreq.begin() + hrEnd);
    int maxIdxHR = std::distance(hrFreq.begin(), maxHRIt);

    double P = 20.0 * std::log10(hrFreq[maxIdxHR] / (std::accumulate(hrFreq.begin() + hrStart, hrFreq.begin() + hrEnd + 1, 0.0) / (hrStart - hrEnd + 1)));

    //if (P > 3)
    //{
    rawHeartRate =maxIdxHR * k_1; // (std::accumulate(hrFreq.begin() + hrStart, hrFreq.begin() + hrEnd + 1, 0.0)/ (hrStart - hrEnd + 1));//maxIdxHR * k_1;
    //}

    if (rawHeartRate > 0) {
        smoothedHeartRate = movingAverage(heartRateBuffer, rawHeartRate);
    }
    heartRate=smoothedHeartRate;

}

std::vector<float> VitalSignsDetector::unwrap(const std::vector<float>& phase)
{
    std::vector<float> unwrapped = phase;
    float cumulative = 0.0f;

    for (int i = 1; i < unwrapped.size(); i++) {
        float delta = unwrapped[i] - unwrapped[i - 1];

        // 如果相位跳跃超过π，进行补偿
        if (delta > M_PI) {
            cumulative -= 2.0f * M_PI;
        } else if (delta < -M_PI) {
            cumulative += 2.0f * M_PI;
        }

        unwrapped[i] = phase[i] + cumulative;
    }

    return unwrapped;
}

std::vector<float> VitalSignsDetector::diff(const std::vector<float>& signal)
{
    std::vector<float> differences;

    for (int i = 1; i < signal.size(); i++) {
        differences.push_back((signal[i] - signal[i - 1])*100);
    }

    return differences;
}



void VitalSignsDetector::designButterworthBandpass(float fcLow, float fcHigh, float32_t* coeffs)
{
    // 归一化频率
    float wLow = 2.0f * fcLow / SamplingRate;
    float wHigh = 2.0f * fcHigh / SamplingRate;

    wLow = std::max(0.001f, std::min(0.999f, wLow));
    wHigh = std::max(0.001f, std::min(0.999f, wHigh));

    if (wLow >= wHigh) {
        std::swap(wLow, wHigh);
    }

    // 第一级：高通滤波器 (截止频率 fcLow)
    float theta_low = M_PI * wLow;
    float sin_low = std::sin(theta_low);
    float cos_low = std::cos(theta_low);
    float alpha_low = std::sin(theta_low) / (2.0f * 0.707f);  // Q = 0.707

    float b0_hp = (1.0f + cos_low) / 2.0f;
    float b1_hp = -(1.0f + cos_low);
    float b2_hp = (1.0f + cos_low) / 2.0f;
    float a0_hp = 1.0f + alpha_low;
    float a1_hp = -2.0f * cos_low;
    float a2_hp = 1.0f - alpha_low;

    // 归一化
    coeffs[0] = b0_hp / a0_hp;
    coeffs[1] = b1_hp / a0_hp;
    coeffs[2] = b2_hp / a0_hp;
    coeffs[3] = a1_hp / a0_hp;
    coeffs[4] = a2_hp / a0_hp;

    // 第二级：低通滤波器 (截止频率 fcHigh)
    float theta_high = M_PI * wHigh;
    float sin_high = std::sin(theta_high);
    float cos_high = std::cos(theta_high);
    float alpha_high = std::sin(theta_high) / (2.0f * 0.707f);

    float b0_lp = (1.0f - cos_high) / 2.0f;
    float b1_lp = (1.0f - cos_high);
    float b2_lp = (1.0f - cos_high) / 2.0f;
    float a0_lp = 1.0f + alpha_high;
    float a1_lp = -2.0f * cos_high;
    float a2_lp = 1.0f - alpha_high;

    // 归一化
    coeffs[5] = b0_lp / a0_lp;
    coeffs[6] = b1_lp / a0_lp;
    coeffs[7] = b2_lp / a0_lp;
    coeffs[8] = a1_lp / a0_lp;
    coeffs[9] = a2_lp / a0_lp;
}

std::vector<float> VitalSignsDetector::applyDSPFilter(
    const std::vector<float>& signal, arm_biquad_cascade_df2T_instance_f64* filterInstance)
{
    std::vector<float> output(signal.size());

    // 转换为 float64_t 数组
    float64_t* inputData = new float64_t[signal.size()];
    float64_t* outputData = new float64_t[signal.size()];

    for (size_t i = 0; i < signal.size(); i++) {
        inputData[i] = signal[i];
    }

    // 应用级联双二阶滤波器
    arm_biquad_cascade_df2T_f64(filterInstance, inputData, outputData, signal.size());

    // 复制结果
    for (size_t i = 0; i < signal.size(); i++) {
        output[i] = outputData[i];
    }

    delete[] inputData;
    delete[] outputData;

    return output;
}

/*FFT设计模块*/
std::vector<float> VitalSignsDetector::DSPapplyWindow(const std::vector<float>& signal)
{
    std::vector<float> windowed = signal;
    int N = signal.size();

    for (int i = 0; i < N; i++) {
        float window = 0.54f - 0.46f* std::cos(2.0f * M_PI * i / (N - 1));
        windowed[i] *= window;
    }

    return windowed;
}


std::vector<float> VitalSignsDetector::DSPcomputeFFT(const std::vector<float>& signal)
{
    // 清空输入缓冲
    std::memset(fftInput, 0, FFTNum * 2 * sizeof(float32_t));

    // 填充实部数据
    size_t copySize = std::min((size_t)FFTNum, signal.size());
    for (size_t i = 0; i < copySize; i++) {
        fftInput[2 * i] = signal[i];
        fftInput[2 * i + 1] = 0.0f;
    }

    // 执行 FFT
    arm_cfft_f32(&fftInstance, fftInput, 0, 1);

    // 计算幅度谱
    std::vector<float> magnitude(FFTNum / 2, 0.0f);
    arm_cmplx_mag_f32(fftInput, magnitude.data(), FFTNum / 2);

    // 归一化
    float scale = 2.0f / FFTNum;
    arm_scale_f32(magnitude.data(), scale, magnitude.data(), FFTNum / 2);

    return magnitude;
}


// ========== IIR 滤波器实现 ==========

void VitalSignsDetector::initIIRFilter(IIRFilter& filter, const double* b, const double* a, int order)
{
    filter.order = order;
    filter.b.assign(b, b + order + 1);
    filter.a.assign(a, a + order + 1);
    filter.w.assign(order + 1, 0.0);  // Direct Form II 状态缓冲
}

void VitalSignsDetector::resetIIRFilter(IIRFilter& filter)
{
    std::fill(filter.w.begin(), filter.w.end(), 0.0);
}

// IIR Direct Form II Transposed 实现
// y[n] = b[0]*x[n] + w[0]
// w[0] = b[1]*x[n] - a[1]*y[n] + w[1]
// w[1] = b[2]*x[n] - a[2]*y[n] + w[2]
// ...
// w[N-1] = b[N]*x[n] - a[N]*y[n]
std::vector<float> VitalSignsDetector::applyIIRFilter(const std::vector<float>& signal, IIRFilter& filter)
{
    std::vector<float> output(signal.size());
    int N = filter.order;

    // 每次调用前重置状态（块处理模式）
    //resetIIRFilter(filter);

    for (size_t n = 0; n < signal.size(); n++) {
        double x = static_cast<double>(signal[n]);

        // 计算输出
        double y = filter.b[0] * x + filter.w[0];

        // 更新状态
        for (int i = 0 ; i < N - 1; i++) {
            filter.w[i] = filter.b[i + 1 ] * x - filter.a[i + 1] * y + filter.w[i + 1];
        }
        filter.w[N - 1] = filter.b[N] * x - filter.a[N] * y;

        output[n] = static_cast<float>(y);
    }

    return output;
}

// 平滑函数
float VitalSignsDetector::movingAverage(std::deque<float>& buffer, float newValue)
{
    // 检查异常值
    if (!buffer.empty()) {
        float mean = std::accumulate(buffer.begin(), buffer.end(), 0.0f) / buffer.size();
        if (isOutlier(newValue, mean, 2.5f)) {
            // 异常值：使用缓冲区的平均值替代
            newValue = mean;
        }
    }

    buffer.push_back(newValue);
    if (buffer.size() > BUFFER_SIZE) {
        buffer.pop_front();
    }

    // 计算平均值
    float sum = std::accumulate(buffer.begin(), buffer.end(), 0.0f);
    return sum / buffer.size();
}

// 异常值检测（3-sigma 规则）
bool VitalSignsDetector::isOutlier(float value, float mean, float threshold)
{
    if (std::abs(value - mean) > threshold * mean * 0.1f) {
        return true;
    }
    return false;
}

void VitalSignsDetector::clearAllBuffers()
{
    phaseBuf.clear();

    // 清空移动平均缓冲区
    heartRateBuffer.clear();
    breathingRateBuffer.clear();

    // 重置平滑后的数据
    smoothedHeartRate = 0.0f;
    smoothedBreathingRate = 0.0f;
}

/*陷波滤波器*/
void VitalSignsDetector::designNotchCoeffs(double fs, double f0, double bw, double* b, double* a)
{
    double r = 1.0 - 3.0 * bw;
    double theta = 2.0 * M_PI * f0 / fs;

    b[0] = 1.0;
    b[1] = -2.0 * cos(theta);
    b[2] = 1.0;

    a[0] = 1.0;
    a[1] = -2.0 * r * cos(theta);
    a[2] = r * r;
}

void VitalSignsDetector::initMultiNotchFilter(MultiNotchFilter& multiFilter,
                          const std::vector<double>& notchFreqs,
                          double fs, double bw) {
    multiFilter.filters.clear();
    for (auto f0 : notchFreqs) {
        IIRFilter filter;
        double b[3], a[3];
        designNotchCoeffs(fs, f0, bw, b, a);
        initIIRFilter(filter, b, a, 2);  // 二阶滤波器
        multiFilter.filters.push_back(filter);
    }
}

std::vector<float> VitalSignsDetector::applyMultiNotchFilter(const std::vector<float>& signal,
                                         MultiNotchFilter& multiFilter)
{
    std::vector<float> filtered = signal;
    for (auto& filter : multiFilter.filters) {
        filtered = applyIIRFilter(filtered, filter);
    }
    return filtered;
}

