#include "radar_processor.h"
#include <cstdlib>
#include <cstdio>
#include <cinttypes>
#include <new>
#include <cstring>
#include <cmath>

const uint8_t start_FFT[2] = {0x66,0xBB};
const uint8_t end_FFT[2] = {0xBB, 0x66};
const size_t FFT_SIZE = 80;


static uint32_t read_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3] <<  0);
}

static uint32_t read_le32(const uint8_t *p) {
    return ((uint32_t)p[3] << 24) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[1] <<  8) |
           ((uint32_t)p[0] <<  0);
}




RadarData* process_radar_data(const uint8_t *raw_data, size_t size) {
    if (!raw_data || size < 4) return nullptr;

    RadarData *result = new (std::nothrow)RadarData();
    if (!result) return nullptr;

    size_t dw_count = size / 4;
    uint32_t *dw = new (std::nothrow)uint32_t[dw_count];
    if (!dw) { delete result; return nullptr; }

    // 转换为 uint32_t 数组
    for (size_t i = 0; i < dw_count; i++) {
        dw[i] = read_be32(&raw_data[i * 4]);//read_be32(&raw_data[i * 4]);
    }

    // 提取 FFT 数据
    uint32_t *fft_raw = new uint32_t[dw_count];
    if (!fft_raw) { delete[] dw; delete result; return nullptr; }

    int dw_len = 0;
    int count_t = 0;

    for (size_t i = 1; i < dw_count; i++) {
        if ((int)(dw[i] >> 24) == 170 && (dw[i - 1] & 0xFF) == 85) { // 0x55AA
            dw_len = (int)(dw[i] & (uint32_t)0x7FF) - 1;

            if (i + dw_len + 1 < dw_count &&
                (int)(dw[i + dw_len + 1] & (uint32_t)0xFF) == 85) { // 0x55

                for (size_t j = i + 1; j < i + dw_len + 1; j++) {
                    fft_raw[count_t] = dw[j];
                    count_t++;
                }
            }
        }
    }

    // 转换为复数数据
    result->complex_data = new int16_t[2 * count_t];
    if (!result->complex_data) {
        delete[] fft_raw;
        delete[] dw;
        delete result;
        return nullptr;
    }

    for (int i = 0; i < count_t; i++) {
        int16_t real = (int16_t)((fft_raw[i] >> 16) & 0xFFFF);
        int16_t imag = (int16_t)(fft_raw[i] & 0xFFFF);
        result->complex_data[2 * i] = real;
        result->complex_data[2 * i + 1] = imag;
    }

    result->data_count = count_t;
    result->fft_points = dw_len;

    delete[] fft_raw;
    delete[] dw;

    return result;
}

int save_radar_csv(const RadarData *radar, const char *output_path) {
    if (!radar || !output_path) return -1;

    //FILE *f = fopen(output_path, "w");
    FILE *f;
    errno_t err = fopen_s(&f,output_path, "w");
    if (err !=0) return -1;

    for (size_t i = 0; i < 2 * radar->data_count; i++) {
        int count = 0;

        if (radar->complex_data[i] == (int16_t)0) {
            for (int j = i; j < (int)i + radar->fft_points && j < (int)(2 * radar->data_count); j++) {
                if (radar->complex_data[j] == (int16_t)0) {
                    count++;
                }
            }
        }

        if (count != radar->fft_points * 2) {
            fprintf(f, "%" PRId16, radar->complex_data[i]);
            if ((i + 1) % (radar->fft_points * 2) != 0) fputc(',', f);
            if ((i + 1) % (radar->fft_points * 2) == 0) fputc('\n', f);
        }
    }

    fclose(f);
    return 0;
}

void free_radar_data(RadarData *radar) {
    if (radar) {
        delete[] radar->complex_data;
        delete radar;
    }
}

RadarData* process_radar_frame(const uint8_t* frame, size_t size)
{
    if (!frame || size < 12) return nullptr;     // 至少 3 个DWORD
    if ((size % 4) != 0) return nullptr;

    const size_t dw_count = size / 4;
    if (dw_count < 3) return nullptr;

    // 读取 prev/header/tail 的关键DWORD
    const uint32_t prev_dw   = read_be32(frame + 0);
    const uint32_t header_dw = read_be32(frame + 4);

    // 严格帧头：header高字节0xAA，且prev最低字节0x55（对应 ...55 AA...）
    if (((header_dw >> 24) & 0xFFu) != 0xAAu) return nullptr;
    if ((prev_dw & 0xFFu) != 0x55u) return nullptr;

    int dw_len = int(header_dw & 0x7FFu) - 1;    // 与你现有逻辑一致
    // 这里范围按需调整：如果你知道固定点数，建议直接写 dw_len == 512 之类
    if (dw_len < 16 || dw_len > 4096) return nullptr;

    // 一帧DWORD总数必须 = dw_len + 3
    if (dw_count != size_t(dw_len + 3)) return nullptr;

    // tail校验：最后一个DWORD最低字节0x55
    const uint32_t tail_dw = read_be32(frame + (dw_count - 1) * 4);
    if ((tail_dw & 0xFFu) != 0x55u) return nullptr;

    RadarData* result = new (std::nothrow) RadarData();
    if (!result) return nullptr;

    result->fft_points = dw_len;
    result->data_count = dw_len;
    result->complex_data = new (std::nothrow) int16_t[2 * dw_len];
    if (!result->complex_data) {
        delete result;
        return nullptr;
    }

    // payload 从第2个DWORD开始（dw[2]..dw[dw_len+1]），每个DWORD = real(高16)+imag(低16)
    for (int i = 0; i < dw_len; ++i) {
        const uint32_t w = read_be32(frame + (2 + i) * 4);
        const int16_t real = (int16_t)((w >> 16) & 0xFFFFu);
        const int16_t imag = (int16_t)(w & 0xFFFFu);
        result->complex_data[2 * i]     = real;
        result->complex_data[2 * i + 1] = imag;
    }

    return result;
}

RadarData* process_fft_data_stream(const uint8_t* raw_data, size_t size)
{
    const size_t MIN_SIZE = 2 + 80 * 2 * sizeof(float) + 2;
    if (!raw_data || size < MIN_SIZE) return nullptr;

    uint16_t header = (uint16_t(raw_data[0]) << 8) | uint16_t(raw_data[1]);
    uint16_t tail = (uint16_t(raw_data[size-2]) << 8) | uint16_t(raw_data[size-1]);

    if (header != 0x66BB || tail != 0xBB66) {
        return nullptr;
    }

    RadarData* result = new (std::nothrow) RadarData();
    if (!result) return nullptr;

    const float* fftFloats = reinterpret_cast<const float*>(raw_data + 2);
    const int FFT_SIZE = 80;
    const float SCALE_FACTOR = 1000.f;

    result->complex_data = new (std::nothrow) int16_t[FFT_SIZE * 2];
    if (!result->complex_data) {
        delete result;
        return nullptr;
    }

    for (int i = 0; i < FFT_SIZE; ++i) {
        float real = fftFloats[2*i];
        float imag = fftFloats[2*i + 1];
        result->complex_data[2*i] = static_cast<int16_t>(std::round(real * SCALE_FACTOR));
        result->complex_data[2*i+1] = static_cast<int16_t>(std::round(imag * SCALE_FACTOR));
    }

    result->data_count = 1;
    result->fft_points = FFT_SIZE;

    return result;
}

static inline bool take_one_frame_from_stream(const uint8_t* raw, size_t size,
                                              size_t& cursor, const uint8_t*& framePtr, size_t& frameSize)
{
    const uint16_t FRAME_HEADER = 0x66BB;
    const uint16_t FRAME_TAIL = 0xBB66;
    const size_t FFT_DATA_SIZE = 80 * 2 * sizeof(float);  // 640字节
    const size_t TOTAL_FRAME_SIZE = 2 + FFT_DATA_SIZE + 2;  // 644字节

    while (cursor + TOTAL_FRAME_SIZE <= size) {
        // 查找帧头
        size_t pos = cursor;
        for (; pos + 1 < size; ++pos) {
            uint16_t header = (uint16_t(raw[pos]) << 8) | uint16_t(raw[pos + 1]);
            if (header == FRAME_HEADER) break;
        }

        if (pos + TOTAL_FRAME_SIZE > size) {
            cursor = pos;
            return false;  // 数据不足一帧
        }

        // 验证帧尾
        size_t tailPos = pos + 2 + FFT_DATA_SIZE;
        uint16_t tail = (uint16_t(raw[tailPos]) << 8) | uint16_t(raw[tailPos + 1]);

        if (tail == FRAME_TAIL) {
            framePtr = raw + pos;
            frameSize = TOTAL_FRAME_SIZE;
            cursor = pos + TOTAL_FRAME_SIZE;
            return true;
        }

        cursor = pos + 1;  // 帧尾不匹配，继续查找
    }

    return false;
}

RadarData* process_radar_stream(const uint8_t* raw, size_t size)
{
    if (!raw || size < 12) return nullptr;

    std::vector<int16_t> all;
    int fft_points = -1;

    size_t cursor = 0;
    const uint8_t* framePtr = nullptr;
    size_t frameSize = 0;

    while (take_one_frame_from_stream(raw, size, cursor, framePtr, frameSize)) {
        RadarData* one = process_fft_data_stream(framePtr, frameSize);
        if (!one) continue;

        if (fft_points == -1) fft_points = one->fft_points;
        if (one->fft_points != fft_points) {  // 长度不一致直接丢弃该帧
            free_radar_data(one);
            continue;
        }

        all.insert(all.end(), one->complex_data, one->complex_data + 2 * one->fft_points);
        free_radar_data(one);
    }

    if (fft_points <= 0 || all.empty()) return nullptr;

    RadarData* out = new (std::nothrow) RadarData();
    if (!out) return nullptr;

    out->fft_points = fft_points;
    out->data_count = int(all.size() / 2);
    out->complex_data = new (std::nothrow) int16_t[all.size()];
    if (!out->complex_data) { delete out; return nullptr; }

    std::memcpy(out->complex_data, all.data(), all.size() * sizeof(int16_t));
    return out;
}
