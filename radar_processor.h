#ifndef RADAR_PROCESSOR_H
#define RADAR_PROCESSOR_H

#include <cstdint>
#include <cstddef>
#include <QByteArray>


struct RadarData {
    int16_t *complex_data;
    size_t data_count;
    int fft_points;
};

//初始函数
RadarData* process_radar_data(const uint8_t *raw_data, size_t size);
int save_radar_csv(const RadarData *radar, const char *output_path);
void free_radar_data(RadarData *radar);

//处理整体包
RadarData* process_radar_stream(const uint8_t* raw, size_t size);

//实时处理专用
// 解析“单帧”（必须是已经组好的完整帧，且从 prevDWORD 开始对齐）
RadarData* process_radar_frame(const uint8_t* frame, size_t size);
RadarData* process_fft_data_stream(const uint8_t* raw_data, size_t size);






#endif // RADAR_PROCESSOR_H

