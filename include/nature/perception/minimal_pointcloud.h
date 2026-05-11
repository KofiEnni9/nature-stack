#pragma once
#include <vector>
namespace nature {
namespace msg {

    struct Point32 {
        float x, y, z;
    };
    struct Channel {
            std::string name;
            std::vector<float> values;
        };
        struct PointCloud {
            std::vector<Point32> points;
            std::vector<Channel> channels;
        };
    struct OccupancyGrid {
        struct Info {
            float resolution;
            uint32_t width;
            uint32_t height;
            struct {
                struct {
                    float x, y, z, w;
                } orientation;
                struct {
                    float x, y, z;
                } position;
            } origin;
        };
        Info info;
        std::vector<uint8_t> data;
        struct {
            std::string frame_id;
        } header;
    };
}
}