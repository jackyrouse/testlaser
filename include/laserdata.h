//
// Created by jacky on 18-6-28.
//

#ifndef TESTLASER_LASERDATA_H
#define TESTLASER_LASERDATA_H

namespace LASER_PandCspace
{

static const int kLASERItemRepositorySize = 10; // Item buffer size.
static const int kLASERItemsToProduce = 1000;   // How many items we plan to produce.

struct lheaderinfo
{
    uint32_t seq;
    timeval stamp;
    std::string frame_id;
};
typedef lheaderinfo LHeaderInfo;

struct LASER_Message
{
    LHeaderInfo header;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[720];
    float intensities[720];
    int rangessize = 720;
    int intensitiessize = 720;
};
typedef LASER_Message LASERMessage;

struct LASERItemRepository
{
    LASERMessage item_buffer[kLASERItemRepositorySize]; // 产品缓冲区, 配合 read_position 和 write_position 模型环形队列.
    size_t read_position; // 消费者读取产品位置.
    size_t write_position; // 生产者写入产品位置.
    std::mutex mtx; // 互斥量,保护产品缓冲区
    std::condition_variable repo_not_full; // 条件变量, 指示产品缓冲区不为满.
    std::condition_variable repo_not_empty; // 条件变量, 指示产品缓冲区不为空.
} gLASERItemRepository; // 产品库全局变量, 生产者和消费者操作该变量.
typedef struct LASERItemRepository LASERItemRepository;

}
#endif //TESTLASER_LASERDATA_H
