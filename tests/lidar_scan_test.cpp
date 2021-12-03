#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <cstdlib>
#include <map>
#include <vector>
#include <algorithm>

#include "ouster/types.h"
#include "ouster/lidar_scan.h"
#include "ouster/impl/lidar_scan_impl.h"

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

static const Table<ouster::sensor::ChanField, ouster::sensor::ChanFieldType, 4> legacy_field_slots{
    {{ouster::sensor::ChanField::RANGE, ouster::sensor::ChanFieldType::UINT32},
     {ouster::sensor::ChanField::SIGNAL, ouster::sensor::ChanFieldType::UINT32},
     {ouster::sensor::ChanField::NEAR_IR, ouster::sensor::ChanFieldType::UINT32},
     {ouster::sensor::ChanField::REFLECTIVITY, ouster::sensor::ChanFieldType::UINT32}
    }};

static const Table<ouster::sensor::ChanField, ouster::sensor::ChanFieldType, 7> dual_field_slots{{
    {ouster::sensor::ChanField::RANGE, ouster::sensor::ChanFieldType::UINT32},
    {ouster::sensor::ChanField::RANGE2, ouster::sensor::ChanFieldType::UINT32},
    {ouster::sensor::ChanField::SIGNAL, ouster::sensor::ChanFieldType::UINT16},
    {ouster::sensor::ChanField::SIGNAL2, ouster::sensor::ChanFieldType::UINT16},
    {ouster::sensor::ChanField::REFLECTIVITY, ouster::sensor::ChanFieldType::UINT8},
    {ouster::sensor::ChanField::REFLECTIVITY2, ouster::sensor::ChanFieldType::UINT8},
    {ouster::sensor::ChanField::NEAR_IR, ouster::sensor::ChanFieldType::UINT16}
    }};

static const Table<ouster::sensor::ChanField, ouster::sensor::ChanFieldType, 4> contrived_slots{{
    {ouster::sensor::ChanField::RANGE2, ouster::sensor::ChanFieldType::UINT32},
    {ouster::sensor::ChanField::SIGNAL2, ouster::sensor::ChanFieldType::UINT16},
    {ouster::sensor::ChanField::REFLECTIVITY, ouster::sensor::ChanFieldType::UINT8},
    {ouster::sensor::ChanField::NEAR_IR, ouster::sensor::ChanFieldType::UINT16}
    }};

void zero_check_fields(ouster::LidarScan& scan) {
    for(auto it = scan.begin(); it != scan.end(); it++) {
        const std::pair<ouster::sensor::ChanField, ouster::sensor::ChanFieldType> data = *it;
        
        switch(std::get<1>(data)) {
        case ouster::sensor::ChanFieldType::UINT64:
            EXPECT_TRUE((scan.field<uint64_t>(std::get<0>(data)) == 0).all());
            break;
        case ouster::sensor::ChanFieldType::UINT32:
            EXPECT_TRUE((scan.field<uint32_t>(std::get<0>(data)) == 0).all());
            break;
        case ouster::sensor::ChanFieldType::UINT16:
            EXPECT_TRUE((scan.field<uint16_t>(std::get<0>(data)) == 0).all());
            break;
        case ouster::sensor::ChanFieldType::UINT8:
            EXPECT_TRUE((scan.field<uint8_t>(std::get<0>(data)) == 0).all());
            break;
        case ouster::sensor::ChanFieldType::VOID:
            break;
        }
    }
}

TEST(LidarScan, EmptyConstructorInit) {
    auto scan = ouster::LidarScan();
    EXPECT_EQ(scan.w, 0);
    EXPECT_EQ(scan.h, 0);
    
    EXPECT_EQ(scan.frame_id, -1);
    
    EXPECT_EQ(scan.headers.size(), 0);

    EXPECT_EQ(scan.end() - scan.begin(), 0);

    zero_check_fields(scan);
}

TEST(LidarScan, LegacyConstructorInit) {
    int w = 10;
    int h = 20;
    auto scan = ouster::LidarScan(w, h);
    EXPECT_EQ(scan.w, w);
    EXPECT_EQ(scan.h, h);
    EXPECT_EQ(scan.frame_id, -1);

    int count = 0;
    int hit_count = 0;
    std::vector<ouster::sensor::ChanField> field_copy;
    for (auto item : legacy_field_slots) {
        field_copy.push_back(std::get<0>(item));
    }
    for(auto it = scan.begin(); it != scan.end(); it++) {
        const std::pair<ouster::sensor::ChanField, ouster::sensor::ChanFieldType> data = *it;
        auto result = std::find(
                     field_copy.begin(),
                     field_copy.end(),
                     std::get<0>(data));
        if(result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(legacy_field_slots.size(), count);

    EXPECT_TRUE((scan.status() == 0).all());
    EXPECT_TRUE((scan.measurement_id() == 0).all());
    EXPECT_TRUE((scan.timestamp() == 0).all());

    zero_check_fields(scan);
}

TEST(LidarScan, DualReturnConstructorInit) {
    int w = 40;
    int h = 60;
    auto scan = ouster::LidarScan(w, h,
                                  ouster::sensor::UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL);
    EXPECT_EQ(scan.w, w);
    EXPECT_EQ(scan.h, h);
    EXPECT_EQ(scan.frame_id, -1);

    int count = 0;
    int hit_count = 0;
    std::vector<ouster::sensor::ChanField> field_copy;
    for(auto item : dual_field_slots) {
        field_copy.push_back(std::get<0>(item));
    }
    for(auto it = scan.begin(); it != scan.end(); it++) {
        const std::pair<ouster::sensor::ChanField, ouster::sensor::ChanFieldType> data = *it;
        auto result = std::find(
                     field_copy.begin(),
                     field_copy.end(),
                     std::get<0>(data));
        if(result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(dual_field_slots.size(), count);

    EXPECT_TRUE((scan.status() == 0).all());
    EXPECT_TRUE((scan.measurement_id() == 0).all());
    EXPECT_TRUE((scan.timestamp() == 0).all());

    zero_check_fields(scan);
}

TEST(LidarScan, CustomFieldConstructorInit) {
    int w = 80;
    int h = 100;
    auto scan = ouster::LidarScan(w, h,
                                  contrived_slots.begin(), contrived_slots.end());
    EXPECT_EQ(scan.w, w);
    EXPECT_EQ(scan.h, h);
    EXPECT_EQ(scan.frame_id, -1);

    int count = 0;
    int hit_count = 0;
    std::vector<ouster::sensor::ChanField> field_copy;
    for(auto item : contrived_slots) {
        field_copy.push_back(std::get<0>(item));
    }

    for(auto it = scan.begin(); it != scan.end(); it++) {
        const std::pair<ouster::sensor::ChanField, ouster::sensor::ChanFieldType> data = *it;
        auto result = std::find(
                     field_copy.begin(),
                     field_copy.end(),
                     std::get<0>(data));
        if(result != field_copy.end()) {
            hit_count++;
            field_copy.erase(result);
        }
        count++;
    }
    EXPECT_EQ(field_copy.size(), 0);
    EXPECT_EQ(hit_count, count);
    EXPECT_EQ(contrived_slots.size(), count);

    EXPECT_TRUE((scan.status() == 0).all());
    EXPECT_TRUE((scan.measurement_id() == 0).all());
    EXPECT_TRUE((scan.timestamp() == 0).all());

    zero_check_fields(scan);
}
