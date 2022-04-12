#include <iostream>

#include <ouster/lidar_scan.h>

int main() {
    ouster::LidarScan scan{1024, 64};

    for (const auto& f : scan) {
        std::cout << "Field: " << ouster::sensor::to_string(f.first)
                  << std::endl;
    }

    return 0;
}
