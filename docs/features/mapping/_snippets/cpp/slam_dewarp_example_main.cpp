#include <iostream>

#include "slam_dewarp_example.cpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: slam_dewarp_example <osf_file>\n";
        return 0;
    }

    auto points = slam_dewarp_once(argv[1]);
    std::cout << "Generated " << points.size() << " points\n";
    return 0;
}
