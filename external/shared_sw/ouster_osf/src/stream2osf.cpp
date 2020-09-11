#include <iostream>

#include "ouster/osf/util.h"

static void show_usage(const std::string& name) {
    std::cerr << "Usage: " << name << " SOURCE_FB DUMP_DIR\n"
              << "SOURCE_FB : path to the source data file\n"
              << "DUMP_DIR   : path to the dump data folder\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        show_usage(argv[0]);
        return 0;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        }
    }

    ouster::OSF::stream2LegacyOsf(argv[1], argv[2]);
}
