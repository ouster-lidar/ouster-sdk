#include <getopt.h>

#include <algorithm>
#include <iostream>

#include "ouster/osf/util.h"

namespace OSF = ouster::OSF;

// Default frame_mode value to encode output OSF file
const static OSF::OSF_FRAME_MODE kDefaultFrameMode =
    OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_32;

const static std::string kOriginalFrameModeKeyword = "ORIGINAL";

static void show_usage(const std::string& name) {
    fprintf(stderr, R"(
Usage: %s [--frame-mode OSF_FRAME_MODE] [-h|--help] INPUT_FILE OUTPUT_FILE
    INPUT_FILE    : path to the source data file
    OUTPUT_FILE   : path to the destination data file

Converts input OSF file from OSF_STREAM mode to OSF_FRAMED by grouping lidar
scans into frames. Supports convertion to different lidar frame modes. By
default converts lidar frame modes to OSF_32 mode. See --frame-mode for other
choices.

Additionally can be used to process OSF_FRAMED input file to OSF_FRAMED output
file with lidar frame mode convertion.

Options:
    --frame-mode    Output frame mode. One of: OSF_32, OSF_40RI, OSF_40RN,
                    OSF_56, OSF_72. (default: OSF_32)

                    Use --frame-mode ORIGINAL to pass-through the unchanged
                    frame mode from input OSF file
    -h, --help      Show this help message
)",
            name.c_str());
}

// Compare strings, case insensitive
bool str_iequal(const std::string& str1, const std::string& str2) {
    return str1.size() == str2.size() &&
           equal(str1.begin(), str1.end(), str2.begin(),
                 [](int a, int b) { return toupper(a) == toupper(b); });
}

int main(int argc, char** argv) {
    int use_sensors_traj = 0;
    struct option options[] = {
        {"help", no_argument, 0, 'h'},
        {"use-sensors-trajectory", no_argument, &use_sensors_traj, 1},
        {"frame-mode", required_argument, 0, 'm'},
        {0, 0, 0, 0}};

    OSF::OSF_FRAME_MODE output_frame_mode = kDefaultFrameMode;

    int c;
    int options_index = 0;
    while ((c = getopt_long(argc, argv, "hm:", options, &options_index)) !=
           -1) {
        switch (c) {
            case 'm':
                if (str_iequal(optarg, kOriginalFrameModeKeyword)) {
                    // Use output frame mode from input file
                    output_frame_mode =
                        OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_UNKNOWN;
                } else {
                    // Parse --frame-mode
                    output_frame_mode = OSF::osf_frame_mode_of_string(optarg);
                    if (!output_frame_mode) {
                        std::cout << "ERROR: Unknown frame mode: " << optarg
                                  << std::endl;
                        return EXIT_FAILURE;
                    };
                }
                break;
            case 'h':
                show_usage(argv[0]);
                return EXIT_FAILURE;
            case '?':
                // ERROR, getopt shows enough info
                return EXIT_FAILURE;
        }
    }

    if (argc - optind != 2) {
        show_usage(argv[0]);
        return EXIT_FAILURE;
    }

    std::string in_file = argv[optind];
    std::string out_file = argv[optind + 1];

    ouster::OSF::rechunk(in_file, out_file, output_frame_mode,
                         !static_cast<bool>(use_sensors_traj));

    return EXIT_SUCCESS;
}
