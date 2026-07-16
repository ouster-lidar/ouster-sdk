#include <dirent.h>
#include <gtest/gtest.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/impl/open_source_impl.h"
#include "ouster/osf/osf_frame_set_source.h"

#define main record_sensor_example_main
#include "record_sensor.cpp"
#undef main

using namespace ouster::sdk;

namespace {

std::string env_or_empty(const char* key) {
    const char* value = std::getenv(key);
    return (value && *value) ? std::string(value) : std::string{};
}

std::string data_root() {
    const auto from_env = env_or_empty("DATA_DIR");
    if (!from_env.empty()) {
        return from_env;
    }
#ifdef OUSTER_SDK_SOURCE_DIR
    return std::string(OUSTER_SDK_SOURCE_DIR) + "/tests";
#else
    throw std::runtime_error(
        "DATA_DIR environment variable not set and "
        "OUSTER_SDK_SOURCE_DIR is undefined.");
#endif
}

struct FixturePaths {
    std::string pcap;
    std::string meta;
    std::string osf;
};

FixturePaths fixtures() {
    const auto root = data_root();
    return {
        root + "/pcaps/OS-1-128_v2.3.0_1024x10_lb_n3.pcap",
        root + "/pcaps/OS-1-128_v2.3.0_1024x10.json",
        root + "/osfs/single_scan_016.osf",
    };
}

bool file_exists(const std::string& p) {
    std::ifstream f(p.c_str(), std::ios::binary);
    return f.good();
}

std::string capture_stdout(const std::function<void()>& action) {
    std::ostringstream buffer;
    auto* original = std::cout.rdbuf(buffer.rdbuf());
    try {
        action();
    } catch (...) {
        std::cout.rdbuf(original);
        throw;
    }
    std::cout.rdbuf(original);
    return buffer.str();
}

class TemporaryDirectory {
   public:
    TemporaryDirectory() {
        std::string pattern = "/tmp/record-snippet-XXXXXX";
        std::vector<char> buf(pattern.begin(), pattern.end());
        buf.push_back('\0');
        char* created = mkdtemp(buf.data());
        if (!created) {
            throw std::runtime_error("mkdtemp failed");
        }
        path_ = created;
    }

    ~TemporaryDirectory() {
        remove_recursive(path_);
    }

    const std::string& path() const {
        return path_;
    }

   private:
    static void remove_recursive(const std::string& dir) {
        DIR* d = opendir(dir.c_str());
        if (d) {
            struct dirent* entry;
            while ((entry = readdir(d)) != nullptr) {
                if (std::strcmp(entry->d_name, ".") == 0 || std::strcmp(entry->d_name, "..") == 0) {
                    continue;
                }
                std::string child = dir + "/" + entry->d_name;
                struct stat st;
                if (stat(child.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
                    remove_recursive(child);
                } else {
                    std::remove(child.c_str());
                }
            }
            closedir(d);
        }
        rmdir(dir.c_str());
    }

    std::string path_;
};

class ScopedCurrentPath {
   public:
    explicit ScopedCurrentPath(const std::string& target) {
        previous_ = current_path();
        change_directory(target);
    }

    ~ScopedCurrentPath() {
        change_directory(previous_);
    }

   private:
    static std::string current_path() {
        char buffer[PATH_MAX];
        if (!::getcwd(buffer, sizeof(buffer))) {
            throw std::runtime_error("getcwd failed");
        }
        return std::string(buffer);
    }

    static void change_directory(const std::string& path) {
        if (::chdir(path.c_str()) != 0) {
            throw std::runtime_error("chdir failed");
        }
    }

    std::string previous_;
};

std::string join_path(const std::string& base, const std::string& leaf) {
    if (leaf.empty()) {
        return leaf;
    }
    if (!leaf.empty() &&
        (leaf[0] == '/' || leaf[0] == '\\' || (leaf.size() > 1 && leaf[1] == ':'))) {
        return leaf;
    }
    if (base.empty()) {
        return leaf;
    }
    const char sep = '/';
    if (base.back() == '/' || base.back() == '\\') {
        return base + leaf;
    }
    return base + sep + leaf;
}

std::size_t file_size(const std::string& path) {
    std::ifstream f(path.c_str(), std::ios::binary);
    if (!f.good()) {
        return 0;
    }
    f.seekg(0, std::ios::end);
    return static_cast<std::size_t>(f.tellg());
}

std::string extract_path_after(const std::string& output, const std::string& marker) {
    const std::size_t pos = output.find(marker);
    if (pos == std::string::npos) {
        return {};
    }
    std::size_t start = pos + marker.size();
    while (start < output.size() && (output[start] == ' ' || output[start] == '\t')) {
        ++start;
    }
    std::size_t end = output.find('\n', start);
    if (end == std::string::npos) {
        end = output.size();
    }
    return output.substr(start, end - start);
}

std::string sanitize_path_token(const std::string& raw) {
    std::size_t end = raw.find_first_of(" \t\r\n(");
    if (end == std::string::npos) {
        return raw;
    }
    return raw.substr(0, end);
}

}  // namespace

// Runs when SENSOR_HOSTNAME is set
TEST(RecordSensorSnippet, RecordSessionCreatesArtifacts) {
    const char* sensor_host = std::getenv("SENSOR_HOSTNAME");
    if (!sensor_host || std::string(sensor_host).empty()) {
        GTEST_SKIP() << "SENSOR_HOSTNAME not set; skipping live sensor test.";
    }

    TemporaryDirectory tmp;
    ScopedCurrentPath change_dir(tmp.path());

    const auto output =
        capture_stdout([&]() { ouster::docs::record_sensor_session(sensor_host, 7502, 7503, 1); });

    const std::string json_rel = extract_path_after(output, "Saving sensor info to:");
    const std::string pcap_rel = extract_path_after(output, "Writing to:");
    ASSERT_FALSE(json_rel.empty()) << "Metadata file path not found in output.";
    ASSERT_FALSE(pcap_rel.empty()) << "PCAP file path not found in output.";
    const std::string json_path = join_path(tmp.path(), sanitize_path_token(json_rel));
    const std::string pcap_path = join_path(tmp.path(), sanitize_path_token(pcap_rel));

    ASSERT_TRUE(file_exists(json_path)) << "Metadata json not created: " << json_path;
    ASSERT_TRUE(file_exists(pcap_path)) << "PCAP file not created: " << pcap_path;

    EXPECT_GT(file_size(json_path), 0u) << "Recorded metadata json is empty: " << json_path;
    EXPECT_GT(file_size(pcap_path), 0u) << "Recorded pcap is empty: " << pcap_path;

    EXPECT_NE(output.find("Captured"), std::string::npos)
        << "Expected capture summary not found in stdout. Output: " << output;
}

TEST(RecordSensorSnippet, PcapRecordSessionLoadsMetadata) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.pcap)) << "Missing fixture: " << paths.pcap;
    ASSERT_TRUE(file_exists(paths.meta)) << "Missing fixture: " << paths.meta;

    auto source_any = ouster::docs::open_pcap_source_with_metadata(paths.pcap, {paths.meta});

    const auto& infos = source_any.sensor_info();
    ASSERT_FALSE(infos.empty()) << "Expected metadata when opening PCAP source";
    ASSERT_TRUE(infos.front()) << "First metadata entry is null";

    auto child = source_any.child();
    auto pcap_src = std::dynamic_pointer_cast<pcap::PcapPacketSource>(child);
    ASSERT_TRUE(pcap_src) << "PCAP packet source not detected";

    size_t packet_samples = 0;
    for (auto it = source_any.begin(); it != source_any.end(); ++it) {
        const auto& entry = *it;
        const auto& pkt_ptr = entry.second;
        if (!pkt_ptr) {
            continue;
        }
        ++packet_samples;
        if (packet_samples >= 10) {
            break;
        }
    }
    EXPECT_GT(packet_samples, 0u) << "Failed to pull packets from PCAP packet source";
}

// Runs when SENSOR_HOSTNAME is set
TEST(RecordSensorSnippet, SaveOsfCreatesFile) {
    const char* sensor_host = std::getenv("SENSOR_HOSTNAME");
    if (!sensor_host || std::string(sensor_host).empty()) {
        GTEST_SKIP() << "SENSOR_HOSTNAME not set; skipping live sensor test.";
    }
    TemporaryDirectory tmp;
    const std::string output_file = join_path(tmp.path(), "recorded_session.osf");

    EXPECT_NO_THROW({ ouster::docs::record_sensor_session_save_osf(sensor_host, output_file, 2); });

    ASSERT_TRUE(file_exists(output_file))
        << "record_sensor_session_save_osf failed to create: " << output_file;
    EXPECT_GT(file_size(output_file), 0u) << "Output OSF file is empty: " << output_file;

    try {
        const osf::OsfFrameSetSource verify(output_file);
        const bool has_frame = verify.begin() != verify.end();
        EXPECT_TRUE(has_frame) << "OSF output contains no frames.";
    } catch (const std::exception& ex) {
        FAIL() << "Failed to open generated OSF file: " << ex.what();
    }
}

TEST(RecordSensorSnippet, SliceOsfProducesSubset) {
    const auto paths = fixtures();
    ASSERT_TRUE(file_exists(paths.osf)) << "Missing fixture: " << paths.osf;

    TemporaryDirectory tmp;
    const std::string working_copy = join_path(tmp.path(), "input.osf");

    {
        std::ifstream src(paths.osf.c_str(), std::ios::binary);
        ASSERT_TRUE(src.good()) << "Failed to open fixture OSF";
        std::ofstream dst(working_copy.c_str(), std::ios::binary);
        ASSERT_TRUE(dst.good()) << "Failed to create working copy";
        dst << src.rdbuf();
    }

    EXPECT_NO_THROW({ ouster::docs::osf_slice_frames(working_copy); });

    const std::string sliced = join_path(tmp.path(), "input_sliced.osf");
    ASSERT_TRUE(file_exists(sliced))
        << "osf_slice_frames did not produce the sliced file: " << sliced;
    EXPECT_GT(file_size(sliced), 0u) << "Sliced OSF file is empty: " << sliced;

    bool validated_fields = false;
    try {
        const osf::OsfFrameSetSource sliced_source(sliced);
        const bool has_frame = sliced_source.begin() != sliced_source.end();
        EXPECT_TRUE(has_frame) << "Sliced OSF contains no frames.";
        const std::vector<std::string> expected_fields = {"RANGE", "SIGNAL", "REFLECTIVITY"};
        std::vector<std::string> expected_sorted = expected_fields;
        std::sort(expected_sorted.begin(), expected_sorted.end());
        for (const auto& frame_set : sliced_source) {
            for (const auto& frame_ptr : frame_set) {
                if (!frame_ptr) {
                    continue;
                }
                const auto types = frame_ptr->field_types();
                std::vector<std::string> actual_fields;
                actual_fields.reserve(types.size());
                for (const auto& type : types) {
                    actual_fields.push_back(type.name);
                }
                auto actual_sorted = actual_fields;
                std::sort(actual_sorted.begin(), actual_sorted.end());
                EXPECT_EQ(actual_sorted, expected_sorted)
                    << "Sliced OSF contains unexpected fields.";
                validated_fields = true;
                break;
            }
            if (validated_fields) {
                break;
            }
        }
    } catch (const std::exception& ex) {
        FAIL() << "Failed to open sliced OSF: " << ex.what();
    }
    EXPECT_TRUE(validated_fields) << "Unable to validate fields written to sliced OSF.";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
